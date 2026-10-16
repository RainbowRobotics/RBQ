const dbus = require('dbus-next');

const NM = 'org.freedesktop.NetworkManager';
const PROPS = 'org.freedesktop.DBus.Properties';
const DEVICE_WIFI = 2;
const AC_ACTIVATED = 2;
const AC_DEACTIVATED = 4;

let bus = null;
const sys = () => (bus ??= dbus.systemBus());
const obj = (path) => sys().getProxyObject(NM, path);
async function get(path, iface, prop) {
  return (await (await obj(path)).getInterface(PROPS).Get(iface, prop)).value;
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));
const ssidOf = (bytes) => Buffer.from(bytes || []).toString('utf8');

async function wifiDevice() {
  const nm = (await obj('/org/freedesktop/NetworkManager')).getInterface(NM);
  for (const d of await nm.GetDevices()) {
    if ((await get(d, `${NM}.Device`, 'DeviceType')) === DEVICE_WIFI) return d;
  }
  throw new Error('이 시스템에서 WiFi 제어를 사용할 수 없습니다');
}

async function accessPoints(dev) {
  const w = (await obj(dev)).getInterface(`${NM}.Device.Wireless`);
  const active = await get(dev, `${NM}.Device.Wireless`, 'ActiveAccessPoint');
  const out = [];
  for (const ap of await w.GetAllAccessPoints()) {
    try {
      const all = (await (await obj(ap)).getInterface(PROPS).GetAll(`${NM}.AccessPoint`));
      const ssid = ssidOf(all.Ssid.value);
      if (!ssid) continue;
      const secured = !!(all.Flags.value & 1) || all.WpaFlags.value !== 0 || all.RsnFlags.value !== 0;
      out.push({ path: ap, ssid, signal: all.Strength.value, secured, active: ap === active });
    } catch { }
  }
  return out;
}

async function savedProfiles() {
  const settings = (await obj('/org/freedesktop/NetworkManager/Settings')).getInterface(`${NM}.Settings`);
  const bySsid = new Map();
  for (const c of await settings.ListConnections()) {
    try {
      const s = await (await obj(c)).getInterface(`${NM}.Settings.Connection`).GetSettings();
      const w = s['802-11-wireless'];
      if (!w) continue;
      const ssid = ssidOf(w.ssid?.value);
      bySsid.set(ssid, [...(bySsid.get(ssid) || []), c]);
    } catch { }
  }
  return bySsid;
}

async function scan() {
  const dev = await wifiDevice();
  const w = (await obj(dev)).getInterface(`${NM}.Device.Wireless`);
  const before = await get(dev, `${NM}.Device.Wireless`, 'LastScan');
  let requested = true;
  try { await w.RequestScan({}); } catch { requested = false; }
  for (let i = 0; requested && i < 50; i++) {
    await sleep(200);
    if ((await get(dev, `${NM}.Device.Wireless`, 'LastScan')) !== before) break;
  }
  const aps = await accessPoints(dev);
  const saved = await savedProfiles();
  return aps.map(({ ssid, signal, secured, active }) => ({ ssid, signal, secured, saved: saved.has(ssid), active }));
}

async function connect(ssid, password) {
  const dev = await wifiDevice();
  const ap = (await accessPoints(dev)).filter((a) => a.ssid === ssid).sort((a, b) => b.signal - a.signal)[0];
  if (!ap) throw new Error(`No network with SSID '${ssid}' found.`);
  const nm = (await obj('/org/freedesktop/NetworkManager')).getInterface(NM);
  const profiles = (await savedProfiles()).get(ssid) || [];
  let acPath;
  if (password == null && profiles.length) {
    acPath = await nm.ActivateConnection(profiles[0], dev, ap.path);
  } else {
    if (password != null) {
      for (const p of profiles) {
        try { await (await obj(p)).getInterface(`${NM}.Settings.Connection`).Delete(); } catch { }
      }
    }
    const conn = {
      connection: { id: new dbus.Variant('s', ssid), type: new dbus.Variant('s', '802-11-wireless') },
      '802-11-wireless': { ssid: new dbus.Variant('ay', Buffer.from(ssid)) },
    };
    if (password != null) {
      conn['802-11-wireless-security'] = {
        'key-mgmt': new dbus.Variant('s', 'wpa-psk'),
        psk: new dbus.Variant('s', password),
      };
    }
    [, acPath] = await nm.AddAndActivateConnection(conn, dev, ap.path);
  }
  for (let i = 0; i < 125; i++) {
    let state;
    try { state = await get(acPath, `${NM}.Connection.Active`, 'State'); } catch { state = AC_DEACTIVATED; }
    if (state === AC_ACTIVATED) return null;
    if (state === AC_DEACTIVATED) throw new Error('Secrets were required, but not provided.');
    await sleep(200);
  }
  throw new Error('Timeout expired.');
}

async function current() {
  const dev = await wifiDevice();
  const active = (await accessPoints(dev)).find((a) => a.active);
  if (!active) return null;
  let ip;
  try {
    const cfg = await get(dev, `${NM}.Device`, 'Ip4Config');
    ip = (await get(cfg, `${NM}.IP4Config`, 'AddressData'))[0]?.address?.value;
  } catch { }
  return { ssid: active.ssid, signal: active.signal, secured: true, saved: true, active: true, ...(ip ? { ip } : {}) };
}

module.exports = { scan, connect, current };
