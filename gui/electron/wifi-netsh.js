const { execFile } = require('child_process');
const fs = require('fs');
const os = require('os');
const path = require('path');

const split = (line) => {
  const t = line.trim();
  const i = t.indexOf(':');
  return i < 0 ? null : [t.slice(0, i).trim(), t.slice(i + 1).trim()];
};
const pct = (v) => (/^\d+\s*%$/.test(v) ? parseInt(v, 10) : null);

function parseNetworks(out) {
  const nets = [];
  let cur = null;
  for (const line of out.split(/\r?\n/)) {
    const kv = split(line);
    if (!kv) continue;
    const [label, val] = kv;
    if (label.startsWith('SSID ')) {
      if (cur?.ssid) nets.push(cur);
      cur = { ssid: val, signal: null, secured: true, saved: false, active: false };
      continue;
    }
    if (!cur) continue;
    if (label === 'Authentication' || label === '인증') cur.secured = !(val === 'Open' || val === '오픈');
    if (cur.signal === null && pct(val) !== null) cur.signal = pct(val);
  }
  if (cur?.ssid) nets.push(cur);
  return nets.map((n) => ({ ...n, signal: n.signal ?? 0 }));
}

function parseInterface(out) {
  let connected = false;
  let ssid = '';
  let signal = 0;
  for (const line of out.split(/\r?\n/)) {
    const kv = split(line);
    if (!kv) continue;
    const [label, val] = kv;
    if (label === 'State' || label === '상태') connected = val === 'connected' || val === '연결됨';
    else if (/^ssid$/i.test(label)) ssid = val;
    else if ((label === 'Signal' || label === '신호') && !signal) signal = pct(val) ?? 0;
  }
  return connected && ssid ? { ssid, signal, secured: true, saved: false, active: true } : null;
}

const esc = (s) => s.replace(/&/g, '&amp;').replace(/</g, '&lt;').replace(/>/g, '&gt;').replace(/"/g, '&quot;').replace(/'/g, '&apos;');

function profileXml(ssid, key) {
  const s = esc(ssid);
  const sec = key == null
    ? '<authEncryption><authentication>open</authentication><encryption>none</encryption><useOneX>false</useOneX></authEncryption>'
    : '<authEncryption><authentication>WPA2PSK</authentication><encryption>AES</encryption><useOneX>false</useOneX></authEncryption>'
      + `<sharedKey><keyType>passPhrase</keyType><protected>false</protected><keyMaterial>${esc(key)}</keyMaterial></sharedKey>`;
  return `<?xml version="1.0"?>
<WLANProfile xmlns="http://www.microsoft.com/networking/WLAN/profile/v1">
  <name>${s}</name>
  <SSIDConfig><SSID><name>${s}</name></SSID></SSIDConfig>
  <connectionType>ESS</connectionType>
  <connectionMode>manual</connectionMode>
  <MSM><security>${sec}</security></MSM>
</WLANProfile>`;
}

function decode(buf) {
  try { return new TextDecoder('utf-8', { fatal: true }).decode(buf); } catch { return new TextDecoder('euc-kr').decode(buf); }
}
function netsh(args) {
  return new Promise((resolve, reject) => {
    execFile('netsh', args, { encoding: 'buffer', windowsHide: true, timeout: 30000 }, (err, stdout, stderr) => {
      if (err) return reject(new Error(decode(stderr).trim() || decode(stdout).trim() || 'netsh 명령 실패'));
      resolve(decode(stdout));
    });
  });
}

async function scan() {
  return parseNetworks(await netsh(['wlan', 'show', 'networks', 'mode=bssid']));
}

async function connect(ssid, password) {
  const file = path.join(os.tmpdir(), `rbq-wifi-profile-${process.pid}.xml`);
  fs.writeFileSync(file, profileXml(ssid, password ?? null));
  try { await netsh(['wlan', 'add', 'profile', `filename=${file}`]); } finally { fs.rmSync(file, { force: true }); }
  await netsh(['wlan', 'connect', `name=${ssid}`]);
  return null;
}

async function current() {
  return parseInterface(await netsh(['wlan', 'show', 'interfaces']));
}

module.exports = { scan, connect, current, parseNetworks, parseInterface, profileXml };
