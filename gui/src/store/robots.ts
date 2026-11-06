import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import { fileBackedStorage } from '@/lib/fileKv';
import { directTarget, type Target } from '@/lib/connectTarget';
import { connectNow, currentTarget, disconnectAll } from '@/lib/connectNow';
import { isDemo } from '@/lib/demoFlag';
import { rest } from '@/lib/rest';
import { probeLan, ROBOT_LAN_IP } from '@/lib/robotScan';
import { currentSsid } from '@/lib/currentSsid';
import { isDesktop, wifiCurrent } from '@/lib/desktopBridge';
import { switchToRobotWifi } from '@/lib/robotWifi';
import { useWifi } from '@/store/wifi';
import { restBase } from '@/lib/endpoints';
import { remote } from '@/modules/registry';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useViewport } from '@/store/viewport';
import { useSettings } from '@/store/settings';
import { setRobotPasswordSource, ipKey } from '@/lib/auth';
import { t } from '@/lib/i18n';
import { otaProbe, visionOtaIp } from '@/lib/ota';
import { LEGACY_SYSTEM } from '@/lib/firmwareRelease';

export type RobotProfile = {
  serial: string;
  name: string;
  lan?: { ip: string; visionIp?: string; ssid?: string };
  wan?: { ip: string; token?: string };
  lastVia: 'lan' | 'wan' | (string & {});
  lastSeenAt: number;
  robotVersion?: RobotVersion;
  system?: string;
  visionSystem?: string;
};
export type RobotVersion = string;

function applyRobotVersion(p: RobotProfile | null) {
  useSettings.getState().setRobotVersion(p?.robotVersion ?? 'none');
}

export const SIM_SERIAL = 'sim';
const LOCAL_SIM_NAME = 'PC 시뮬레이터';
const OLD_LOCAL_SIM_NAME = '이 PC 시뮬';
export const SIM_PROFILE: RobotProfile = { serial: SIM_SERIAL, name: '시뮬레이터', lastVia: 'lan', lastSeenAt: 0 };
export const profileName = (p: { name: string }): string => (p.name === SIM_PROFILE.name || p.name === LOCAL_SIM_NAME ? t(p.name) : p.name);

const isSharedLanIp = (ip: string) => ip === ROBOT_LAN_IP;
export function noSerialKey(ip: string, ssid: string | null): string {
  return isSharedLanIp(ip) && ssid ? `ssid:${ssid}` : `addr:${ip}`;
}

export function ssidOfKey(serial: string): string | null {
  return serial.startsWith('ssid:') ? serial.slice(5) : null;
}

export const isRemoteVia = (v: string | undefined): boolean => !!v && v !== 'lan' && v !== 'wan';

export function targetFor(p: RobotProfile): Target | null {
  if (p.serial === SIM_SERIAL) return null;
  if (isRemoteVia(p.lastVia)) { const t = remote.robots?.target(p); if (t) return t; }
  if (p.lastVia === 'wan' && p.wan) { const r = directTarget(p.wan.ip, { token: p.wan.token, label: p.name }); return r.ok ? r.target : null; }
  if (p.lan) { const r = directTarget(p.lan.ip, { label: p.name }); return r.ok ? r.target : null; }
  return null;
}

export function migrateLegacy(_lanIp: string, wanIp: string, _robotIp: string, token: string): RobotProfile | null {
  const wan = wanIp.trim();
  if (!wan) return null;
  return { serial: `pending:${wan}`, name: t('내 로봇'), wan: { ip: wan, token: token || undefined }, lastVia: 'wan', lastSeenAt: Date.now() };
}

export const byRecent = (list: RobotProfile[]): RobotProfile[] => [...list].sort((a, b) => b.lastSeenAt - a.lastSeenAt);

async function isProfileAtLan(p: RobotProfile): Promise<boolean> {
  if (!p.lan) return false;
  const found = await probeLan(p.lan.ip, { base: restBase(p.lan.ip) });
  if (found === null) return false;
  return (found || noSerialKey(p.lan.ip, await currentSsid())) === p.serial;
}

export type WifiMove = 'auto' | 'ask' | 'skip';

async function onOtherWifi(ssid: string): Promise<boolean> {
  if (!isDesktop()) return false;
  const cur = await wifiCurrent().catch(() => null);
  return !!cur?.ssid && cur.ssid !== ssid;
}

async function pickVia(p: RobotProfile): Promise<RobotProfile['lastVia']> {
  const rv = remote.robots?.has(p) ? remote.robots.lastVia : null;
  if (p.lan && rv) return (await isProfileAtLan(p)) ? 'lan' : rv;
  if (rv) return rv;
  if (p.lan) return 'lan';
  return p.wan ? 'wan' : p.lastVia;
}

type RobotsState = {
  local: RobotProfile[];
  currentSerial: string | null;
  codeNeeded: boolean;
  wifiSwitching: string | null;
  wifiAsk: { serial: string; name: string; ssid: string } | null;
  wifiAskedNo: boolean;
  answerWifiAsk: (ok: boolean) => void;
  lastRealSerial: string | null;
  selectIssue: { name: string; ssid: string; kind: 'off' | 'wifi_pw' | 'wifi_fail' } | null;
  switching: boolean;
  passwords: Record<string, string>;
  setPassword: (key: string, password: string) => void;
  clearSelectIssue: () => void;
  select: (serial: string, wifi?: WifiMove) => Promise<void>;
  selectInner: (serial: string, wifi?: WifiMove) => Promise<void>;
  leaveSim: () => void;
  saveConnected: (name?: string) => Promise<boolean>;
  addLan: (serial: string, ip?: string) => Promise<void>;
  adoptLanRobot: () => Promise<void>;
  addLocalSim: () => Promise<void>;
  addAddress: (ip: string, serial: string, visionIp?: string) => Promise<void>;
  rename: (serial: string, name: string) => void;
  setVisionIp: (serial: string, ip: string) => void;
  setRobotVersion: (serial: string, v: RobotVersion) => void;
  setSystem: (serial: string, pc: 'motion' | 'vision', system: string) => void;
  remove: (serial: string) => void;
  clearLocal: () => void;
  migrate: (lanIp: string, wanIp: string, robotIp: string, token: string) => void;
};

function findProfile(serial: string): RobotProfile | null {
  if (serial === SIM_SERIAL) return SIM_PROFILE;
  const local = useRobots.getState().local;
  return (remote.robots?.merge(local) ?? local).find((p) => p.serial === serial) ?? null;
}

let selectSeq = 0;

export const useRobots = create<RobotsState>()(persist((set, get) => ({
  local: [],
  currentSerial: null,
  codeNeeded: false,
  lastRealSerial: null,
  wifiSwitching: null,
  wifiAsk: null,
  wifiAskedNo: false,
  selectIssue: null,
  switching: false,
  passwords: {},
  setPassword: (key, password) => set({ passwords: { ...get().passwords, [key]: password } }),
  clearSelectIssue: () => set({ selectIssue: null }),
  answerWifiAsk: (ok) => {
    const ask = get().wifiAsk;
    if (!ask) return;
    set({ wifiAsk: null, wifiAskedNo: !ok });
    void get().select(ask.serial, ok ? 'auto' : 'skip');
  },
  leaveSim: () => {
    if (get().currentSerial !== SIM_SERIAL) return;
    const back = get().lastRealSerial ?? get().local[0]?.serial ?? null;
    set({ currentSerial: back });
    applyRobotVersion(back ? findProfile(back) : null);
    if (useViewport.getState().key === 'sim') useViewport.getState().setKey('pose3d');
  },
  select: async (serial, wifi = 'auto') => {
    if (get().wifiAsk) set({ wifiAsk: null });
    if (serial === SIM_SERIAL) return get().selectInner(serial);
    const seq = ++selectSeq;
    if (serial !== get().currentSerial) useTelemetry.getState().clearMotionData();
    set({ switching: true });
    try { await get().selectInner(serial, wifi); } finally {
      if (seq === selectSeq) set({ switching: false });
    }
  },
  selectInner: async (serial, wifi = 'auto') => {
    const p = findProfile(serial);
    if (!p) return;
    const was = get().currentSerial;
    if (serial === SIM_SERIAL && was && was !== SIM_SERIAL) set({ lastRealSerial: was });
    set({ currentSerial: serial, codeNeeded: false, selectIssue: null });
    if (serial !== SIM_SERIAL) applyRobotVersion(p);
    const vp = useViewport.getState();
    if (serial === SIM_SERIAL) { vp.setKey('sim'); return; }
    if (vp.key === 'sim') vp.setKey('pose3d');
    const mayMove = wifi === 'auto' || (wifi === 'ask' && !get().wifiAskedNo);
    if (p.lan?.ssid && mayMove && !(await isProfileAtLan(p))) {
      if (wifi === 'ask' && (await onOtherWifi(p.lan.ssid))) {
        if (get().currentSerial !== serial) return;
        set({ wifiAsk: { serial, name: p.name, ssid: p.lan.ssid } });
        return;
      }
      const r = await switchToRobotWifi(p.lan.ssid, () => set({ wifiSwitching: p.lan!.ssid! }), undefined, useWifi.getState().networks);
      set({ wifiSwitching: null });
      if (get().currentSerial !== serial) return;
      const kind = r === 'not_visible' ? 'off' : r === 'not_saved' ? 'wifi_pw' : r === 'failed' ? 'wifi_fail' : null;
      if (kind && !remote.robots?.has(p)) {
        const back = was === SIM_SERIAL ? null : was;
        set({ currentSerial: back, selectIssue: { name: p.name, ssid: p.lan.ssid, kind } });
        applyRobotVersion(back ? findProfile(back) : null);
        return;
      }
    }
    const via = await pickVia(p);
    if (get().currentSerial !== serial) return;
    const prev = get().local.find((x) => x.serial === serial);
    if (p.lan || p.wan) {
      const ssid = via === 'lan' ? await currentSsid() : null;
      const lan = p.lan ? { ...p.lan, ssid: ssidOfKey(serial) ?? ssid ?? p.lan.ssid } : undefined;
      const cached: RobotProfile = { serial, name: prev?.name ?? p.name, lan, wan: prev?.wan ?? p.wan, robotVersion: prev?.robotVersion, system: prev?.system, visionSystem: prev?.visionSystem, lastVia: via, lastSeenAt: Date.now() };
      set({ local: [cached, ...get().local.filter((x) => x.serial !== serial)] });
    }
    if (isRemoteVia(via) && !remote.robots?.canConnect()) { disconnectAll(); set({ codeNeeded: true }); return; }
    if (via === 'lan') useRobot.getState().setVisionIp(p.lan?.visionIp ?? '');
    const t = targetFor({ ...p, lastVia: via });
    if (t) connectNow(t);
  },
  saveConnected: async (name) => {
    const rb = useRobot.getState();
    if (rb.conn !== 'connected') return false;
    let serial: string;
    try { serial = (await rest.serialNumber(rb.ip)).serial_number.trim(); } catch { return false; }
    const noSerial = !serial;
    const ssid = await currentSsid();
    if (noSerial) serial = noSerialKey(rb.ip, ssid);
    const cur = get().currentSerial;
    const prev = get().local.find((p) => p.serial === serial) ?? get().local.find((p) => p.serial === cur && cur?.startsWith('pending:'));
    const next: RobotProfile = {
      ...prev, serial, name: name?.trim() || (prev && !prev.serial.startsWith('pending:') ? prev.name : '') || (noSerial && /^127\.|^localhost$/.test(rb.ip) ? LOCAL_SIM_NAME : prev?.name || ssid || (noSerial ? rb.ip : serial)),
      lan: { ...(prev?.lan ?? { ip: rb.ip }), ssid: ssid ?? prev?.lan?.ssid }, lastVia: prev?.lastVia ?? 'lan', lastSeenAt: Date.now(),
    };
    const mergeByAddr = !isSharedLanIp(rb.ip);
    set({ local: [next, ...get().local.filter((p) => p.serial !== serial && p.serial !== prev?.serial && !(mergeByAddr && p.serial === `pending:${rb.ip}`) && !(mergeByAddr && !noSerial && p.serial === `addr:${rb.ip}`))], currentSerial: serial });
    adoptIpPassword(serial, rb.ip);
    return true;
  },
  addLan: async (serial, ip = ROBOT_LAN_IP) => {
    const prev = get().local.find((p) => p.serial === serial);
    const name = prev?.name ?? (await currentSsid()) ?? serial;
    const next: RobotProfile = { ...(prev ?? { serial, name }), lan: { ...(prev?.lan ?? {}), ip }, lastVia: 'lan', lastSeenAt: Date.now() };
    set({ local: [next, ...get().local.filter((p) => p.serial !== serial && p.serial !== `pending:${ip}`)] });
    adoptIpPassword(serial, ip);
    return get().select(serial);
  },
  addLocalSim: () => get().addAddress('127.0.0.1', ''),
  addAddress: async (ip, serial, visionIp) => {
    const ssid = /^127\.|^localhost$/.test(ip) ? null : await currentSsid();
    const key = serial || noSerialKey(ip, ssid);
    const prev = get().local.find((p) => p.serial === key);
    const name = prev?.name ?? (/^127\.|^localhost$/.test(ip) ? LOCAL_SIM_NAME : ssid ?? (serial || ip));
    const lan = { ...(prev?.lan ?? {}), ip, ...(visionIp !== undefined ? { visionIp: visionIp.trim() || undefined } : {}) };
    const next: RobotProfile = { ...(prev ?? { serial: key, name }), lan, lastVia: 'lan', lastSeenAt: Date.now() };
    const mergeByAddr = !isSharedLanIp(ip);
    set({ local: [next, ...get().local.filter((p) => p.serial !== key && !(mergeByAddr && p.serial === `pending:${ip}`) && !(mergeByAddr && serial && p.serial === `addr:${ip}`))] });
    adoptIpPassword(key, ip);
    return get().select(key);
  },
  adoptLanRobot: async () => {
    if (isDemo() || get().currentSerial === SIM_SERIAL) return;
    const serial = await probeLan(ROBOT_LAN_IP, { base: restBase(ROBOT_LAN_IP) });
    if (serial === null) return;
    const key = serial || noSerialKey(ROBOT_LAN_IP, await currentSsid());
    const rb = useRobot.getState();
    const onIt = get().currentSerial === key && rb.conn === 'connected' && rb.via !== 'remote' && currentTarget()?.route === 'direct';
    if (onIt) return;
    console.log(`[conn] 망 변경 — 로봇망에서 ${key} 발견, 연결`);
    return serial ? get().addLan(serial) : get().addAddress(ROBOT_LAN_IP, '');
  },
  setRobotVersion: (serial, v) => {
    set({ local: get().local.map((p) => (p.serial === serial ? { ...p, robotVersion: v } : p)) });
    if (get().currentSerial === serial) useSettings.getState().setRobotVersion(v);
  },
  setSystem: (serial, pc, system) => {
    const key = pc === 'vision' ? 'visionSystem' : 'system';
    const p = get().local.find((x) => x.serial === serial);
    if (!p || p[key] === system) return;
    set({ local: get().local.map((x) => (x.serial === serial ? { ...x, [key]: system } : x)) });
  },
  setVisionIp: (serial, ip) => set({ local: get().local.map((p) => (p.serial === serial && p.lan ? { ...p, lan: { ...p.lan, visionIp: ip.trim() || undefined } } : p)) }),
  rename: (serial, name) => set({ local: get().local.map((p) => (p.serial === serial ? { ...p, name: name.trim() || p.name } : p)) }),
  remove: (serial) => set({ local: get().local.filter((p) => p.serial !== serial),
    currentSerial: get().currentSerial === serial ? null : get().currentSerial }),
  clearLocal: () => set({ local: [], currentSerial: null }),
  migrate: (lanIp, wanIp, robotIp, token) => {
    const fixed = get().local.map((p) => {
      const key = ssidOfKey(p.serial);
      return key && p.lan && p.lan.ssid !== key ? { ...p, lan: { ...p.lan, ssid: key } } : p;
    });
    if (fixed.some((p, i) => p !== get().local[i])) set({ local: fixed });
    const oldAddr = get().local.find((p) => p.serial === `addr:${ROBOT_LAN_IP}` && p.lan?.ssid);
    if (oldAddr) {
      const nk = `ssid:${oldAddr.lan!.ssid}`;
      set({ local: get().local.map((p) => (p === oldAddr ? { ...p, serial: nk } : p)).filter((p, i, arr) => arr.findIndex((q) => q.serial === p.serial) === i),
        currentSerial: get().currentSerial === oldAddr.serial ? nk : get().currentSerial });
    }
    const realIps = new Set(get().local.filter((p) => !p.serial.includes(':') && p.lan?.ip && !isSharedLanIp(p.lan.ip)).map((p) => p.lan!.ip));
    const kept = get().local.filter((p) => !(p.serial.startsWith('pending:') && p.lan) && !(p.serial.startsWith('addr:') && realIps.has(p.lan?.ip ?? '')));
    if (kept.length !== get().local.length) set({ local: kept, currentSerial: kept.some((p) => p.serial === get().currentSerial) ? get().currentSerial : kept[0]?.serial ?? null });
    if (get().local.length) return;
    const p = migrateLegacy(lanIp, wanIp, robotIp, token);
    if (p) set({ local: [p], currentSerial: p.serial });
  },
}), {
  name: 'rbq-robots',
  storage: createJSONStorage(() => fileBackedStorage),
  partialize: (s) => ({ local: s.local, currentSerial: s.currentSerial, passwords: s.passwords }),
  merge: (persisted, current) => {
    const p = (persisted ?? {}) as Partial<RobotsState>;
    const local = (p.local ?? current.local).map((r) => (r.name === OLD_LOCAL_SIM_NAME ? { ...r, name: LOCAL_SIM_NAME } : r));
    const { '*': _legacy, ...passwords } = p.passwords ?? current.passwords;
    return { ...current, ...p, local, passwords };
  },
}));

function isCurrentAddr(target: string): boolean {
  const { currentSerial, local } = useRobots.getState();
  const cur = currentSerial ? local.find((p) => p.serial === currentSerial) : undefined;
  return target === useRobot.getState().ip || cur?.lan?.ip === target || cur?.wan?.ip === target;
}

setRobotPasswordSource((target) => {
  const { passwords, currentSerial } = useRobots.getState();
  if (!target) return currentSerial ? passwords[currentSerial] : undefined;
  return passwords[ipKey(target)] ?? (currentSerial && isCurrentAddr(target) ? passwords[currentSerial] : undefined);
});

export function passwordKeyFor(target?: string, probe = false): string | undefined {
  const { currentSerial } = useRobots.getState();
  if (!target) return currentSerial ?? undefined;
  if (!probe && currentSerial && isCurrentAddr(target)) return currentSerial;
  return ipKey(target);
}

function adoptIpPassword(serial: string, ip: string) {
  const { passwords } = useRobots.getState();
  const k = ipKey(ip);
  if (passwords[k] === undefined) return;
  const { [k]: pw, ...rest } = passwords;
  useRobots.setState({ passwords: { ...rest, [serial]: pw } });
}

export function useRobotReady(): boolean {
  const conn = useRobot((s) => s.conn);
  const switching = useRobots((s) => s.switching);
  return conn === 'connected' && !switching;
}

export function useProfiles(): RobotProfile[] {
  const local = useRobots((s) => s.local);
  const useMerge = remote.robots?.useMerge;
  return useMerge ? useMerge(local) : byRecent(local);
}

let wasConnected = false;
useRobot.subscribe((st) => {
  const on = st.conn === 'connected';
  if (on === wasConnected) return;
  wasConnected = on;
  if (!on || isDemo()) return;
  const direct = currentTarget()?.route === 'direct';
  (async () => {
    for (let i = 0; direct && i < 3; i++) {
      if (await useRobots.getState().saveConnected().catch(() => false)) break;
      await new Promise((r) => setTimeout(r, 1500));
      if (useRobot.getState().conn !== 'connected') return;
    }
    await learnSystems(direct).catch(() => {});
  })();
});

async function learnSystems(direct: boolean) {
  const serial = useRobots.getState().currentSerial;
  if (!serial || serial === SIM_SERIAL) return;
  const { ip, visionIp } = useRobot.getState();
  const still = () => useRobot.getState().conn === 'connected' && useRobots.getState().currentSerial === serial;
  const handshake = async (addr: string) => {
    const r = await otaProbe(addr);
    return r ? (r.system || LEGACY_SYSTEM) : undefined;
  };
  const motion = (await rest.version(ip).catch(() => null))?.system
    || (direct ? await handshake(ip) : undefined);
  if (motion && still()) {
    useRobots.getState().setSystem(serial, 'motion', motion);
    useSettings.getState().rememberOtaSystem(ip, motion);
  }
  if (!direct) return;
  const vip = visionOtaIp(visionIp);
  const vision = await handshake(vip);
  if (vision && still()) {
    useRobots.getState().setSystem(serial, 'vision', vision);
    useSettings.getState().rememberOtaSystem(vip, vision);
  }
}

const profileSystem = (s: Pick<RobotsState, 'local' | 'currentSerial'>, pc: 'motion' | 'vision') => {
  const p = s.local.find((x) => x.serial === s.currentSerial);
  return pc === 'vision' ? p?.visionSystem : p?.system;
};

export function knownSystem(pc: 'motion' | 'vision', ip?: string): string | undefined {
  return profileSystem(useRobots.getState(), pc) || (ip ? useSettings.getState().otaSystems[ip] : undefined);
}

export function useKnownSystem(pc: 'motion' | 'vision', ip?: string): string | undefined {
  const own = useRobots((s) => profileSystem(s, pc));
  const byIp = useSettings((s) => (ip ? s.otaSystems[ip] : undefined));
  return own || byIp;
}
