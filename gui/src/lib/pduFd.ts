
export const FD_PORT = {
  LAN_12V_1: 0, LAN_12V_2: 1, LAN_12V_3: 2, WAN_12V: 3, TBD_12V: 4, FRONT_12V: 5, HIND_12V: 6,
  CAMERA_0: 7, CAMERA_1: 8, CAMERA_2: 9, CAMERA_3: 10, CAMERA_4: 11, CAMERA_5: 12,
  LEG_48V: 13, ARM_48V: 14, PDU_48V: 15, SIDE_CAM_LEFT: 16, SIDE_CAM_RIGHT: 17, AMP: 18,
  NO_PORT_OUT: 19,
} as const;
export type FdPortName = Exclude<keyof typeof FD_PORT, 'NO_PORT_OUT'>;

export type FdPortState = 1 | 0 | -1;

export type FdPortOut = { index: number; name: string; state: FdPortState; voltage: number; current: number };
export type FdBattery = {
  index: number; name: string; voltage: number; current: number; soc: number;
  detect: boolean; ov: boolean; uv: boolean; ot: boolean; ut: boolean; occ: boolean; ocd: boolean; scd: boolean; cid: boolean;
};
export type FdTemperature = { index: number; name: string; value: number };
export type FdSideHub = { side: string; seq: number; running: boolean; result: number; cfg: number; rst: number; vid: number; pid: number };
export type PduFdState = { portOut: FdPortOut[]; portIn: FdPortOut[]; battery: FdBattery[]; temperature: FdTemperature[]; sideHub: FdSideHub[] };

export type PduFdResp = {
  pdu_fd?: { port_out?: unknown[]; port_in?: unknown[]; battery?: unknown[]; temperature?: unknown[]; side_hub?: unknown[] };
  timestamp?: string; status?: string;
};

const num = (v: unknown): number => (typeof v === 'number' && Number.isFinite(v) ? v : 0);
const str = (v: unknown): string => (typeof v === 'string' ? v : '');
const state = (v: unknown): FdPortState => (v === 1 ? 1 : v === -1 ? -1 : 0);

function port(raw: unknown, i: number): FdPortOut {
  const o = (raw ?? {}) as Record<string, unknown>;
  return { index: typeof o.index === 'number' ? o.index : i, name: str(o.name), state: state(o.state), voltage: num(o.voltage), current: num(o.current) };
}
function battery(raw: unknown, i: number): FdBattery {
  const o = (raw ?? {}) as Record<string, unknown>;
  return {
    index: typeof o.index === 'number' ? o.index : i, name: str(o.name),
    voltage: num(o.voltage), current: num(o.current), soc: num(o.soc),
    detect: !!o.detect, ov: !!o.ov, uv: !!o.uv, ot: !!o.ot, ut: !!o.ut, occ: !!o.occ, ocd: !!o.ocd, scd: !!o.scd, cid: !!o.cid,
  };
}
function temperature(raw: unknown, i: number): FdTemperature {
  const o = (raw ?? {}) as Record<string, unknown>;
  return { index: typeof o.index === 'number' ? o.index : i, name: str(o.name), value: num(o.value) };
}

function sideHub(raw: unknown, i: number): FdSideHub {
  const o = (raw ?? {}) as Record<string, unknown>;
  return {
    side: str(o.side) || (i === 0 ? 'front' : 'hind'), seq: num(o.seq), running: !!o.running,
    result: num(o.result), cfg: num(o.cfg), rst: num(o.rst), vid: num(o.vid), pid: num(o.pid),
  };
}

export function normalizePduFd(resp: PduFdResp | null | undefined): PduFdState {
  const f = resp?.pdu_fd ?? {};
  const arr = (v: unknown): unknown[] => (Array.isArray(v) ? v : []);
  return {
    portOut: arr(f.port_out).map(port),
    portIn: arr(f.port_in).map(port),
    battery: arr(f.battery).map(battery),
    temperature: arr(f.temperature).map(temperature),
    sideHub: arr(f.side_hub).map(sideHub),
  };
}

export const SIDE_HUBS: { side: 'front' | 'hind'; nm: string; cams: string }[] = [
  { side: 'front', nm: '전방 허브 리셋', cams: '카메라 0 · 1 · 4' },
  { side: 'hind', nm: '후방 허브 리셋', cams: '카메라 2 · 3 · 5' },
];

export const SIDE_HUB_RESULT: Record<number, string> = {
  0: '완료 — 허브가 다시 붙었습니다',
  1: '보드가 명령을 거절했습니다(키)',
  2: '허브가 SMBus 에 답하지 않습니다 — SMBus 모드가 아니거나(R10) 허브 전원 · 배선',
  3: '허브 ID 가 TI(0451)가 아닙니다',
  4: '허브 설정 끝(cfgActive)이 지워지지 않습니다',
  5: '앞선 리셋이 아직 진행 중입니다',
  6: '리셋이 먹지 않았습니다 — smbusRst 뒤에도 cfgActive 가 서지 않음',
  7: '포트 전원 설정을 덮어쓰지 못했습니다 — 허브는 다시 붙었지만 카메라 5V 가 안 나갈 수 있음',
  8: '보드 회신 없음 — 사이드 보드 펌웨어가 이 명령을 모릅니다(261021 이전)',
  9: '이 사이드 보드는 허브 SMBus 를 지원하지 않습니다 — hw 1.1.0.1 보드만 됩니다',
};
const hex = (v: number, n: number) => v.toString(16).toUpperCase().padStart(n, '0');
export function sideHubRaw(h: FdSideHub): string {
  return `rst ${hex(h.rst, 2)} · cfg ${hex(h.cfg, 2)} · ${hex(h.vid, 4)}:${hex(h.pid, 4)}`;
}

export const FD_BATTERY_FAULTS: { key: keyof FdBattery; nm: string }[] = [
  { key: 'ov', nm: 'OV' }, { key: 'uv', nm: 'UV' }, { key: 'ot', nm: 'OT' }, { key: 'ut', nm: 'UT' },
  { key: 'occ', nm: 'OCC' }, { key: 'ocd', nm: 'OCD' }, { key: 'scd', nm: 'SCD' }, { key: 'cid', nm: 'CID' },
];
export function batteryFaults(b: FdBattery): string[] {
  return FD_BATTERY_FAULTS.filter((f) => b[f.key] === true).map((f) => f.nm);
}

export type FdPortGroup = 'top' | 'side' | 'camera' | 'internal';

export type FdPortDef = {
  index: number; name: FdPortName; nm: string; sub: string; group: FdPortGroup;
  danger?: boolean; legGuard?: boolean; readOnly?: boolean; askOff?: boolean;
};
export const FD_PORTS: FdPortDef[] = [
  { index: FD_PORT.LAN_12V_1, name: 'LAN_12V_1', nm: '12V 포트 1', sub: '상단 12V/LAN 포트 전원', group: 'top', askOff: true },
  { index: FD_PORT.LAN_12V_2, name: 'LAN_12V_2', nm: '12V 포트 2', sub: '상단 12V/LAN 포트 전원', group: 'top', askOff: true },
  { index: FD_PORT.LAN_12V_3, name: 'LAN_12V_3', nm: '12V 포트 3', sub: '상단 12V/LAN 포트 전원', group: 'top', askOff: true },
  { index: FD_PORT.WAN_12V, name: 'WAN_12V', nm: '12V 포트 4', sub: '상단 12V/LAN 포트 전원', group: 'top', askOff: true },
  { index: FD_PORT.ARM_48V, name: 'ARM_48V', nm: 'ARM · ADDON (48V)', sub: '팔·부가장치 전원 — 팔이 정지 상태인지 확인 후 끄기', group: 'top', danger: true },
  { index: FD_PORT.FRONT_12V, name: 'FRONT_12V', nm: '전방 12V', sub: '전방 사이드 보드 · IR LED 전원', group: 'side' },
  { index: FD_PORT.HIND_12V, name: 'HIND_12V', nm: '후방 12V', sub: '후방 사이드 보드 · IR LED 전원', group: 'side' },
  { index: FD_PORT.CAMERA_0, name: 'CAMERA_0', nm: '카메라 0 (하부 · 전방 보드)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_1, name: 'CAMERA_1', nm: '카메라 1 (하부 · 전방 보드)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_2, name: 'CAMERA_2', nm: '카메라 2 (하부 · 후방 보드)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_3, name: 'CAMERA_3', nm: '카메라 3 (하부 · 후방 보드)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_4, name: 'CAMERA_4', nm: '카메라 4 (전방)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_5, name: 'CAMERA_5', nm: '카메라 5 (후방)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.SIDE_CAM_LEFT, name: 'SIDE_CAM_LEFT', nm: '사이드 카메라 좌 (USB)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.SIDE_CAM_RIGHT, name: 'SIDE_CAM_RIGHT', nm: '사이드 카메라 우 (USB)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.AMP, name: 'AMP', nm: '스피커 앰프', sub: '워키토키 소리 출력 — 꺼져 있으면 무음', group: 'internal' },
  { index: FD_PORT.LEG_48V, name: 'LEG_48V', nm: 'LEGS (48V 구동)', sub: '다리 모터 전원 — 끄면 로봇이 주저앉음. 끄는 것은 비기립 상태에서만', group: 'internal', danger: true, legGuard: true },
  { index: FD_PORT.PDU_48V, name: 'PDU_48V', nm: 'PDU (48V 배터리 레일)', sub: '배터리 합산 레일 — 표시만', group: 'internal', readOnly: true },
];
