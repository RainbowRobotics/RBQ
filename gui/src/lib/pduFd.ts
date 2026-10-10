
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
export type PduFdState = { portOut: FdPortOut[]; portIn: FdPortOut[]; battery: FdBattery[]; temperature: FdTemperature[] };

export type PduFdResp = {
  pdu_fd?: { port_out?: unknown[]; port_in?: unknown[]; battery?: unknown[]; temperature?: unknown[] };
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

export function normalizePduFd(resp: PduFdResp | null | undefined): PduFdState {
  const f = resp?.pdu_fd ?? {};
  const arr = (v: unknown): unknown[] => (Array.isArray(v) ? v : []);
  return {
    portOut: arr(f.port_out).map(port),
    portIn: arr(f.port_in).map(port),
    battery: arr(f.battery).map(battery),
    temperature: arr(f.temperature).map(temperature),
  };
}

export const FD_BATTERY_FAULTS: { key: keyof FdBattery; nm: string }[] = [
  { key: 'ov', nm: 'OV' }, { key: 'uv', nm: 'UV' }, { key: 'ot', nm: 'OT' }, { key: 'ut', nm: 'UT' },
  { key: 'occ', nm: 'OCC' }, { key: 'ocd', nm: 'OCD' }, { key: 'scd', nm: 'SCD' }, { key: 'cid', nm: 'CID' },
];
export function batteryFaults(b: FdBattery): string[] {
  return FD_BATTERY_FAULTS.filter((f) => b[f.key] === true).map((f) => f.nm);
}

export type FdPortGroup = '48v' | '12v' | 'camera' | 'audio';

export type FdPortDef = {
  index: number; name: FdPortName; nm: string; sub: string; group: FdPortGroup;
  danger?: boolean; noL1?: boolean; l1?: boolean; legGuard?: boolean; readOnly?: boolean;
};
export const FD_PORTS: FdPortDef[] = [
  { index: FD_PORT.LEG_48V, name: 'LEG_48V', nm: 'LEGS (48V 구동)', sub: '다리 모터 전원 — 끄면 로봇이 주저앉음. 끄는 것은 비기립 상태에서만', group: '48v', danger: true, legGuard: true },
  { index: FD_PORT.ARM_48V, name: 'ARM_48V', nm: 'ARM · ADDON (48V)', sub: '팔·부가장치 전원 — 팔이 정지 상태인지 확인 후 끄기', group: '48v', danger: true, l1: true },
  { index: FD_PORT.PDU_48V, name: 'PDU_48V', nm: 'PDU (48V 배터리 레일)', sub: '배터리 합산 레일 — 표시만', group: '48v', readOnly: true },
  { index: FD_PORT.WAN_12V, name: 'WAN_12V', nm: '통신 (12V · WAN)', sub: '⚠ 끄면 이 앱과의 연결 자체가 끊깁니다', group: '12v', danger: true },
  { index: FD_PORT.LAN_12V_1, name: 'LAN_12V_1', nm: 'LiDAR (12V · LAN1)', sub: '라이다 센서 전원', group: '12v', noL1: true },
  { index: FD_PORT.LAN_12V_2, name: 'LAN_12V_2', nm: 'CCTV (12V · LAN2)', sub: 'CCTV 카메라 전원', group: '12v' },
  { index: FD_PORT.LAN_12V_3, name: 'LAN_12V_3', nm: '열화상 · PTZ (12V · LAN3)', sub: '열화상 카메라 / PTZ 전원', group: '12v' },
  { index: FD_PORT.FRONT_12V, name: 'FRONT_12V', nm: '전방 12V (IR LED)', sub: '전방 사이드 보드 · 야간 조명 전원', group: '12v' },
  { index: FD_PORT.HIND_12V, name: 'HIND_12V', nm: '후방 12V (IR LED)', sub: '후방 사이드 보드 · 야간 조명 전원', group: '12v' },
  { index: FD_PORT.CAMERA_0, name: 'CAMERA_0', nm: '카메라 0 (하부 · 전방 보드)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_1, name: 'CAMERA_1', nm: '카메라 1 (하부 · 전방 보드)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_2, name: 'CAMERA_2', nm: '카메라 2 (하부 · 후방 보드)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_3, name: 'CAMERA_3', nm: '카메라 3 (하부 · 후방 보드)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_4, name: 'CAMERA_4', nm: '카메라 4 (전방)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.CAMERA_5, name: 'CAMERA_5', nm: '카메라 5 (후방)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.SIDE_CAM_LEFT, name: 'SIDE_CAM_LEFT', nm: '사이드 카메라 좌 (USB)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.SIDE_CAM_RIGHT, name: 'SIDE_CAM_RIGHT', nm: '사이드 카메라 우 (USB)', sub: '끄면 이 카메라 영상 끊김', group: 'camera', danger: true },
  { index: FD_PORT.AMP, name: 'AMP', nm: '스피커 앰프', sub: '워키토키 소리 출력 — 꺼져 있으면 무음', group: 'audio', l1: true },
];

export function fdPortsForLevel(level: 'l1' | 'l2'): FdPortDef[] {
  if (level === 'l2') return FD_PORTS;
  return FD_PORTS.filter((p) => !p.readOnly && ((!p.danger && !p.noL1) || p.l1));
}
