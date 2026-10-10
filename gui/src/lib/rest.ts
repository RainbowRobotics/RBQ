import { robotAuth, reportAuthFailure } from './auth';
import { restBase, streamerBase } from './endpoints';
import { isDemo, demoRest } from '@/lib/demo';
import { dcCommand, streamerCommand } from './commandBus';
import type { LogLine, LogLevel, Trip, VersionInfo } from '@/types/robot';
import type { PayloadCom, PayloadSlot, PayloadRow, PayloadLimits } from '@/lib/payload';
import type { PduFdResp } from '@/lib/pduFd';

export { restBase };

export class HttpError extends Error {
  constructor(public status: number, message: string) { super(message); this.name = 'HttpError'; }
}

export class RemoteUnsupportedError extends Error {
  constructor(path: string) {
    super(`${path} — 원격 연결에서는 사용할 수 없습니다`);
    this.name = 'RemoteUnsupportedError';
  }
}

function viaRendezvous(): boolean {
  try { return require('@/store/robot').useRobot.getState().via === 'rendezvous'; }
  catch { return false; }
}

async function req<T = any>(ip: string, path: string, init?: RequestInit): Promise<T> {
  if (isDemo()) return demoRest(path) as T;
  if (viaRendezvous()) throw new RemoteUnsupportedError(path);
  const auth = robotAuth();
  const res = await fetch(restBase(ip) + path, {
    ...init,
    headers: { ...auth, ...(init?.headers ?? {}) },
  });
  if (res.status === 401) reportAuthFailure(undefined, false, auth);
  if (!res.ok) throw new HttpError(res.status, `${path} → HTTP ${res.status}`);
  return res.json() as Promise<T>;
}

const MAX_TEXT_BYTES = 24 * 1024 * 1024;
async function reqText(ip: string, path: string): Promise<string> {
  if (isDemo()) { const r = demoRest(path); return typeof r === 'string' ? r : JSON.stringify(r); }
  if (viaRendezvous()) throw new RemoteUnsupportedError(path);
  const res = await fetch(restBase(ip) + path, { headers: robotAuth() });
  if (!res.ok) throw new HttpError(res.status, `${path} → HTTP ${res.status}`);
  const len = Number(res.headers.get('content-length') ?? 0);
  if (len > MAX_TEXT_BYTES) {
    throw new Error(`로그가 너무 큽니다(${(len / 1048576).toFixed(0)}MB) — 앱에서 열 수 없어 '저장'으로 내려받아 PC에서 확인하세요`);
  }
  return res.text();
}

const MAX_BINARY_BYTES = 128 * 1048576;
async function reqBinary(ip: string, path: string): Promise<ArrayBuffer> {
  if (isDemo()) return new ArrayBuffer(0);
  if (viaRendezvous()) throw new RemoteUnsupportedError(path);
  const res = await fetch(restBase(ip) + path, { headers: robotAuth() });
  if (!res.ok) throw new Error(`${path} → HTTP ${res.status}`);
  const len = Number(res.headers.get('content-length') ?? 0);
  if (len > MAX_BINARY_BYTES) throw new Error(`파일이 너무 큽니다(${(len / 1048576).toFixed(0)}MB)`);
  return res.arrayBuffer();
}

const LOG_LEVELS = new Set<LogLevel>(['TRACE', 'DEBUG', 'INFO', 'SUCCESS', 'WARNING', 'ERROR', 'FATAL']);

const MAX_PARSE_LINES = 50_000;

export function parseLogLines(text: string): LogLine[] {
  let lines = text.split('\n');
  if (lines.length > MAX_PARSE_LINES) lines = lines.slice(-MAX_PARSE_LINES);
  const out: LogLine[] = [];
  for (const line of lines) {
    const s = line.trim();
    if (!s) continue;
    try {
      const o: any = JSON.parse(s);
      if (!o || typeof o !== 'object' || o.message === undefined) continue;
      const lv = String(o.level || 'INFO').toUpperCase() as LogLevel;
      out.push({
        ts: typeof o.timestamp === 'string' ? o.timestamp.slice(11, 23) : '',
        process: o.application || o.app || '',
        level: LOG_LEVELS.has(lv) ? lv : 'INFO',
        msg: o.message ?? '',
      });
    } catch {}
  }
  return out;
}

export type LogFileEntry = { path: string; mtime: number; size: number };

export type MediaKind = 'photo' | 'clip' | 'archive' | 'audio';
export const MEDIA_ROOT: Record<MediaKind, string> = {
  photo: '/api/snapshot',
  clip: '/api/video',
  archive: '/api/blackbox_video',
  audio: '/api/audio',
};

export type AudioRecordSource = 'uplink' | 'mic';

export type RobotAudioStatus = {
  mic_present?: boolean;
  mic_signal?: boolean | null;
  speaker_present?: boolean;
  ptz?: boolean;
  output?: 'ptz' | 'robot' | 'none';
  input?: 'robot' | 'ptz' | 'none';
};
export type AudioCmdResp = {
  audio?: RobotAudioStatus & {
    recording?: boolean;
    recording_path?: string;
    playing?: boolean;
    playing_path?: string;
  };
  path?: string;
  rms_dbfs?: number;
};

export type OwnershipResp = {
  ownerIP: string;
  requesterIP: string;
  IsOwner: boolean;
  previousIP?: string;
};

export type { IniSection } from './robotRendezvousConfig';
import type { IniSection } from './robotRendezvousConfig';

export const rest = {
  getOwnership: (ip: string) => req<OwnershipResp>(ip, '/api/gamepad/ownership'),
  claimOwnership: (_ip: string) => dcCommand('POST', '/api/gamepad/ownership') as Promise<OwnershipResp>,
  version: (ip: string) =>
    req<VersionInfo>(ip, '/api/version').then((r) => ({ ...r, version: (r.version || '').replace(/^v/, '') })),
  webrtcSetting: (ip: string) =>
    req<{ ini: { sections: IniSection[] }; path: string }>(ip, '/api/webrtc/setting'),
  command: (_ip: string, cmd: string, _msgId?: string) => dcCommand('POST', '/api/motion/command', { cmd }),
  commandStruct: (
    _ip: string,
    target: number,
    userCommand: number,
    para?: { char?: number[]; int?: number[]; float?: number[] },
  ) =>
    dcCommand('POST', '/api/motion/command', {
      target,
      user_command: userCommand,
      ...(para?.char ? { para_char: para.char } : null),
      ...(para?.int ? { para_int: para.int } : null),
      ...(para?.float ? { para_float: para.float } : null),
    }),
  payload: (_ip: string) => dcCommand('GET', '/api/payload/parameters') as Promise<PayloadResp>,
  legHomeSet: (_ip: string) => dcCommand('GET', '/api/motion/leg_home_set') as Promise<LegHomeSetStatus>,

  systemlogList: (ip: string) => req<{ files: LogFileEntry[] }>(ip, '/api/systemlog/list'),
  systemlogByDate: (ip: string, date: string) =>
    reqText(ip, `/api/systemlog/date?date=${encodeURIComponent(date)}`).then(parseLogLines),
  blackboxList: (ip: string) => req<{ files: LogFileEntry[] }>(ip, '/api/blackbox/list'),
  blackboxFile: (ip: string, path: string) =>
    reqText(ip, `/api/blackbox/file?path=${encodeURIComponent(path)}`).then(parseLogLines),
  blackboxRaw: (ip: string, path: string) =>
    reqText(ip, `/api/blackbox/file?path=${encodeURIComponent(path)}`),
  blackboxBinary: (ip: string, path: string) =>
    reqBinary(ip, `/api/blackbox/file?path=${encodeURIComponent(path)}`),
  blackboxFileUrl: (ip: string, path: string): string | null =>
    viaRendezvous() ? null : `${restBase(ip)}/api/blackbox/file?path=${encodeURIComponent(path)}`,

  mediaList: (ip: string, kind: MediaKind) =>
    req<{ files: LogFileEntry[] }>(ip, `${MEDIA_ROOT[kind]}/list`),
  mediaDelete: (ip: string, kind: MediaKind, paths: string[]) =>
    req<{ deleted: string[]; skipped: { path: string; reason: string }[] }>(ip, `${MEDIA_ROOT[kind]}/delete`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ paths }),
    }),
  mediaFileUrl: (ip: string, kind: MediaKind, path: string): string | null =>
    viaRendezvous()
      ? null
      : `${restBase(ip)}${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`,
  mediaBinary: (ip: string, kind: MediaKind, path: string) =>
    reqBinary(ip, `${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`),
  snapshotSave: (ip: string, label?: string) =>
    streamerReq(ip, '/api/snapshot/save', 'POST', label ? { label } : {}),
  videoRecord: (ip: string, state: 'start' | 'stop') =>
    streamerReq<{ since?: number }>(ip, '/api/video/record', 'PUT', { state }),
  audioRecord: (ip: string, state: 'start' | 'stop', source: AudioRecordSource = 'uplink') =>
    streamerReq<AudioCmdResp>(ip, '/api/audio/record', 'PUT', { state, source }),
  audioPlay: (ip: string, path: string, loop = false) =>
    streamerReq<AudioCmdResp>(ip, '/api/audio/play', 'PUT', { path, loop }),
  audioPlayStop: (ip: string) =>
    streamerReq<AudioCmdResp>(ip, '/api/audio/play', 'PUT', { state: 'stop' }),
  audioStatus: (ip: string) =>
    streamerReq<AudioCmdResp>(ip, '/api/audio/status', 'GET'),
  audioUpload: (ip: string, part: { name: string; offset: number; total: number; data: string }) =>
    streamerReq<{ status: string; received: number; path?: string }>(ip, '/api/audio/upload', 'POST', part),
  getDockParams: (_ip: string) =>
    (dcCommand('GET', '/api/dock/parameters') as Promise<{ dock: DockParams }>).then((r) => r.dock),
  serialNumber: (ip: string) => req<{ serial_number: string }>(ip, '/api/robot/serial_number'),
  robotFeatures: (_ip: string) => dcCommand('GET', '/api/robot/features') as Promise<RobotFeaturesResp>,
  boardFw: (_ip: string) => dcCommand('GET', '/api/firmware/update') as Promise<unknown>,
  trip: (ip: string) => req<Trip>(ip, '/api/trip'),
};

export type DockParams = { offset_x: number; offset_y: number; count_req: number; count_try: number };

export type { PayloadCom, PayloadSlot, PayloadRow } from '@/lib/payload';
export { PAYLOAD_LIMITS } from '@/lib/payload';
export type { PayloadLimits } from '@/lib/payload';

export type PayloadResp = {
  payload: { mass_kg: number; center_of_mass: PayloadCom };
  slots?: PayloadSlot[];
  total?: { mass_kg: number; center_of_mass: PayloadCom };
  limits?: PayloadLimits;
};

export type RobotFeatures = {
  [key: string]: boolean | undefined;
  fire_fight?: boolean; slam?: boolean;
  wheel?: boolean; qc?: boolean; debug?: boolean;
  can_fd?: boolean;
  fw_update?: boolean;
};
export type RobotFeaturesResp = { features: RobotFeatures };

export type LegHomeSetLeg = {
  leg: number;
  joints: [number, number, number];
  expected_deg: [number, number, number];
  ok?: boolean;
  reason?: string;
  measured_deg?: [number, number, number];
  joint_done?: [boolean, boolean, boolean];
  joint_ok?: [boolean, boolean, boolean];
  roll_ref?: LegHomeRef;
  finished_at?: string;
};
export type LegHomeGroup = 'roll' | 'pitch_knee';
export type LegHomeRef = 'limit' | 'level';
export type LegHomeSetStatus = {
  running: boolean;
  leg: number;
  roll_group?: boolean;
  tolerance_deg: number;
  legs: LegHomeSetLeg[];
  boards_alive?: boolean;
  timestamp: string;
};

function put(_ip: string, path: string, body: object) {
  return dcCommand('PUT', path, body);
}
function post(_ip: string, path: string, body: object) {
  return dcCommand('POST', path, body);
}

export function walkPercentToSi(walk: { body_height: number; max_speed: number }) {
  const bp = Math.min(100, Math.max(0, walk.body_height));
  const sp = Math.min(100, Math.max(0, walk.max_speed));
  return {
    body_height: -0.25 + (0.35 * bp) / 100,
    max_speed: 0.5 + (2.0 * sp) / 100,
  };
}

export const actions = {
  walkParams: (ip: string, p: { body_height: number; max_speed: number; foot_height: number }) =>
    put(ip, '/api/motion/walk_parameters', p),
  setBodyTilt: (ip: string, deg: number, walk: { body_height: number; max_speed: number }) =>
    put(ip, '/api/motion/walk_parameters', { unit: 'si', ...walkPercentToSi(walk), body_tilt: deg }),
  tripReset: (ip: string, id: 'A' | 'B') => post(ip, '/api/trip/reset', { id }),
  setWebrtcSetting: (ip: string, sections: IniSection[]) =>
    put(ip, '/api/webrtc/setting', { sections }),
  setPayload: (ip: string, mass_kg: number, com: PayloadCom) =>
    put(ip, '/api/payload/parameters', { mass_kg, center_of_mass: com }),
  setPayloadSlots: (ip: string, rows: PayloadRow[]) =>
    put(ip, '/api/payload/parameters', {
      slots: rows.map((r) => ({
        id: r.id,
        mass_kg: r.enabled ? r.mass : 0,
        center_of_mass: r.enabled
          ? { x_m: r.x, y_m: r.y, z_m: r.z }
          : { x_m: 0, y_m: 0, z_m: 0 },
      })),
    }),
  legHomeSetStart: (ip: string, leg: number, group: LegHomeGroup, ref: LegHomeRef = 'limit') =>
    post(ip, '/api/motion/leg_home_set', { leg, group: group === 'roll' && ref === 'level' ? 'roll_level' : group }),
  reboot: (ip: string) => put(ip, '/api/system/reboot', {}),
  boardFwCheck: (ip: string) => post(ip, '/api/firmware/check', {}) as Promise<{ check_seq?: number }>,
  boardFwUpdate: (ip: string, board: number | 'all' | 'motors', allowDowngrade: boolean, powerDown: boolean, checkSeq: number) =>
    post(ip, '/api/firmware/update', { board, allow_downgrade: allowDowngrade, power_down: powerDown, check_seq: checkSeq }),
  pduPower: (ip: string, port: number, state: boolean) => put(ip, '/api/pdu/power', { port, state }),
  pduFd: (_ip: string) => dcCommand('GET', '/api/pdu/fd') as Promise<PduFdResp>,
  pduFdPort: (ip: string, port: number, state: boolean) => put(ip, '/api/pdu/fd/port', { port, state }),
  ledBottom: (_ip: string) => dcCommand('GET', '/api/led/bottom') as Promise<unknown>,
  ledBottomSet: (ip: string, body: object) => put(ip, '/api/led/bottom', body),
  blackboxSaveNow: (ip: string) => post(ip, '/api/blackbox/save', {}),
  visionProgram: (ip: string, id: number, running: boolean) =>
    streamerReq(ip, '/api/vision/program', 'PUT', { id, running }),
  visionDayNight: (ip: string, day: boolean) =>
    streamerReq(ip, '/api/vision/day_night', 'PUT', { day }),
  irProjector: (ip: string, enabled: boolean) =>
    streamerReq(ip, '/api/hal/ir_projector', 'PUT', { enabled }),
  visionReset: (ip: string) => streamerReq(ip, '/api/vision/state', 'DELETE'),
  setDockParams: (ip: string, p: DockParams) => put(ip, '/api/dock/parameters', p),
  gamepadExternal: (ip: string, enabled: boolean) => put(ip, '/api/gamepad/external', { enabled }),
  visionPoint2Go: (ip: string, enabled: boolean) =>
    streamerReq(ip, '/api/vision/point2go', 'PUT', { enabled }),
  visionTouchClick: (ip: string, x: number, y: number) =>
    streamerReq(ip, '/api/vision/touch', 'POST', { type: 'click', x, y }),
  visionGuide: (ip: string, enabled: boolean, id = 0) =>
    streamerReq(ip, '/api/vision/guide', 'PUT', { enabled, id }),
  visionFaceDetect: (ip: string, enabled: boolean) =>
    streamerReq(ip, '/api/vision/face_detect', 'PUT', { enabled }),
  visionDockScan: (ip: string, enabled: boolean) =>
    streamerReq(ip, '/api/vision/dock/scan', 'PUT', { enabled }),
  visionHeightmapSettings: (ip: string, stairs: boolean, edge: boolean) =>
    streamerReq(ip, '/api/vision/heightmap/settings', 'PUT', { stairs, edge }),
  visionWebrtcResolution: (ip: string, width: number, height: number) =>
    streamerReq(ip, '/api/vision/webrtc/resolution', 'PUT', { width, height }),
  visionWebrtcPreset: (ip: string, preset: string) =>
    streamerReq(ip, '/api/vision/webrtc/preset', 'PUT', { preset }),
  visionWebrtcResolutionGet: (_ip: string) =>
    streamerCommand('GET', '/api/vision/webrtc/resolution') as Promise<{ width: number; height: number; active_width?: number; active_height?: number }>,
};

export const WEBRTC_RES = [
  { label: 'HD 720p', width: 1280, height: 720 },
  { label: 'SQHD 540', width: 960, height: 540 },
  { label: '절약 360', width: 640, height: 360 },
] as const;

async function streamerReq<T = void>(
  _ip: string, path: string, method: 'GET' | 'PUT' | 'POST' | 'DELETE', body?: object,
): Promise<T> {
  if (isDemo()) { demoRest(path); return {} as T; }
  return (await streamerCommand(method, path, body ?? {})) as T;
}

export const VISION_PROGRAM = { Handeye: 3, Heightmap: 4, SLAMNAV_3D: 5, CCTV: 6, Ptz: 7, ThermalCam: 8 } as const;

export function restErrorText(e: unknown): string {
  if (e instanceof RemoteUnsupportedError) {
    return '원격 연결에서는 사용할 수 없습니다 — 로봇과 같은 네트워크에서 연결하거나, [서버로 보내기]로 올린 뒤 대시보드에서 확인하세요';
  }
  return e instanceof Error ? e.message : String(e);
}
