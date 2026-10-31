import { robotAuth, reportAuthFailure } from './auth';
import { restBase, streamerBase } from './endpoints';
import { isDemo, demoDriver } from '@/lib/demoFlag';
import { dcCommand, streamerCommand, type CommandOptions } from './commandBus';
import { RemoteUnsupportedError, tunnelFetch, tunnelReady, viaRemote } from './robotTunnel';

export { RemoteUnsupportedError, viaRemote };
import type { LogLine, LogLevel, SystemInfo, SystemStats, Trip, VersionInfo } from '@/types/robot';
import type { PayloadCom, PayloadSlot, PayloadRow, PayloadLimits } from '@/lib/payload';
import type { PduFdResp } from '@/lib/pduFd';

export { restBase };

export class HttpError extends Error {
  constructor(public status: number, message: string, public detail = '') { super(message); this.name = 'HttpError'; }
}

export function robotFetch(url: string, init?: RequestInit): Promise<Response> {
  if (!viaRemote()) return fetch(url, init);
  if (!tunnelReady()) return Promise.reject(new RemoteUnsupportedError(url.replace(/^https?:\/\/[^/]+/, '')));
  return tunnelFetch(url, init);
}

async function req<T = any>(ip: string, path: string, init?: RequestInit): Promise<T> {
  const demo = demoDriver();
  if (demo) return demo.rest(path) as T;
  const auth = robotAuth();
  const res = await robotFetch(restBase(ip) + path, {
    ...init,
    headers: { ...auth, ...(init?.headers ?? {}) },
  });
  if (res.status === 401) reportAuthFailure(undefined, false, auth);
  if (!res.ok) throw new HttpError(res.status, `${path} → HTTP ${res.status}`, (await res.text().catch(() => '')).slice(0, 300));
  return res.json() as Promise<T>;
}

const MAX_TEXT_BYTES = 24 * 1024 * 1024;
async function reqText(ip: string, path: string): Promise<string> {
  const demo = demoDriver();
  if (demo) { const r = demo.rest(path); return typeof r === 'string' ? r : JSON.stringify(r); }
  const res = await robotFetch(restBase(ip) + path, { headers: robotAuth() });
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
  const res = await robotFetch(restBase(ip) + path, { headers: robotAuth() });
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

export type ArchiveSegment = { cam: string; path: string; start: number; end: number };
export type ArchiveIndex = {
  enabled?: boolean;
  days?: string[];
  date?: string;
  dayStartMs?: number;
  cams?: string[];
  segments?: ArchiveSegment[];
};
export type ArchiveState = {
  mode?: 'archive' | 'live';
  cam?: string;
  posMs?: number;
  speed?: number;
  paused?: boolean;
  ended?: boolean;
  waiting?: boolean;
  nextMs?: number;
  segment?: string;
  error?: string;
};
export type ArchiveCmd =
  | { action: 'play' | 'seek'; cam: string; atMs: number }
  | { action: 'speed'; speed: number }
  | { action: 'pause' | 'resume' | 'live' | 'state' };

export type RobotAudioStatus = {
  mic_present?: boolean;
  mic_signal?: boolean | null;
  speaker_present?: boolean;
  ptz?: boolean;
  output?: 'ptz' | 'robot' | 'none';
  input?: 'robot' | 'ptz' | 'none';
  recording?: boolean;
  talk_gate?: boolean;
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

export type IniKey = { key: string; value: string };
export type IniSection = { name: string; keys: IniKey[] };

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
    options?: CommandOptions,
  ) =>
    dcCommand('POST', '/api/motion/command', {
      target,
      user_command: userCommand,
      ...(para?.char ? { para_char: para.char } : null),
      ...(para?.int ? { para_int: para.int } : null),
      ...(para?.float ? { para_float: para.float } : null),
    }, options),
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
    viaRemote() ? null : `${restBase(ip)}/api/blackbox/file?path=${encodeURIComponent(path)}`,

  mediaList: (ip: string, kind: MediaKind) =>
    req<{ files: LogFileEntry[] }>(ip, `${MEDIA_ROOT[kind]}/list`),
  mediaDelete: (ip: string, kind: MediaKind, paths: string[]) =>
    req<{ deleted: string[]; skipped: { path: string; reason: string }[] }>(ip, `${MEDIA_ROOT[kind]}/delete`, {
      method: 'POST', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify({ paths }),
    }),
  mediaFileUrl: (ip: string, kind: MediaKind, path: string): string | null =>
    viaRemote()
      ? null
      : `${restBase(ip)}${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`,
  mediaBinary: (ip: string, kind: MediaKind, path: string) =>
    reqBinary(ip, `${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`),
  snapshotSave: (ip: string, label?: string) =>
    streamerReq(ip, '/api/snapshot/save', 'POST', label ? { label } : {}),
  videoRecord: (ip: string, state: 'start' | 'stop') =>
    streamerReq<{ since?: number }>(ip, '/api/video/record', 'PUT', { state }),
  archiveIndex: (ip: string, date?: string) =>
    streamerReq<ArchiveIndex>(ip, '/api/blackbox_video/index', 'GET', date ? { date } : {}),
  archivePlayback: (ip: string, cmd: ArchiveCmd) =>
    streamerReq<ArchiveState>(ip, '/api/blackbox_video/playback', 'PUT', cmd),
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
  remoteSsh: <T = unknown>(ip: string) => req<T>(ip, '/api/remote/ssh'),
  remoteSshSet: <T = unknown>(ip: string, body: object) =>
    req<T>(ip, '/api/remote/ssh', { method: 'PUT', headers: { 'Content-Type': 'application/json' }, body: JSON.stringify(body) }),
  robotFeatures: (_ip: string) => dcCommand('GET', '/api/robot/features') as Promise<RobotFeaturesResp>,
  boardFw: (_ip: string) => dcCommand('GET', '/api/firmware/update') as Promise<unknown>,
  trip: (ip: string) => req<Trip>(ip, '/api/trip'),
  systemBoots: (_ip: string) =>
    dcCommand('GET', '/api/system/boots') as Promise<{ boots: { boot_id: string; boot_unix: number }[] }>,
  systemInfo: (_ip: string) => dcCommand('GET', '/api/system/info') as Promise<SystemInfo>,
  systemStats: (_ip: string, since?: number) =>
    dcCommand('GET', '/api/system/stats', since ? { since } : undefined) as Promise<SystemStats>,
};

export type DockParams = { offset_x: number; offset_y: number; count_req: number; count_try: number };

export type CameraStreamKey = 'rgb' | 'ir' | 'depth';
export type CameraCheckStream = { current: number; median: number; min: number; drops: number; target: number; samples: number };
export type CameraCheckVerdict = 'pass' | 'fail' | 'skip';
export type CameraCheckReason =
  'ok' | 'disabled' | 'sleeping' | 'not_opened' | 'not_detected' | 'device_error' | 'stream_missing' | 'frame_drop';
export type CameraStatus = {
  enabled: boolean; powered: boolean; detected: boolean; connected: boolean; idle: boolean; sleep: boolean;
  running: boolean; failed: boolean; error_id: number; projector: boolean; projector_on: boolean;
};
export type CameraCheckSensor = {
  id: number; name: string; type: 'realsense' | 'oak' | 'unknown'; serial: string;
  usb: number | null;
  status: CameraStatus;
  streams: Partial<Record<CameraStreamKey, CameraCheckStream>>;
  verdict: CameraCheckVerdict | null; reason: CameraCheckReason | null; reason_streams: string[];
};
export type CameraLiveView = CameraStreamKey | 'auto';
export type CameraLiveSensor = {
  id: number; name: string; status: CameraStatus; usb: number | null;
  fps: Partial<Record<CameraStreamKey, number>>; view: CameraLiveView;
};
export type CameraLive = { hal: boolean; sensors: CameraLiveSensor[] };
export type CameraCheck = {
  state: 'idle' | 'running' | 'done' | 'aborted' | 'failed';
  duration_s: number; elapsed_s: number; started_at: string | null;
  error: string | null;
  sensors: CameraCheckSensor[];
};

export type DepthMetrics = { frames: number; fill_rate: number; subpixel_rms: number; plane_rms_mm: number; distance_mm: number };
export type DepthCheckResult = 'running' | 'ok' | 'failed' | 'not_streaming' | 'busy' | 'nothing' | 'timeout';
export type DepthCheckSensor = {
  id: number; result: DepthCheckResult;
  measured: DepthMetrics | null; after: DepthMetrics | null;
  health: number; applied: boolean; low_fill: boolean; needs_occ: boolean;
  occ: '' | 'running' | 'applied' | 'saved' | 'kept' | Exclude<DepthCheckResult, 'running' | 'ok'>;
};
export type DepthCheck = {
  state: 'idle' | 'running'; action: string;
  current: { sensor: number; step: 'measure' | 'on_chip' | 'apply' | 'revert' | 'save'; progress: number } | null;
  sensors: DepthCheckSensor[];
};

export type UserExtrinsicSensor = {
  id: number; result: 'running' | 'ok' | 'no_plane' | 'not_streaming' | 'no_board' | 'timeout';
  tilt_roll?: number; tilt_pitch?: number; height_mm?: number; ref_rms_mm?: number; needs_align?: boolean;
  cross_x_mm?: number; cross_y_mm?: number; cross_yaw_deg?: number;
  boards_used?: number[]; boards_seen?: number[];
};
export type UserExtrinsicBoard = { id: number; result: 'running' | 'ok' | 'skipped' | 'no_board' | 'timeout'; delta?: number[] };
export type UserExtrinsicLimits = { roll_deg: number; pitch_deg: number; z_mm: number; x_mm: number; y_mm: number; yaw_deg: number };
export type UserExtrinsicAction = 'measure' | 'board' | 'revert' | 'approach' | 'stop';
export type UserExtrinsicApproach = { result: 'running' | 'ok' | 'no_board' | 'not_moved' | 'stopped' | 'reversed' | 'timeout'; walks: number; target: number[] };
export type UserExtrinsic = {
  state: 'idle' | 'running'; action: 'measure' | 'board' | 'approach'; phase?: 'collecting' | 'remeasuring' | 'reverting' | 'searching' | 'walking';
  ref_ok: boolean; on_board: boolean;
  sensors: UserExtrinsicSensor[]; board: UserExtrinsicBoard[]; before?: UserExtrinsicSensor[];
  outcome?: 'kept' | 'reverted' | 'skipped' | 'cancelled'; limits?: UserExtrinsicLimits; approach?: UserExtrinsicApproach;
};

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
  [key: string]: boolean | string | undefined;
  fire_fight?: boolean; slam?: boolean;
  wheel?: boolean; qc?: boolean; debug?: boolean;
  can_fd?: boolean;
  fw_update?: boolean;
  simulation?: boolean;
  arm?: string;
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
  restart: (ip: string) => put(ip, '/api/system/restart', {}),
  boardFwCheck: (ip: string) => post(ip, '/api/firmware/check', {}) as Promise<{ check_seq?: number }>,
  boardFwUpdate: (ip: string, board: number | 'all' | 'motors', allowDowngrade: boolean, powerDown: boolean, checkSeq: number) =>
    post(ip, '/api/firmware/update', { board, allow_downgrade: allowDowngrade, power_down: powerDown, check_seq: checkSeq }),
  pduPower: (ip: string, port: number, state: boolean) => put(ip, '/api/pdu/power', { port, state }),
  pduFd: (_ip: string) => dcCommand('GET', '/api/pdu/fd') as Promise<PduFdResp>,
  pduFdPort: (ip: string, port: number, state: boolean) => put(ip, '/api/pdu/fd/port', { port, state }),
  pduFdHubReset: (ip: string, side: 'front' | 'hind') => put(ip, '/api/pdu/fd/hub_reset', { side }),
  ledBottom: (_ip: string) => dcCommand('GET', '/api/led/bottom') as Promise<unknown>,
  ledBottomSet: (ip: string, body: object) => put(ip, '/api/led/bottom', body),
  ledTopLampTest: (ip: string) => post(ip, '/api/led/top/lamp_test', {}),
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
  visionFaceDetect: (ip: string, enabled: boolean) =>
    streamerReq(ip, '/api/vision/face_detect', 'PUT', { enabled }),
  visionDockScan: (ip: string, enabled: boolean) =>
    streamerReq(ip, '/api/vision/dock/scan', 'PUT', { enabled }),
  visionDockScanGet: (_ip: string) =>
    streamerCommand('GET', '/api/vision/dock/scan') as Promise<{ available?: boolean; active?: boolean }>,
  visionObsMapping: (ip: string, enabled: boolean) =>
    streamerReq(ip, '/api/vision/obstacle/mapping', 'PUT', { enabled }),
  visionObsMargin: (ip: string, margin: number) =>
    streamerReq(ip, '/api/vision/obstacle/margin', 'PUT', { margin }),
  visionObsStopMode: (ip: string, enabled: boolean) =>
    streamerReq(ip, '/api/vision/obstacle/stop_mode', 'PUT', { enabled }),
  visionObsRange: (ip: string, range: number) =>
    streamerReq(ip, '/api/vision/obstacle/range', 'PUT', { range }),
  visionHeightmapSettings: (ip: string, stairs: boolean, edge: boolean) =>
    streamerReq(ip, '/api/vision/heightmap/settings', 'PUT', { stairs, edge }),
  visionWebrtcResolution: (ip: string, width: number, height: number) =>
    streamerReq(ip, '/api/vision/webrtc/resolution', 'PUT', { width, height }),
  visionWebrtcPreset: (ip: string, preset: string) =>
    streamerReq(ip, '/api/vision/webrtc/preset', 'PUT', { preset }),
  visionWebrtcResolutionGet: (_ip: string) =>
    streamerCommand('GET', '/api/vision/webrtc/resolution') as Promise<{ width: number; height: number; active_width?: number; active_height?: number }>,
  cameraCheckStart: (_ip: string, durationS?: number) =>
    (streamerCommand('POST', '/api/hal/camera_check/stream', durationS ? { duration_s: durationS } : {}) as Promise<{ camera_check: CameraCheck }>)
      .then((r) => r.camera_check),
  cameraCheckGet: (_ip: string) =>
    (streamerCommand('GET', '/api/hal/camera_check/stream') as Promise<{ camera_check: CameraCheck }>).then((r) => r.camera_check),
  cameraCheckStop: (_ip: string) =>
    (streamerCommand('DELETE', '/api/hal/camera_check/stream') as Promise<{ camera_check: CameraCheck }>).then((r) => r.camera_check),
  cameraLivePut: (_ip: string, streams: CameraLiveView[]) =>
    (streamerCommand('PUT', '/api/hal/camera_check/live', { streams }) as Promise<{ camera_live: CameraLive }>).then((r) => r.camera_live),
  cameraLiveGet: (_ip: string) =>
    (streamerCommand('GET', '/api/hal/camera_check/live') as Promise<{ camera_live: CameraLive }>).then((r) => r.camera_live),
  cameraLiveStop: (_ip: string) => streamerCommand('DELETE', '/api/hal/camera_check/live'),
  cameraDepthPost: (_ip: string, action: 'measure' | 'on_chip' | 'revert', sensors?: number[]) =>
    (streamerCommand('POST', '/api/hal/camera_check/depth', sensors ? { action, sensors } : { action }) as Promise<{ depth_check: DepthCheck }>)
      .then((r) => r.depth_check),
  cameraDepthGet: (_ip: string) =>
    (streamerCommand('GET', '/api/hal/camera_check/depth') as Promise<{ depth_check: DepthCheck }>).then((r) => r.depth_check),
  cameraUserExtrinsicPost: (_ip: string, action: UserExtrinsicAction = 'measure') =>
    (streamerCommand('POST', '/api/hal/camera_check/user_extrinsic', { action }) as Promise<{ user_extrinsic: UserExtrinsic }>).then((r) => r.user_extrinsic),
  cameraUserExtrinsicGet: (_ip: string) =>
    (streamerCommand('GET', '/api/hal/camera_check/user_extrinsic') as Promise<{ user_extrinsic: UserExtrinsic }>).then((r) => r.user_extrinsic),
};

export const WEBRTC_RES = [
  { label: 'HD 720p', width: 1280, height: 720 },
  { label: 'SQHD 540', width: 960, height: 540 },
  { label: '절약 360', width: 640, height: 360 },
] as const;

async function streamerReq<T = void>(
  _ip: string, path: string, method: 'GET' | 'PUT' | 'POST' | 'DELETE', body?: object,
): Promise<T> {
  const demo = demoDriver();
  if (demo) { demo.rest(path); return {} as T; }
  return (await streamerCommand(method, path, body ?? {})) as T;
}

export const VISION_PROGRAM = { Handeye: 3, Heightmap: 4, SLAMNAV_3D: 5, CCTV: 6, Ptz: 7, ThermalCam: 8 } as const;

export function restErrorText(e: unknown): string {
  if (e instanceof RemoteUnsupportedError) {
    return '이 로봇 소프트웨어는 원격에서 이 기능을 지원하지 않습니다 — 로봇을 업데이트하거나 같은 네트워크에서 연결하세요';
  }
  return e instanceof Error ? e.message : String(e);
}
