

export type SlamnavFlags = {
  connected: boolean; mapLoaded: boolean; slam: boolean; loc: boolean; sim: boolean;
  recording: boolean; quickAnnotation: boolean; quickNoise: boolean;
  tasking: boolean; scheduleRunning: boolean; recovery: boolean; autoInit: boolean;
};

export type SlamnavState = {
  v: number;
  t: number;
  mode: 'DRIVE' | 'ANNOTATION' | string;
  boot: string;
  busy: boolean;
  flags: SlamnavFlags;
  map: { dir: string; name: string; version: number; topoVersion: number; pathVersion: number; points: number };
  pose: { x: number; y: number; z: number; roll: number; pitch: number; yaw: number };
  loc: { state: string; ie: number; ir: number };
  robot: {
    link: string; battery: string; gait: number; extJoy: boolean; docking: number;
    vel: [number, number, number]; cmd: [number, number, number];
    size: { x: [number, number]; y: [number, number]; z: [number, number] };
  };
  ctrl: { state: 'stop' | 'move' | 'pause' | string; fsm: string; goal: string; obs: string; zone: string;
    localPath: [number, number, number, number][] };
  task: { phase: string; fsm: string; nodes: string[]; odo: number };
  schedule: { entries: string[]; fsm: string };
};

const FLAG_KEYS: (keyof SlamnavFlags)[] = ['connected', 'mapLoaded', 'slam', 'loc', 'sim', 'recording',
  'quickAnnotation', 'quickNoise', 'tasking', 'scheduleRunning', 'recovery', 'autoInit'];
const DEFAULT_SIZE = { x: [-0.65, 0.35] as [number, number], y: [-0.25, 0.25] as [number, number], z: [-0.47, 0.2] as [number, number] };

export function utf8(bytes: Uint8Array): string {
  const TD = (globalThis as { TextDecoder?: new () => { decode: (b: Uint8Array) => string } }).TextDecoder;
  if (TD) return new TD().decode(bytes);
  let out = '';
  for (let i = 0; i < bytes.length;) {
    const b = bytes[i++];
    let cp = b;
    if (b >= 0xf0) cp = ((b & 7) << 18) | ((bytes[i++] & 63) << 12) | ((bytes[i++] & 63) << 6) | (bytes[i++] & 63);
    else if (b >= 0xe0) cp = ((b & 15) << 12) | ((bytes[i++] & 63) << 6) | (bytes[i++] & 63);
    else if (b >= 0xc0) cp = ((b & 31) << 6) | (bytes[i++] & 63);
    out += String.fromCodePoint(cp);
  }
  return out;
}

const num = (v: unknown, d = 0) => (typeof v === 'number' && isFinite(v) ? v : d);
const str = (v: unknown, d = '') => (typeof v === 'string' ? v : d);
const vec3 = (v: unknown): [number, number, number] =>
  Array.isArray(v) ? [num(v[0]), num(v[1]), num(v[2])] : [0, 0, 0];
const range = (v: unknown, d: [number, number]): [number, number] =>
  Array.isArray(v) && v.length >= 2 ? [num(v[0], d[0]), num(v[1], d[1])] : d;

export function parseSlamnavState(data: string | ArrayBuffer | Uint8Array): SlamnavState | null {
  let o: any;
  try {
    const text = typeof data === 'string' ? data : utf8(data instanceof Uint8Array ? data : new Uint8Array(data));
    o = JSON.parse(text);
  } catch { return null; }
  if (!o || typeof o !== 'object') return null;
  const f = o.flags ?? {};
  const flags = {} as SlamnavFlags;
  for (const k of FLAG_KEYS) flags[k] = !!f[k];
  const m = o.map ?? {}, p = o.pose ?? {}, l = o.loc ?? {}, r = o.robot ?? {}, c = o.ctrl ?? {};
  const task = o.task ?? {}, sch = o.schedule ?? {}, sz = r.size ?? {};
  return {
    v: num(o.v, 1), t: num(o.t), mode: str(o.mode, 'DRIVE'), boot: str(o.boot), busy: !!o.busy, flags,
    map: { dir: str(m.dir), name: str(m.name), version: num(m.version), topoVersion: num(m.topoVersion),
      pathVersion: num(m.pathVersion), points: num(m.points) },
    pose: { x: num(p.x), y: num(p.y), z: num(p.z), roll: num(p.roll), pitch: num(p.pitch), yaw: num(p.yaw) },
    loc: { state: str(l.state, 'none'), ie: num(l.ie), ir: num(l.ir) },
    robot: {
      link: str(r.link, 'none'), battery: str(r.battery, 'none'), gait: num(r.gait), extJoy: !!r.extJoy,
      docking: num(r.docking), vel: vec3(r.vel), cmd: vec3(r.cmd),
      size: { x: range(sz.x, DEFAULT_SIZE.x), y: range(sz.y, DEFAULT_SIZE.y), z: range(sz.z, DEFAULT_SIZE.z) },
    },
    ctrl: {
      state: str(c.state, 'stop'), fsm: str(c.fsm), goal: str(c.goal), obs: str(c.obs), zone: str(c.zone),
      localPath: Array.isArray(c.localPath)
        ? c.localPath.filter((q: unknown) => Array.isArray(q)).map((q: number[]) => [num(q[0]), num(q[1]), num(q[2]), num(q[3])] as [number, number, number, number])
        : [],
    },
    task: { phase: str(task.phase), fsm: str(task.fsm), nodes: Array.isArray(task.nodes) ? task.nodes.map(String) : [], odo: num(task.odo) },
    schedule: { entries: Array.isArray(sch.entries) ? sch.entries.map(String) : [], fsm: str(sch.fsm) },
  };
}


export type SlamnavResults = { mapSaved?: boolean; mapReloaded?: boolean; annotationSaved?: boolean;
  route?: 'start' | 'return' };

export type SlamUiState = {
  mapBuilding: boolean; mapLoaded: boolean; mapSaved: boolean; mapReloaded: boolean;
  annotationMode: boolean; quickAnnotation: boolean; annotationSaved: boolean;
  locaStarted: boolean; locaInit: boolean;
  navActive: boolean;
  driveStarted: boolean; autoTravel: boolean;
  tasking: boolean; scheduleRunning: boolean; busy: boolean;
  raw: SlamnavState;
};

export function deriveSlamUi(s: SlamnavState, r: SlamnavResults = {}): SlamUiState {
  const navActive = s.ctrl.state !== 'stop' || s.flags.tasking || s.flags.scheduleRunning;
  return {
    mapBuilding: s.flags.slam, mapLoaded: s.flags.mapLoaded,
    mapSaved: !!r.mapSaved, mapReloaded: !!r.mapReloaded,
    annotationMode: s.mode === 'ANNOTATION', quickAnnotation: s.flags.quickAnnotation,
    annotationSaved: !!r.annotationSaved,
    locaStarted: s.flags.loc, locaInit: s.flags.autoInit,
    navActive,
    driveStarted: navActive && r.route === 'start',
    autoTravel: navActive && r.route === 'return',
    tasking: s.flags.tasking, scheduleRunning: s.flags.scheduleRunning, busy: s.busy,
    raw: s,
  };
}


export const CLOUD_MAGIC = 0x4c53;
export const CLOUD_HEADER = 48;
export const CLOUD_KIND = { SCAN: 1, OBS: 2, LIVE_MAP: 3 } as const;
export type CloudKind = 1 | 2 | 3;

export type CloudPart = {
  kind: CloudKind; seq: number; part: number; parts: number; time: number;
  positions: Float32Array;
  values: Uint8Array | null;
  count: number;
};

export function parseCloudFrame(buf: ArrayBuffer): CloudPart | null {
  if (buf.byteLength < CLOUD_HEADER) return null;
  const dv = new DataView(buf);
  if (dv.getUint16(0, true) !== CLOUD_MAGIC) return null;
  const kind = dv.getUint8(2);
  if (kind !== 1 && kind !== 2 && kind !== 3) return null;
  const seq = dv.getUint32(4, true);
  const part = dv.getUint16(8, true);
  const parts = Math.max(1, dv.getUint16(10, true));
  const time = dv.getFloat64(12, true);
  const ox = dv.getFloat64(20, true), oy = dv.getFloat64(28, true), oz = dv.getFloat64(36, true);
  const count = dv.getUint32(44, true);
  const stride = kind === 1 ? 6 : 7;
  if (buf.byteLength < CLOUD_HEADER + count * stride) return null;
  const positions = new Float32Array(count * 3);
  const values = kind === 1 ? null : new Uint8Array(count);
  let o = CLOUD_HEADER;
  for (let i = 0; i < count; i++, o += stride) {
    positions[i * 3] = ox + dv.getInt16(o, true) / 100;
    positions[i * 3 + 1] = oy + dv.getInt16(o + 2, true) / 100;
    positions[i * 3 + 2] = oz + dv.getInt16(o + 4, true) / 100;
    if (values) values[i] = dv.getUint8(o + 6);
  }
  return { kind: kind as CloudKind, seq, part, parts, time, positions, values, count };
}

export type CloudSet = { kind: CloudKind; seq: number; positions: Float32Array; values: Uint8Array | null; count: number };

export class CloudAssembler {
  private pending = new Map<CloudKind, { seq: number; parts: (CloudPart | undefined)[]; got: number; at: number }>();

  push(p: CloudPart): CloudSet | null {
    if (p.parts === 1) { this.pending.delete(p.kind); return { kind: p.kind, seq: p.seq, positions: p.positions, values: p.values, count: p.count }; }
    let cur = this.pending.get(p.kind);
    if (cur && cur.seq !== p.seq && ((cur.seq - p.seq) >>> 0) < 0x80000000 && Date.now() - cur.at < 2000) return null;
    if (!cur || cur.seq !== p.seq || cur.parts.length !== p.parts) {
      cur = { seq: p.seq, parts: new Array(p.parts), got: 0, at: Date.now() };
      this.pending.set(p.kind, cur);
    }
    if (p.part >= p.parts || cur.parts[p.part]) return null;
    cur.parts[p.part] = p;
    cur.got++;
    if (cur.got < p.parts) return null;
    this.pending.delete(p.kind);
    const all = cur.parts as CloudPart[];
    const count = all.reduce((n, x) => n + x.count, 0);
    const positions = new Float32Array(count * 3);
    const values = p.values ? new Uint8Array(count) : null;
    let n = 0;
    for (const x of all) {
      positions.set(x.positions, n * 3);
      if (values && x.values) values.set(x.values, n);
      n += x.count;
    }
    return { kind: p.kind, seq: p.seq, positions, values, count };
  }

  reset() { this.pending.clear(); }
}


export const MAP_MAGIC = 0x50414d53;

export type MapCloud = { version: number; count: number; positions: Float32Array; intensity: Uint8Array; label: Uint8Array };

export function parseMapCloud(buf: ArrayBuffer): MapCloud | null {
  if (buf.byteLength < 16) return null;
  const dv = new DataView(buf);
  if (dv.getUint32(0, true) !== MAP_MAGIC) return null;
  const version = dv.getUint32(4, true);
  const n = dv.getUint32(8, true);
  const need = 16 + n * 12 + n + n * 3 + n;
  if (buf.byteLength < need) return null;
  const positions = new Float32Array(buf.slice(16, 16 + n * 12));
  const iOff = 16 + n * 12;
  const intensity = new Uint8Array(buf.slice(iOff, iOff + n));
  const lOff = iOff + n + n * 3;
  const label = new Uint8Array(buf.slice(lOff, lOff + n));
  return { version, count: n, positions, intensity, label };
}

export const MAP_POINT_BUDGET = 300_000;
export function mapStride(points: number, budget = MAP_POINT_BUDGET): number {
  return points > budget ? Math.ceil(points / budget) : 1;
}


type Stops = [number, number, number][];
export const MAP_STOPS: Stops = [
  [0.12, 0.23, 0.45],
  [0.18, 0.50, 0.75],
  [0.25, 0.76, 0.69],
  [0.78, 0.94, 0.88],
];
export const SCAN_STOPS: Stops = [
  [0.85, 0.16, 0.20],
  [1.00, 0.55, 0.10],
  [1.00, 0.86, 0.35],
];

function heightRamp(stops: Stops, t: number, out: Float32Array, o: number) {
  const x = Math.min(1, Math.max(0, t)) * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(x));
  const f = x - i, a = stops[i], b = stops[i + 1];
  out[o] = a[0] + (b[0] - a[0]) * f;
  out[o + 1] = a[1] + (b[1] - a[1]) * f;
  out[o + 2] = a[2] + (b[2] - a[2]) * f;
}

export function heightRange(positions: Float32Array, count: number): [number, number] {
  let lo = Infinity, hi = -Infinity;
  for (let i = 0; i < count; i++) { const z = positions[i * 3 + 2]; if (z < lo) lo = z; if (z > hi) hi = z; }
  if (!(hi > lo)) return [lo === Infinity ? 0 : lo, (lo === Infinity ? 0 : lo) + 1];
  const BINS = 256, hist = new Uint32Array(BINS), k = (BINS - 1) / (hi - lo);
  for (let i = 0; i < count; i++) hist[Math.round((positions[i * 3 + 2] - lo) * k)]++;
  const cut = count * 0.02;
  let a = 0, b = BINS - 1, acc = 0;
  for (; a < BINS - 1; a++) { acc += hist[a]; if (acc > cut) break; }
  acc = 0;
  for (; b > a; b--) { acc += hist[b]; if (acc > cut) break; }
  const z0 = lo + a / k, z1 = lo + b / k;
  return z1 - z0 > 0.2 ? [z0, z1] : [z0 - 0.1, z0 + 0.1];
}

export function heightColors(positions: Float32Array, count: number, label?: Uint8Array, stops: Stops = MAP_STOPS): Float32Array {
  const out = new Float32Array(count * 3);
  const [z0, z1] = heightRange(positions, count);
  const inv = 1 / (z1 - z0);
  for (let i = 0; i < count; i++) {
    if (label && label[i] === 1) { out[i * 3] = 1; out[i * 3 + 1] = 0.1; out[i * 3 + 2] = 0.1; continue; }
    heightRamp(stops, (positions[i * 3 + 2] - z0) * inv, out, i * 3);
  }
  return out;
}

