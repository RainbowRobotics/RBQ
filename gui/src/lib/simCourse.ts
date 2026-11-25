import LEVELS_JSON from './sim/levels.json';

export type LevelBox = {
  x: number; y: number; z: number; hx: number; hy: number; hz: number; m: string;
  pitch?: number; roll?: number; yaw?: number;
};
export type Gate = { x: number; y: number; z: number; r?: number; facing?: number; tol?: number };
export type Level = {
  id: string; no: number; name: string; theme?: string; teach?: string; ref?: string;
  spawn: { x: number; y: number; z: number; yaw: number };
  gates: Gate[];
  goal: { x: number; y: number; z: number; hx: number; hy: number };
  boxes: LevelBox[];
};

export const LEVELS = (LEVELS_JSON as unknown as { levels: Level[] }).levels;
export const LEVEL_COLORS = (LEVELS_JSON as unknown as { colors: Record<string, string> }).colors;
export const levelById = (id: string) => LEVELS.find((l) => l.id === id);

export const GATE_R = 0.9;
const BODY_MIN = 0.15, BODY_MAX = 1.0;

export function killZ(level: Level) {
  let low = level.spawn.z;
  for (const b of level.boxes) low = Math.min(low, b.z + b.hz);
  return low - 1.2;
}

const wrapDeg = (a: number) => ((((a + 180) % 360) + 360) % 360) - 180;
const RAD = Math.PI / 180;

export function groundAt(level: Level, x: number, y: number): number | null {
  let best: number | null = null;
  for (const b of level.boxes) {
    let top: number | null = null;
    if (b.pitch) {
      const f = b.pitch * RAD, nx = Math.sin(f), nz = Math.cos(f);
      if (Math.abs(y - b.y) <= b.hy && Math.abs(x - b.x) <= b.hx * Math.abs(nz) + 0.02) {
        const px = b.x + b.hz * nx, pz = b.z + b.hz * nz;
        top = pz - (nx * (x - px)) / nz;
      }
    } else if (b.roll) {
      const r = b.roll * RAD, ny = -Math.sin(r), nz = Math.cos(r);
      if (Math.abs(x - b.x) <= b.hx && Math.abs(y - b.y) <= b.hy * Math.abs(nz) + 0.02) {
        const py = b.y + b.hz * ny, pz = b.z + b.hz * nz;
        top = pz - (ny * (y - py)) / nz;
      }
    } else {
      const a = (b.yaw ?? 0) * RAD, dx = x - b.x, dy = y - b.y;
      const u = Math.cos(a) * dx + Math.sin(a) * dy, v = -Math.sin(a) * dx + Math.cos(a) * dy;
      if (Math.abs(u) <= b.hx && Math.abs(v) <= b.hy) top = b.z + b.hz;
    }
    if (top != null && (best == null || top > best)) best = top;
  }
  return best;
}

function footprint(level: Level, x: number, y: number, yawDeg: number) {
  const a = yawDeg * RAD, c = Math.cos(a), s = Math.sin(a), out: (number | null)[] = [];
  for (const f of [-0.35, 0, 0.35]) for (const l of [-0.2, 0, 0.2]) out.push(groundAt(level, x + c * f - s * l, y + s * f + c * l));
  return out;
}

export function footprintGround(level: Level, x: number, y: number, yawDeg: number): number | null {
  let best: number | null = null;
  for (const g of footprint(level, x, y, yawDeg)) if (g != null && (best == null || g > best)) best = g;
  return best;
}

export type Pose = { x: number; y: number; z: number; yaw: number };
export type Progress = { passed: number; done: boolean; lowSince?: number | null; lowAt?: [number, number]; fallAt?: number };
export const FALL_HOLD_MS = 500;
export const STUCK_H = 0.33, STUCK_MS = 3000;
export type CourseEvent =
  | { t: 'gate'; n: number; total: number }
  | { t: 'clear' }
  | { t: 'fall'; to: { x: number; y: number; z: number; yaw: number }; stuck?: boolean }
  | { t: 'wrongFacing'; n: number };

function onTop(z: number, groundZ: number) {
  const h = z - groundZ;
  return h >= BODY_MIN && h <= BODY_MAX;
}

export function respawnPoint(level: Level, passed: number) {
  if (passed <= 0) return { ...level.spawn };
  const g = level.gates[passed - 1];
  const next = level.gates[passed] ?? level.goal;
  const yaw = g.facing ?? (Math.atan2(next.y - g.y, next.x - g.x) * 180) / Math.PI;
  const z = footprintGround(level, g.x, g.y, yaw) ?? g.z;
  return { x: g.x, y: g.y, z, yaw };
}

export function stepCourse(level: Level, prog: Progress, pose: Pose, now = 0, walking = false): { prog: Progress; ev: CourseEvent | null } {
  if (prog.done) return { prog, ev: null };
  if (prog.fallAt && now - prog.fallAt < FALL_HOLD_MS) return { prog, ev: null };
  if (pose.z < killZ(level)) return { prog: { ...prog, lowSince: null, fallAt: now || 1 }, ev: { t: 'fall', to: respawnPoint(level, prog.passed) } };
  if (walking) {
    const fp = footprint(level, pose.x, pose.y, (pose.yaw * 180) / Math.PI);
    const straddle = fp.some((v) => v == null) && fp.some((v) => v != null);
    let g: number | null = null;
    for (const v of fp) if (v != null && (g == null || v < g)) g = v;
    const low = straddle && g != null && pose.z - g < STUCK_H;
    const moved = prog.lowAt ? Math.hypot(pose.x - prog.lowAt[0], pose.y - prog.lowAt[1]) > 0.12 : true;
    if (!low) prog = prog.lowSince ? { ...prog, lowSince: null } : prog;
    else if (!prog.lowSince || moved) prog = { ...prog, lowSince: now || 1, lowAt: [pose.x, pose.y] };
    else if (now - prog.lowSince > STUCK_MS) {
      return { prog: { ...prog, lowSince: null, fallAt: now || 1 }, ev: { t: 'fall', to: respawnPoint(level, prog.passed), stuck: true } };
    }
  } else if (prog.lowSince) prog = { ...prog, lowSince: null };

  const g = level.gates[prog.passed];
  if (g) {
    const inside = Math.hypot(pose.x - g.x, pose.y - g.y) <= (g.r ?? GATE_R) && onTop(pose.z, g.z);
    if (!inside) return { prog, ev: null };
    if (g.facing != null) {
      const err = Math.abs(wrapDeg((pose.yaw * 180) / Math.PI - g.facing));
      if (err > (g.tol ?? 35)) return { prog, ev: { t: 'wrongFacing', n: prog.passed + 1 } };
    }
    const passed = prog.passed + 1;
    return { prog: { passed, done: false, lowSince: prog.lowSince }, ev: { t: 'gate', n: passed, total: level.gates.length } };
  }

  const G = level.goal;
  if (Math.abs(pose.x - G.x) <= G.hx && Math.abs(pose.y - G.y) <= G.hy && onTop(pose.z, G.z)) {
    return { prog: { passed: prog.passed, done: true }, ev: { t: 'clear' } };
  }
  return { prog, ev: null };
}

export function fmtTime(sec: number) {
  const m = Math.floor(sec / 60), s = sec - m * 60;
  return `${m}:${s < 10 ? '0' : ''}${s.toFixed(1)}`;
}
