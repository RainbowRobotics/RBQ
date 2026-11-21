import type { BlackboxSession } from '@/lib/blackbox';

const R2D = 180 / Math.PI;

export function minMaxBuckets(sess: BlackboxSession, name: string, f0: number, f1: number, buckets: number, scale = 1):
  { min: Float32Array; max: Float32Array } | null {
  const col = sess.cols.get(name);
  if (col === undefined || buckets <= 0) return null;
  const a = Math.max(0, Math.floor(f0));
  const b = Math.min(sess.frameCount - 1, Math.ceil(f1));
  const min = new Float32Array(buckets).fill(NaN);
  const max = new Float32Array(buckets).fill(NaN);
  if (b < a) return { min, max };
  const span = b - a + 1;
  for (let k = 0; k < buckets; k++) {
    const s = a + Math.floor((k * span) / buckets);
    const e = Math.max(s + 1, a + Math.floor(((k + 1) * span) / buckets));
    let lo = Infinity, hi = -Infinity;
    for (let f = s; f < e && f <= b; f++) {
      const v = sess.data[f * sess.numCols + col] * scale;
      if (!Number.isFinite(v)) continue;
      if (v < lo) lo = v;
      if (v > hi) hi = v;
    }
    if (lo !== Infinity) { min[k] = lo; max[k] = hi; }
  }
  return { min, max };
}

export function niceRange(lo: number, hi: number, n = 4): { lo: number; hi: number; ticks: number[] } {
  if (!Number.isFinite(lo) || !Number.isFinite(hi)) return { lo: -1, hi: 1, ticks: [-1, 0, 1] };
  if (hi - lo < 1e-9) { lo -= 1; hi += 1; }
  const raw = (hi - lo) / n;
  const mag = 10 ** Math.floor(Math.log10(raw));
  const step = [1, 2, 5, 10].map((m) => m * mag).find((s) => s >= raw) ?? 10 * mag;
  const a = Math.floor(lo / step) * step;
  const b = Math.ceil(hi / step) * step;
  const ticks: number[] = [];
  for (let v = a; v <= b + step * 1e-6; v += step) ticks.push(Math.abs(v) < step * 1e-6 ? 0 : v);
  return { lo: a, hi: b, ticks };
}


export type Leg = 'FL' | 'FR' | 'HL' | 'HR';
export type LegSel = Leg | 'ALL';
export const LEG_ORDER: readonly Leg[] = ['FL', 'FR', 'HL', 'HR'];
export const LEG_BASE: Record<Leg, number> = { HR: 0, HL: 3, FR: 6, FL: 9 };
const JOINT_SUFFIX = ['R', 'P', 'K'] as const;
export const jointLabel = (i: number) => {
  const leg = (Object.keys(LEG_BASE) as Leg[]).find((l) => i >= LEG_BASE[l] && i < LEG_BASE[l] + 3)!;
  return `${leg}${JOINT_SUFFIX[i - LEG_BASE[leg]]}${i}`;
};

export type ChartLine = { name: string; label: string; color: string; dash?: boolean; scale?: number };
export type ChartSpec = { title: string; unit: string; lines: ChartLine[]; limits?: number[] };
export type PresetKey = 'overview' | 'torque' | 'current' | 'pos' | 'temp' | 'imu' | 'cpu' | 'power' | 'custom';
export const PRESETS: ReadonlyArray<{ key: PresetKey; label: string; perLeg: boolean }> = [
  { key: 'overview', label: '개요', perLeg: false },
  { key: 'torque', label: '토크', perLeg: true },
  { key: 'current', label: '전류', perLeg: true },
  { key: 'pos', label: '관절 위치', perLeg: true },
  { key: 'temp', label: '온도', perLeg: true },
  { key: 'imu', label: 'IMU', perLeg: false },
  { key: 'cpu', label: 'CPU·시스템', perLeg: false },
  { key: 'power', label: '전원', perLeg: false },
  { key: 'custom', label: '사용자 지정', perLeg: false },
];

export const LINE_COLORS = ['#e5534b', '#4d9cf5', '#3fb950', '#d29922', '#bc8cff', '#39c5cf'];

const LEG_CUR_LIMIT = 15;

function legCharts(key: PresetKey, leg: Leg, custom: string[]): ChartSpec[] {
  const base = LEG_BASE[leg];
  const joints = [0, 1, 2].map((k) => base + k);
  switch (key) {
    case 'torque':
      return joints.map((j) => ({
        title: jointLabel(j), unit: 'Nm', lines: [
          { name: `joint.torque[${j}]`, label: 'sens', color: LINE_COLORS[0] },
          { name: `ref.joint.torque[${j}]`, label: 'ref', color: LINE_COLORS[1], dash: true },
        ],
      }));
    case 'current':
      return joints.map((j) => ({
        title: jointLabel(j), unit: 'A', limits: [-LEG_CUR_LIMIT, LEG_CUR_LIMIT],
        lines: [{ name: `motor.cur[${j}]`, label: 'cur', color: LINE_COLORS[0] }],
      }));
    case 'pos':
      return joints.map((j) => ({
        title: jointLabel(j), unit: '°', lines: [
          { name: `joint.pos[${j}]`, label: 'sens', color: LINE_COLORS[0], scale: R2D },
          { name: `ref.joint.pos[${j}]`, label: 'ref', color: LINE_COLORS[1], dash: true, scale: R2D },
        ],
      }));
    case 'temp':
      return [
        { title: `${leg} motor`, unit: '°C',
          lines: joints.map((j, k) => ({ name: `motor.temp[${j}]`, label: jointLabel(j), color: LINE_COLORS[k] })) },
        { title: `${leg} board`, unit: '°C',
          lines: joints.map((j, k) => ({ name: `board.temp[${j}]`, label: jointLabel(j), color: LINE_COLORS[k] })) },
      ];
    case 'imu':
      return [
        { title: 'RPY', unit: '°', lines: (['r', 'p', 'y'] as const).map((a, k) =>
          ({ name: `imu.rpy.${a}`, label: a.toUpperCase(), color: LINE_COLORS[k], scale: R2D })) },
        { title: 'Gyro', unit: 'rad/s', lines: (['x', 'y', 'z'] as const).map((a, k) =>
          ({ name: `imu.gyro.${a}`, label: a, color: LINE_COLORS[k] })) },
        { title: 'Acc', unit: 'm/s²', lines: (['x', 'y', 'z'] as const).map((a, k) =>
          ({ name: `imu.acc.${a}`, label: a, color: LINE_COLORS[k] })) },
      ];
    case 'cpu':
      return [
        { title: 'CPU temp', unit: '°C', lines: [{ name: 'cpu.temp', label: 'temp', color: LINE_COLORS[0] }] },
        { title: 'CPU throttle', unit: '', lines: [{ name: 'cpu.throttled', label: 'throttled', color: LINE_COLORS[0] }] },
        { title: 'CPU core', unit: '%', lines: [0, 1, 2, 3].map((i) =>
          ({ name: `cpu.core[${i}]`, label: `core${i}`, color: LINE_COLORS[i] })) },
        { title: 'Deadline miss', unit: '', lines: [
          { name: 'deadline_miss', label: 'total', color: LINE_COLORS[0] },
          { name: 'deadline_miss.motion', label: 'motion', color: LINE_COLORS[1] },
          { name: 'deadline_miss.wbc', label: 'wbc', color: LINE_COLORS[2] },
        ] },
        { title: 'Process time', unit: 'ms', lines: [{ name: 'process_time_ms', label: 'proc', color: LINE_COLORS[1] }] },
      ];
    case 'power':
      return [
        { title: 'Battery V', unit: 'V', lines: [
          { name: 'pdu.bat.left.voltage', label: 'L', color: LINE_COLORS[0] },
          { name: 'pdu.bat.right.voltage', label: 'R', color: LINE_COLORS[1] },
        ] },
        { title: 'Battery I', unit: 'A', lines: [
          { name: 'pdu.bat.left.current', label: 'L', color: LINE_COLORS[0] },
          { name: 'pdu.bat.right.current', label: 'R', color: LINE_COLORS[1] },
        ] },
        { title: 'LEG 48V', unit: '', lines: [
          { name: 'pdu.out.leg_48v.v', label: 'V', color: LINE_COLORS[2] },
          { name: 'pdu.out.leg_48v.i', label: 'I', color: LINE_COLORS[3] },
        ] },
      ];
    case 'overview': {
      const knees = LEG_ORDER.map((l) => LEG_BASE[l] + 2);
      return [
        { title: 'RPY', unit: '°', lines: (['r', 'p', 'y'] as const).map((a, k) =>
          ({ name: `imu.rpy.${a}`, label: a.toUpperCase(), color: LINE_COLORS[k], scale: R2D })) },
        { title: 'Gyro', unit: 'rad/s', lines: (['x', 'y', 'z'] as const).map((a, k) =>
          ({ name: `imu.gyro.${a}`, label: a, color: LINE_COLORS[k] })) },
        { title: 'CMD', unit: '', lines: [
          { name: 'cmd.vel_x', label: 'vx', color: LINE_COLORS[0] },
          { name: 'cmd.vel_y', label: 'vy', color: LINE_COLORS[1] },
          { name: 'cmd.omega_z', label: 'wz', color: LINE_COLORS[2] },
        ] },
        { title: 'Knee torque', unit: 'Nm', lines: knees.map((j, k) =>
          ({ name: `joint.torque[${j}]`, label: LEG_ORDER[k], color: LINE_COLORS[k] })) },
        { title: 'Knee current', unit: 'A', limits: [-LEG_CUR_LIMIT, LEG_CUR_LIMIT], lines: knees.map((j, k) =>
          ({ name: `motor.cur[${j}]`, label: LEG_ORDER[k], color: LINE_COLORS[k] })) },
        { title: 'Battery V', unit: 'V', lines: [
          { name: 'pdu.bat.left.voltage', label: 'L', color: LINE_COLORS[0] },
          { name: 'pdu.bat.right.voltage', label: 'R', color: LINE_COLORS[1] },
        ] },
        { title: 'CPU temp', unit: '°C', lines: [{ name: 'cpu.temp', label: 'temp', color: LINE_COLORS[0] }] },
        { title: 'Deadline miss', unit: '', lines: [{ name: 'deadline_miss', label: 'total', color: LINE_COLORS[0] }] },
      ];
    }
    case 'custom':
      return custom.length ? [{ title: 'Custom', unit: '',
        lines: custom.map((n, k) => ({ name: n, label: n, color: LINE_COLORS[k % LINE_COLORS.length] })) }] : [];
  }
}

export function presetCharts(key: PresetKey, leg: LegSel, custom: string[]): ChartSpec[] {
  const perLeg = PRESETS.find((p) => p.key === key)!.perLeg;
  if (perLeg && leg === 'ALL') {
    if (key === 'temp') return LEG_ORDER.map((l) => {
      const [motor, board] = legCharts('temp', l, custom);
      return { title: `${l} temp`, unit: '°C', lines: [...motor.lines, ...board.lines.map((ln) => ({ ...ln, label: `${ln.label} brd`, dash: true }))] };
    });
    return LEG_ORDER.flatMap((l) => legCharts(key, l, custom));
  }
  return legCharts(key, leg === 'ALL' ? 'FL' : leg, custom);
}

export function presetCols(key: PresetKey): number {
  const perLeg = PRESETS.find((p) => p.key === key)!.perLeg;
  if (key === 'custom') return 1;
  if (perLeg) return key === 'temp' ? 2 : 3;
  return key === 'overview' ? 4 : 3;
}

export const CUSTOM_MAX = 6;
