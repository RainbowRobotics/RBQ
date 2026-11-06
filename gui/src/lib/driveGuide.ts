import { OFFSETS } from '@/lib/robotPose';

const LEN_M = 5.0;
const DEADBAND = 0.05;
const MAX_CMD = 5;
const BASE_SPEED = 1.0;
const SHOULDER_MARGIN = 0.1;
const STEPS = 48;
const SHOULDER_X = OFFSETS.centerToLegX + OFFSETS.centerToHipX;

export type GuideSample = { x: number; y: number; th: number };

export type DriveGuide = {
  samples: GuideSample[];
  ticks: GuideSample[];
  halfWidth: number;
};

export function shoulderHalfWidth(wheel: boolean): number {
  return OFFSETS.centerToLegY + OFFSETS.hipToThigh + (wheel ? OFFSETS.wheelOffsetZ : 0) + SHOULDER_MARGIN;
}

export function edgePoint(s: GuideSample, off: number): [number, number] {
  return [s.x - Math.sin(s.th) * off, s.y + Math.cos(s.th) * off];
}

export function driveGuide(cmd: readonly [number, number, number], halfWidth: number): DriveGuide {
  let [vx, vy, wz] = cmd.map((v) => (Number.isFinite(v) && Math.abs(v) <= MAX_CMD ? v : 0));
  if (Math.abs(vx) < DEADBAND) { vx = BASE_SPEED; vy = 0; wz = 0; }
  else if (Math.abs(vy) < DEADBAND && Math.abs(wz) < DEADBAND) { vx = Math.sign(vx) * BASE_SPEED; vy = 0; wz = 0; }
  const speed = Math.hypot(vx, vy);
  const T = Math.min(LEN_M / speed, Math.abs(wz) > 1e-6 ? Math.PI / Math.abs(wz) : Infinity);
  const sx = vx < 0 ? -SHOULDER_X : SHOULDER_X;

  const at = (t: number): GuideSample => {
    const th = wz * t;
    let px: number, py: number;
    if (Math.abs(wz) < 1e-6) { px = vx * t; py = vy * t; }
    else {
      const s = Math.sin(th), c = Math.cos(th);
      px = (vx * s + vy * (c - 1)) / wz;
      py = (vx * (1 - c) + vy * s) / wz;
    }
    return { x: px + Math.cos(th) * sx, y: py + Math.sin(th) * sx, th };
  };

  const samples = Array.from({ length: STEPS + 1 }, (_, i) => at((T * i) / STEPS));
  const ticks: GuideSample[] = [];
  for (let d = 1; d < speed * T - 0.2; d += 1) ticks.push(at(d / speed));
  ticks.push(samples[STEPS]);
  return { samples, ticks, halfWidth };
}
