
export const OFFSETS = {
  centerToLegX: 0.19725,
  centerToLegY: 0.09,
  centerToHipX: 0.11493,
  hipToThigh: 0.10285,
  thighToKnee: 0.33,
  kneeToFoot: 0.33,
  footRadius: 0.03,
  kneeToWheel: 0.29,
  wheelOffsetZ: 0.0745,
  wheelRadius: 0.1025,
};

import type { RobotMeshKey } from '@/lib/robotMeshes';

export type LegDef = {
  name: 'RR' | 'RL' | 'FR' | 'FL';
  base: number;
  front: 1 | -1;
  right: 1 | -1;
  hip: RobotMeshKey;
  hipRear: boolean;
};

export const LEGS: LegDef[] = [
  { name: 'RR', base: 0, front: -1, right: 1, hip: 'hip3', hipRear: true },
  { name: 'RL', base: 3, front: -1, right: -1, hip: 'hip2', hipRear: true },
  { name: 'FR', base: 6, front: 1, right: 1, hip: 'hip2', hipRear: false },
  { name: 'FL', base: 9, front: 1, right: -1, hip: 'hip3', hipRear: false },
];

export const d2r = (deg: number) => (deg * Math.PI) / 180;

export const STANDING_RAD: number[] = [0, 50, -90, 0, 50, -90, 0, 50, -90, 0, 50, -90].map(d2r);

export function footCenters(joints: number[], out: number[][], wheel = false): void {
  const O = OFFSETS;
  const L2 = wheel ? O.kneeToWheel : O.kneeToFoot;
  LEGS.forEach((def, i) => {
    const abd = joints[def.base] ?? 0;
    const hip = joints[def.base + 1] ?? 0;
    const knee = joints[def.base + 2] ?? 0;
    const x = -O.thighToKnee * Math.sin(hip) - L2 * Math.sin(hip + knee);
    const y = -O.thighToKnee * Math.cos(hip) - L2 * Math.cos(hip + knee);
    const z = def.right * (O.hipToThigh + (wheel ? O.wheelOffsetZ : 0));
    const c = Math.cos(abd), s = Math.sin(abd);
    const p = out[i];
    p[0] = x + def.front * (O.centerToLegX + O.centerToHipX);
    p[1] = y * c - z * s;
    p[2] = y * s + z * c + def.right * O.centerToLegY;
  });
}
