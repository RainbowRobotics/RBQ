import { BoxGeometry, BufferGeometry, CylinderGeometry, ExtrudeGeometry, Path, RingGeometry, Shape } from 'three';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';

export const WHEEL_MAX_R = 0.1053;
const TREAD_N = 22;
export const TREAD_PITCH = (2 * Math.PI) / TREAD_N;
const PLATE_Z = 0.0235;
const PLATE_T = 0.004;
const PLATE_R = 0.0748;

export type WheelFaceGeos = { motor: BufferGeometry; bronze: BufferGeometry; plate: BufferGeometry; bolts: BufferGeometry; tread: BufferGeometry };

const disc = (r: number, h: number, z: number, x = 0, y = 0, seg = 24) => {
  const g = new CylinderGeometry(r, r, h, seg); g.rotateX(Math.PI / 2); g.translate(x, y, z); return g;
};
const extrude = (s: Shape) => {
  const g = new ExtrudeGeometry(s, { depth: PLATE_T, bevelEnabled: false, curveSegments: 32 }); g.translate(0, 0, PLATE_Z); return g;
};

function build(): WheelFaceGeos {
  const spokeAngles = [0, 1, 2, 3, 4].map((k) => Math.PI / 2 + (k * 2 * Math.PI) / 5);

  const motor = mergeGeometries([disc(0.06, 0.004, 0.004), disc(0.0065, 0.0004, PLATE_Z + PLATE_T + 0.0002)]);
  const bronze = new RingGeometry(0.040, 0.047, 64); bronze.translate(0, 0, 0.012);

  const ring = new Shape(); ring.absarc(0, 0, PLATE_R, 0, Math.PI * 2, false);
  const hole = new Path(); hole.absarc(0, 0, 0.066, 0, Math.PI * 2, true); ring.holes.push(hole);
  const hub = new Shape(); hub.absarc(0, 0, 0.018, 0, Math.PI * 2, false);
  const spokes = spokeAngles.map((a) => {
    const s = new Shape();
    s.moveTo(0.010, -0.0085); s.lineTo(0.068, -0.0065); s.lineTo(0.068, 0.0065); s.lineTo(0.010, 0.0085); s.closePath();
    const g = extrude(s); g.rotateZ(a); return g;
  });
  const plate = mergeGeometries([extrude(ring), extrude(hub), ...spokes]);

  const top = PLATE_Z + PLATE_T + 0.0006;
  const bolts = mergeGeometries([
    ...spokeAngles.map((a) => disc(0.0028, 0.0012, top, 0.0705 * Math.cos(a), 0.0705 * Math.sin(a), 8)),
    ...[0, 1, 2, 3, 4, 5].map((k) => { const a = Math.PI / 6 + (k * Math.PI) / 3; return disc(0.0018, 0.0012, top, 0.0125 * Math.cos(a), 0.0125 * Math.sin(a), 8); }),
  ]);

  const N = TREAD_N, blocks: BufferGeometry[] = [];
  for (let i = 0; i < N; i++) {
    for (const side of [1, -1]) {
      const g = new BoxGeometry(0.010, 0.013, 0.025);
      g.rotateX(side * 0.25);
      g.translate(0.0995, 0, side * 0.0155);
      g.rotateZ(((i + (side > 0 ? 0 : 0.5)) * 2 * Math.PI) / N);
      blocks.push(g);
    }
  }
  const tread = mergeGeometries(blocks);
  return { motor, bronze, plate, bolts, tread };
}

let GEOS: WheelFaceGeos | null = null;
export const wheelFaceGeos = (): WheelFaceGeos => (GEOS ??= build());
