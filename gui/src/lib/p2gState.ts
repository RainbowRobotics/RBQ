export const ID_P2G_STATE = 27;
export const P2G_STATE_SIZE = 32;

export const P2G_STATE = { idle: 0, moving: 1 } as const;

export type P2gState = {
  state: number;
  visible: boolean;
  u: number;
  v: number;
  distHorizontal: number;
  fxNorm: number;
  arriveRemainSec: number;
};

export function parseP2gState(data: ArrayBuffer, off: number): P2gState {
  const dv = new DataView(data);
  return {
    state: dv.getUint8(off + 8),
    visible: dv.getUint8(off + 9) !== 0,
    u: dv.getFloat32(off + 12, true),
    v: dv.getFloat32(off + 16, true),
    distHorizontal: dv.getFloat32(off + 20, true),
    arriveRemainSec: dv.getFloat32(off + 24, true),
    fxNorm: dv.getFloat32(off + 28, true),
  };
}
