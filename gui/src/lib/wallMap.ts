import { create } from 'zustand';

export const WALLMAP_MAGIC = 0x574d;
const HEADER_BYTES = 2 + 4 + 4 + 4 + 64;

export type WallMapFrame = {
  rows: number;
  cols: number;
  gs: number;
  cells: Uint8Array;
  at: number;
};

type WallMapStore = {
  frame?: WallMapFrame;
  apply: (f: WallMapFrame) => void;
  reset: () => void;
};

export const useWallMap = create<WallMapStore>((set) => ({
  frame: undefined,
  apply: (f) => set({ frame: f }),
  reset: () => set({ frame: undefined }),
}));

export function handleWallMapFrame(data: ArrayBuffer) {
  if (data.byteLength <= HEADER_BYTES) return;
  const dv = new DataView(data);
  if (dv.getUint16(0, true) !== WALLMAP_MAGIC) return;
  const rows = dv.getInt32(2, true);
  const cols = dv.getInt32(6, true);
  const gs = dv.getFloat32(10, true);
  const n = rows * cols;
  if (rows <= 0 || cols <= 0 || n <= 0) return;
  if (data.byteLength < HEADER_BYTES + n) return;
  const cells = new Uint8Array(data, HEADER_BYTES, n);
  useWallMap.getState().apply({ rows, cols, gs, cells, at: Date.now() });
}

export function attachWallmapChannel(pc: any) {
  try {
    const dc = pc.createDataChannel('wall-map');
    dc.binaryType = 'arraybuffer';
    dc.onmessage = (e: any) => {
      const d = e?.data;
      if (d instanceof ArrayBuffer) handleWallMapFrame(d);
    };
  } catch {}
}
