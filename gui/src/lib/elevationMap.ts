import { create } from 'zustand';
import { useTelemetry } from '@/store/telemetry';

export const ELEVATION_MAGIC = 0x454d;
const HEADER_BYTES = 2 + 4 + 4 + 4 + 4 + 4;

export type ElevationFrame = {
  rows: number;
  cols: number;
  gs: number;
  originX: number;
  originY: number;
  height: Float32Array;
  valid: Uint8Array;
  at: number;
  robotZ: number;
};

type ElevationMapStore = {
  frame?: ElevationFrame;
  apply: (f: ElevationFrame) => void;
  reset: () => void;
};

export const useElevationMap = create<ElevationMapStore>((set) => ({
  frame: undefined,
  apply: (f) => set({ frame: f }),
  reset: () => set({ frame: undefined }),
}));

export function handleElevationFrame(data: ArrayBuffer) {
  if (data.byteLength <= HEADER_BYTES) return;
  const dv = new DataView(data);
  if (dv.getUint16(0, true) !== ELEVATION_MAGIC) return;
  const rows = dv.getInt32(2, true);
  const cols = dv.getInt32(6, true);
  const gs = dv.getFloat32(10, true);
  const originX = dv.getFloat32(14, true);
  const originY = dv.getFloat32(18, true);
  const n = rows * cols;
  if (rows <= 0 || cols <= 0 || n <= 0) return;
  const heightBytes = n * 4;
  if (data.byteLength < HEADER_BYTES + heightBytes + n) return;
  const height = new Float32Array(n);
  for (let i = 0; i < n; i++) {
    height[i] = dv.getFloat32(HEADER_BYTES + i * 4, true);
  }
  const valid = new Uint8Array(data, HEADER_BYTES + heightBytes, n);
  const robotZ = useTelemetry.getState().robot?.worldPos?.[2] ?? 0;
  useElevationMap.getState().apply({ rows, cols, gs, originX, originY, height, valid, at: Date.now(), robotZ });
}

export function attachElevationChannel(pc: any) {
  try {
    const dc = pc.createDataChannel('elevation-map');
    dc.binaryType = 'arraybuffer';
    dc.onmessage = (e: any) => {
      const d = e?.data;
      if (d instanceof ArrayBuffer) handleElevationFrame(d);
    };
  } catch {}
}
