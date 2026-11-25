import { create } from 'zustand';
import { useTelemetry } from '@/store/telemetry';

export const ELEVATION_MAGIC = 0x454d;
export const ELEVATION_Z_MAGIC = 0x5a45;
export const ELEVATION_CODEC_HELLO = 'ez1';
const HEADER_BYTES = 2 + 4 + 4 + 4 + 4 + 4;
const Z_HEADER_BYTES = 2 + 1 + 4 * 6;
const Z_RICE_LIMIT = 20;
const Z_MAX_CELLS = 300 * 300;

type ElevationGrid = Pick<ElevationFrame, 'rows' | 'cols' | 'gs' | 'originX' | 'originY' | 'height' | 'valid'>;

export function decodeElevationZ(data: ArrayBuffer): ElevationGrid | null {
  if (data.byteLength < Z_HEADER_BYTES) return null;
  const dv = new DataView(data);
  if (dv.getUint16(0, true) !== ELEVATION_Z_MAGIC || dv.getUint8(2) !== 1) return null;
  const rows = dv.getInt32(3, true);
  const cols = dv.getInt32(7, true);
  const gs = dv.getFloat32(11, true);
  const originX = dv.getFloat32(15, true);
  const originY = dv.getFloat32(19, true);
  const q = dv.getFloat32(23, true);
  if (rows <= 0 || cols <= 0 || rows * cols > Z_MAX_CELLS || !(q > 0)) return null;
  const n = rows * cols;
  const bytes = new Uint8Array(data, Z_HEADER_BYTES);
  let pos = 0;
  let bad = false;
  const bit = () => {
    const b = pos >> 3;
    if (b >= bytes.length) { bad = true; return 0; }
    const v = (bytes[b] >> (7 - (pos & 7))) & 1;
    pos++;
    return v;
  };
  const bitsOf = (k: number) => { let v = 0; for (let i = 0; i < k; i++) v = v * 2 + bit(); return v; };
  const rice = () => {
    const st = { A: 4, N: 1 };
    return () => {
      let k = 0;
      while (st.N * 2 ** k < st.A && k < 24) k++;
      let q2 = 0;
      while (q2 < Z_RICE_LIMIT && bit()) q2++;
      const v = q2 === Z_RICE_LIMIT ? bitsOf(32) : q2 * 2 ** k + bitsOf(k);
      st.A += v;
      if (++st.N >= 64 || st.A >= 2 ** 30) { st.A = Math.floor((st.A + 1) / 2); st.N = Math.floor((st.N + 1) / 2); }
      return v;
    };
  };
  const runs = rice();
  const res = rice();
  const ok = new Uint8Array(n);
  let cur = 0;
  for (let i = 0; i < n; ) {
    const len = runs();
    if (bad || len > n - i) return null;
    if (cur) ok.fill(1, i, i + len);
    i += len;
    cur ^= 1;
  }
  const v = new Float64Array(n);
  let last = 0;
  for (let i = 0; i < n; i++) {
    if (!ok[i]) continue;
    const c = i % cols;
    const hasW = c > 0 && ok[i - 1] === 1;
    const hasN = i >= cols && ok[i - cols] === 1;
    let pred: number;
    if (hasW && hasN) {
      const w = v[i - 1], nn = v[i - cols];
      if (ok[i - cols - 1] !== 1) pred = Math.floor((w + nn) / 2);
      else {
        const nw = v[i - cols - 1];
        const mx = Math.max(w, nn), mn = Math.min(w, nn);
        pred = nw >= mx ? mn : nw <= mn ? mx : w + nn - nw;
      }
    } else if (hasW) pred = v[i - 1];
    else if (hasN) pred = v[i - cols];
    else pred = last;
    const u = res();
    if (bad) return null;
    v[i] = pred + (u % 2 ? -(u - 1) / 2 - 1 : u / 2);
    last = v[i];
  }
  const height = new Float32Array(n);
  for (let i = 0; i < n; i++) if (ok[i]) height[i] = v[i] * q;
  return { rows, cols, gs, originX, originY, height, valid: ok };
}

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
  if (data.byteLength >= 2 && new DataView(data).getUint16(0, true) === ELEVATION_Z_MAGIC) {
    const g = decodeElevationZ(data);
    if (!g) return;
    const robotZ = useTelemetry.getState().robot?.worldPos?.[2] ?? 0;
    useElevationMap.getState().apply({ ...g, at: Date.now(), robotZ });
    return;
  }
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
    dc.onopen = () => {
      try { dc.send(ELEVATION_CODEC_HELLO); } catch {}
    };
    dc.onmessage = (e: any) => {
      const d = e?.data;
      if (d instanceof ArrayBuffer) handleElevationFrame(d);
    };
  } catch {}
}
