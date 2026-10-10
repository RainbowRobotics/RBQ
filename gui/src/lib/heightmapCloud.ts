import { create } from 'zustand';

export enum HmChannel { Map = 0, Edge = 1, Stair = 2, StairEdge = 3, FootQuery = 4, FootAnswer = 5 }
export const HM_CHANNEL_COUNT = 6;

export type HmLayers = { grid: boolean; stair: boolean; edge: boolean; foot: boolean };
export const HM_LAYERS_OFF: HmLayers = { grid: false, stair: false, edge: false, foot: false };

export const GAIT_TROT_STAIRS = 4;
export const GAIT_RL_WALK_VISION = 48;
export function isHeightmapGait(gaitId: number | undefined): boolean {
  return gaitId === GAIT_TROT_STAIRS || gaitId === GAIT_RL_WALK_VISION;
}
export const HM_LAYERS_ALL: HmLayers = { grid: true, stair: true, edge: true, foot: true };

export type HmFrame = {
  positions: Float32Array;
  w: Float32Array | null;
  count: number;
  at: number;
};

export function hmChannelOfSensorId(id: number): HmChannel | null {
  return id >= 0x80 && id <= 0x85 ? ((id - 0x80) as HmChannel) : null;
}

export function toBodyFrame(tf: ArrayLike<number>, src: ArrayLike<number>, stride: number, n: number): Float32Array {
  const out = new Float32Array(n * 3);
  if (tf[0] === 0 && tf[5] === 0 && tf[10] === 0 && tf[15] === 0) {
    for (let i = 0; i < n; i++) { out[i * 3] = src[i * stride]; out[i * 3 + 1] = src[i * stride + 1]; out[i * 3 + 2] = src[i * stride + 2]; }
    return out;
  }
  const r0 = tf[0], r1 = tf[4], r2 = tf[8],
        r3 = tf[1], r4 = tf[5], r5 = tf[9],
        r6 = tf[2], r7 = tf[6], r8 = tf[10];
  const tx = tf[3], ty = tf[7], tz = tf[11];
  const ox = -(r0 * tx + r1 * ty + r2 * tz);
  const oy = -(r3 * tx + r4 * ty + r5 * tz);
  const oz = -(r6 * tx + r7 * ty + r8 * tz);
  for (let i = 0; i < n; i++) {
    const wx = src[i * stride], wy = src[i * stride + 1], wz = src[i * stride + 2];
    out[i * 3]     = r0 * wx + r1 * wy + r2 * wz + ox;
    out[i * 3 + 1] = r3 * wx + r4 * wy + r5 * wz + oy;
    out[i * 3 + 2] = r6 * wx + r7 * wy + r8 * wz + oz;
  }
  return out;
}

type HmStore = {
  clouds: Partial<Record<HmChannel, HmFrame>>;
  lastAt?: number;
  apply: (ch: HmChannel, frame: HmFrame) => void;
  reset: () => void;
};
export const useHeightmapCloud = create<HmStore>((set) => ({
  clouds: {},
  apply: (ch, frame) => set((s) => ({ clouds: { ...s.clouds, [ch]: frame }, lastAt: frame.at })),
  reset: () => set({ clouds: {}, lastAt: undefined }),
}));

export const PCD_VERSION = 1;
const PCD_FILE_HDR = 32;
const PCD_FRAME_HDR = 80;

export type PcdFrame = { ch: HmChannel; msFromStart: number; frame: HmFrame };

function i64(dv: DataView, off: number): number {
  return dv.getInt32(off + 4, true) * 4294967296 + dv.getUint32(off, true);
}

export function parsePcdBin(buf: ArrayBuffer, ref: { frameZeroEpochMs: number | null; durMs: number }): PcdFrame[] {
  const out: PcdFrame[] = [];
  if (buf.byteLength < PCD_FILE_HDR) return out;
  const dv = new DataView(buf);
  const magic = String.fromCharCode(dv.getUint8(0), dv.getUint8(1), dv.getUint8(2), dv.getUint8(3));
  if (magic !== 'RPCD') { console.warn('[heightmapCloud] pcd.bin: bad magic'); return out; }
  const version = dv.getUint16(4, true);
  if (version !== PCD_VERSION) { console.warn(`[heightmapCloud] pcd.bin: version ${version} != ${PCD_VERSION} — skipped`); return out; }
  const endMs = i64(dv, 24);
  let pos = Math.max(PCD_FILE_HDR, dv.getUint16(6, true));

  while (pos + PCD_FRAME_HDR <= buf.byteLength) {
    const kind = dv.getUint8(pos);
    const stride = dv.getUint8(pos + 1);
    const n = dv.getUint32(pos + 4, true);
    const epochMs = i64(dv, pos + 8);
    if (stride !== 3 && stride !== 4) { console.warn(`[heightmapCloud] pcd.bin: bad stride ${stride}`); break; }
    const payload = n * stride * 4;
    const dataOff = pos + PCD_FRAME_HDR;
    if (payload > buf.byteLength - dataOff) { console.warn('[heightmapCloud] pcd.bin: truncated frame'); break; }
    if (kind < HM_CHANNEL_COUNT) {
      const tf = new Float32Array(buf.slice(pos + 16, pos + PCD_FRAME_HDR));
      const src = new Float32Array(buf.slice(dataOff, dataOff + payload));
      let w: Float32Array | null = null;
      if (stride === 4) { w = new Float32Array(n); for (let i = 0; i < n; i++) w[i] = src[i * 4 + 3]; }
      const msFromStart = ref.frameZeroEpochMs != null ? epochMs - ref.frameZeroEpochMs : ref.durMs - (endMs - epochMs);
      out.push({ ch: kind as HmChannel, msFromStart, frame: { positions: toBodyFrame(tf, src, stride, n), w, count: n, at: msFromStart } });
    }
    pos = dataOff + payload;
  }
  out.sort((a, b) => a.msFromStart - b.msFromStart);
  return out;
}

export function pickPcdAt(frames: readonly PcdFrame[], nowMs: number): Partial<Record<HmChannel, HmFrame>> {
  const out: Partial<Record<HmChannel, HmFrame>> = {};
  for (const f of frames) { if (f.msFromStart > nowMs) break; out[f.ch] = f.frame; }
  return out;
}
