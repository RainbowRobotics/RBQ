import { create } from 'zustand';
import { DRACO_SUPPORT, decodeDracoPointCloud, type DecodedCloud } from './dracoDecode';
import { hmChannelOfSensorId, toBodyFrame, useHeightmapCloud } from './heightmapCloud';

export const POINTCLOUD_MAGIC = 0x5052;
const HEADER_BYTES = 2 + 2 + 4 + 64;
const LIDAR_SENSOR_IDS = [11, 12] as const;

export type CloudFrame = DecodedCloud & { at: number };

type PointCloudStore = {
  clouds: Partial<Record<number, CloudFrame>>;
  lastAt?: number;
  apply: (sensorId: number, frame: CloudFrame) => void;
  reset: () => void;
};

export const usePointCloud = create<PointCloudStore>((set) => ({
  clouds: {},
  apply: (sensorId, frame) =>
    set((s) => ({ clouds: { ...s.clouds, [sensorId]: frame }, lastAt: frame.at })),
  reset: () => set({ clouds: {}, lastAt: undefined }),
}));

export function resetAllClouds(): void {
  usePointCloud.getState().reset();
  useHeightmapCloud.getState().reset();
}

const decodeBusy: Partial<Record<number, boolean>> = {};

export function handlePointcloudFrame(data: ArrayBuffer) {
  if (data.byteLength < HEADER_BYTES) return;
  const dv = new DataView(data);
  if (dv.getUint16(0, true) !== POINTCLOUD_MAGIC) return;
  const sensorId = dv.getUint16(2, true);
  const hm = hmChannelOfSensorId(sensorId);
  if (hm === null && !(LIDAR_SENSOR_IDS as readonly number[]).includes(sensorId)) return;
  if (dv.getUint32(4, true) === 0) {
    if (hm !== null) useHeightmapCloud.getState().apply(hm, { positions: new Float32Array(0), w: null, count: 0, at: Date.now() });
    else usePointCloud.getState().apply(sensorId, { positions: new Float32Array(0), reflectivity: null, count: 0, at: Date.now() });
    return;
  }
  if (data.byteLength <= HEADER_BYTES) return;
  if (decodeBusy[sensorId]) return;
  decodeBusy[sensorId] = true;
  const tf = hm !== null ? new Float32Array(data.slice(8, HEADER_BYTES)) : null;
  const bytes = new Uint8Array(data, HEADER_BYTES);
  decodeDracoPointCloud(bytes)
    .then((d) => {
      if (!d) return;
      const at = Date.now();
      if (hm !== null) {
        useHeightmapCloud.getState().apply(hm, { positions: toBodyFrame(tf!, d.positions, 3, d.count), w: d.reflectivity, count: d.count, at });
      } else {
        usePointCloud.getState().apply(sensorId, { ...d, at });
      }
    })
    .catch(() => {})
    .finally(() => { decodeBusy[sensorId] = false; });
}

export function attachPointcloudChannel(pc: any) {
  if (!DRACO_SUPPORT.available) return;
  try {
    const dc = pc.createDataChannel('pointcloud');
    dc.binaryType = 'arraybuffer';
    dc.onmessage = (e: any) => {
      const d = e?.data;
      if (d instanceof ArrayBuffer) handlePointcloudFrame(d);
    };
  } catch {}
}
