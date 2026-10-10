import { create } from 'zustand';

export const ID_CAMERA_CALIB = 33;
export const CAMERA_CALIB_SIZE = 4 + 4 + 2 * 4 * 4 + 2 * 8 * 4 + 2 * 16 * 4 + 2 * 4 + 2 * 4;

export type CamCalib = {
  fx: number; fy: number; cx: number; cy: number;
  coeffs: number[];
  tf: number[];
  width: number;
  height: number;
};

type CameraCalibStore = {
  front?: CamCalib;
  rear?: CamCalib;
  apply: (front: CamCalib, rear: CamCalib) => void;
};

export const useCameraCalib = create<CameraCalibStore>((set) => ({
  apply: (front, rear) => set({ front, rear }),
}));

export function parseCameraCalib(data: ArrayBuffer, off: number): { front: CamCalib; rear: CamCalib } {
  const dv = new DataView(data);
  const intrinsicsOff = off + 8;
  const coeffsOff = intrinsicsOff + 2 * 4 * 4;
  const tfOff = coeffsOff + 2 * 8 * 4;
  const widthOff = tfOff + 2 * 16 * 4;
  const heightOff = widthOff + 2 * 4;

  const readCam = (camIdx: 0 | 1): CamCalib => {
    const io = intrinsicsOff + camIdx * 4 * 4;
    const fx = dv.getFloat32(io, true);
    const fy = dv.getFloat32(io + 4, true);
    const cx = dv.getFloat32(io + 8, true);
    const cy = dv.getFloat32(io + 12, true);
    const co = coeffsOff + camIdx * 8 * 4;
    const coeffs = Array.from({ length: 8 }, (_, i) => dv.getFloat32(co + i * 4, true));
    const to = tfOff + camIdx * 16 * 4;
    const tf = Array.from({ length: 16 }, (_, i) => dv.getFloat32(to + i * 4, true));
    const width = dv.getInt32(widthOff + camIdx * 4, true);
    const height = dv.getInt32(heightOff + camIdx * 4, true);
    return { fx, fy, cx, cy, coeffs, tf, width, height };
  };

  return { front: readCam(0), rear: readCam(1) };
}

export function handleCameraCalibFrame(data: ArrayBuffer, off: number) {
  const { front, rear } = parseCameraCalib(data, off);
  useCameraCalib.getState().apply(front, rear);
}
