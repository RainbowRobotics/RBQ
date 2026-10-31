import { create } from 'zustand';

export const ID_CAMERA_CALIB = 33;
const N_CAMS = 6;
export const CAMERA_CALIB_SIZE = 4 + 4 + N_CAMS * 4 * 4 + N_CAMS * 8 * 4 + N_CAMS * 16 * 4 + N_CAMS * 4 + N_CAMS * 4;

export type CamCalib = {
  fx: number; fy: number; cx: number; cy: number;
  coeffs: number[];
  tf: number[];
  width: number;
  height: number;
};

export type CameraCalibSet = {
  front: CamCalib; rear: CamCalib;
  bottom0: CamCalib; bottom1: CamCalib; bottom2: CamCalib; bottom3: CamCalib;
};

type CameraCalibStore = Partial<CameraCalibSet> & {
  apply: (cams: CameraCalibSet) => void;
};

export const useCameraCalib = create<CameraCalibStore>((set) => ({
  apply: (cams) => set(cams),
}));

export function parseCameraCalib(data: ArrayBuffer, off: number): CameraCalibSet {
  const dv = new DataView(data);
  const intrinsicsOff = off + 8;
  const coeffsOff = intrinsicsOff + N_CAMS * 4 * 4;
  const tfOff = coeffsOff + N_CAMS * 8 * 4;
  const widthOff = tfOff + N_CAMS * 16 * 4;
  const heightOff = widthOff + N_CAMS * 4;

  const readCam = (camIdx: number): CamCalib => {
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

  return {
    front: readCam(0), rear: readCam(1),
    bottom0: readCam(2), bottom1: readCam(3), bottom2: readCam(4), bottom3: readCam(5),
  };
}

export function handleCameraCalibFrame(data: ArrayBuffer, off: number) {
  useCameraCalib.getState().apply(parseCameraCalib(data, off));
}
