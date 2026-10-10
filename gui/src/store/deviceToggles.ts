import { create } from 'zustand';
import { bindRobotCache } from '@/store/robotSettings';

type DeviceToggles = {
  cctvZoom: number;
  ptzZoom: number;
  ptzFaceTemp: boolean;
  slamFollow: boolean;
  doorHandleType: number;
  doorHingeSide: number;
  doorOpenType: number;
};

export const useDeviceToggles = create<DeviceToggles>(() => ({
  cctvZoom: 1, ptzZoom: 1, ptzFaceTemp: false, slamFollow: false,
  doorHandleType: 0, doorHingeSide: 0, doorOpenType: 0,
}));

bindRobotCache(useDeviceToggles, [
  'cctvZoom', 'ptzZoom', 'slamFollow',
  'doorHandleType', 'doorHingeSide', 'doorOpenType',
], 'device');
