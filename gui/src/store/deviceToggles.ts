import { create } from 'zustand';
import { bindRobotCache } from '@/store/robotSettings';

type DeviceToggles = {
  cctvZoom: number;
  ptzZoom: number;
  ptzFaceTemp: boolean;
  slamFollow: boolean;
  slamView: '2d' | '3d';
  doorHandleType: number;
  doorHingeSide: number;
  doorOpenType: number;
};

export const useDeviceToggles = create<DeviceToggles>(() => ({
  cctvZoom: 1, ptzZoom: 1, ptzFaceTemp: false, slamFollow: false, slamView: '3d',
  doorHandleType: 0, doorHingeSide: 0, doorOpenType: 0,
}));

bindRobotCache(useDeviceToggles, [
  'cctvZoom', 'ptzZoom', 'slamFollow', 'slamView',
  'doorHandleType', 'doorHingeSide', 'doorOpenType',
], 'device');
