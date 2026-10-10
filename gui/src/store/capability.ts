import { create } from 'zustand';
import type { RobotState } from '@/lib/robotState';
import type { SensorState } from '@/lib/sensorState';
import type { RobotFeatures } from '@/lib/rest';
import type { DeviceStatus } from '@/lib/robotState';
import { useTelemetry } from './telemetry';

const DEV = { LIDAR: 14, PTZ: 15, CCTV: 16, THERMAL: 17 } as const;

export type Capabilities = {
  hasArm: boolean;
  hasCctv: boolean;
  hasThermal: boolean;
  hasPtz: boolean;
  hasCamera: boolean;
  hasIr: boolean;
  hasDepth: boolean;
  hasProjector: boolean;
  online: boolean;
  visionOnline: boolean;
};

const NONE: Capabilities = {
  hasArm: false, hasCctv: false, hasThermal: false, hasPtz: false,
  hasCamera: false, hasIr: false, hasDepth: false, hasProjector: false,
  online: false, visionOnline: false,
};

export function deriveCapabilities(
  robot?: RobotState, sensors?: SensorState[], devices?: DeviceStatus[],
): Capabilities {
  const a = robot?.attached;
  return {
    hasArm: !!a?.arm,
    hasCctv: !!devices?.[DEV.CCTV]?.connected,
    hasThermal: !!devices?.[DEV.THERMAL]?.connected,
    hasPtz: !!devices?.[DEV.PTZ]?.connected,
    hasCamera: !!sensors?.some((s) => s.rgb),
    hasIr: !!sensors?.some((s) => s.ir),
    hasDepth: !!sensors?.some((s) => s.depth),
    hasProjector: !!sensors?.some((s) => s.projector),
    online: !!robot,
    visionOnline: !!sensors,
  };
}

export const useHasArm = () => useTelemetry((s) => !!s.robot?.attached.arm);
export const useHasCctv = () => useTelemetry((s) => !!s.devices?.[DEV.CCTV]?.connected);
export const useHasThermal = () => useTelemetry((s) => !!s.devices?.[DEV.THERMAL]?.connected);
export const useHasPtz = () => useTelemetry((s) => !!s.devices?.[DEV.PTZ]?.connected);
export const useHasLidar = () => useTelemetry((s) => !!s.devices?.[DEV.LIDAR]?.connected);
export const useTelemetryOnline = () => useTelemetry((s) => !!s.robot);
export const useHasCamera = () => useTelemetry((s) => !!s.sensors?.some((x) => x.rgb));
export const useHasIr = () => useTelemetry((s) => !!s.sensors?.some((x) => x.ir));
export const useHasDepth = () => useTelemetry((s) => !!s.sensors?.some((x) => x.depth));
export const useHasProjector = () => useTelemetry((s) => !!s.sensors?.some((x) => x.projector));
export const useVisionOnline = () => useTelemetry((s) => !!s.sensors);

type FeatureStore = {
  features: RobotFeatures | null;
  setFeatures: (f: RobotFeatures) => void;
  clearFeatures: () => void;
};
export const useFeatures = create<FeatureStore>((set) => ({
  features: null,
  setFeatures: (f) => set({ features: f }),
  clearFeatures: () => set({ features: null }),
}));

export const useFeatureWheel = () => useFeatures((s) => !!s.features?.wheel);

export const useFeatureCanFd = () => useFeatures((s) => !!s.features?.can_fd);

export const useFeatureFwUpdate = () => useFeatures((s) => !!s.features?.fw_update);
