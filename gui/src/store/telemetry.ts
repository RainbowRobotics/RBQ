import { create } from 'zustand';
import type { RobotState, DeviceStatus, PduState } from '@/lib/robotState';
import { nextChargeAvg, type ChargeAvg } from '@/lib/robotState';
import { packInAmps } from '@/lib/charge';
import type { SensorState, SensorLayout } from '@/lib/sensorState';
import type { VisionProgramState } from '@/lib/visionProgramStates';
import type { MarkerState } from '@/lib/arucoState';
import type { P2gState } from '@/lib/p2gState';
import { motionFrames } from '@/modules/registry';

export type LinkConn = 'disconnected' | 'connecting' | 'connected';

type TelemetryStore = {
  motionConn: LinkConn;
  robot?: RobotState;
  devices?: DeviceStatus[];
  pdu?: PduState;
  chargeAvg?: ChargeAvg;
  packAvg?: ChargeAvg;
  plugSince?: number | null;
  lastRobotAt?: number;
  lastDeviceAt?: number;
  visionConn: LinkConn;
  sensors?: SensorState[];
  lastSensorAt?: number;
  visionPrograms?: VisionProgramState[];
  visionProgramWire?: number;
  sensorLayout: SensorLayout;
  doorHandlePose?: [number, number, number, number, number, number];
  doorPose?: [number, number, number, number, number, number];
  markerState?: MarkerState;
  p2gState?: P2gState;

  setMotionConn: (c: LinkConn) => void;
  setSensorLayout: (l: SensorLayout) => void;
  applyRobotState: (r: RobotState) => void;
  applyDeviceStates: (d: DeviceStatus[]) => void;
  applyPduState: (p: PduState) => void;
  setVisionConn: (c: LinkConn) => void;
  applySensorStates: (s: SensorState[]) => void;
  applyVisionPrograms: (p: VisionProgramState[]) => void;
  noteVisionProgramWire: (size: number) => void;
  applyDoorHandlePose: (p: [number, number, number, number, number, number]) => void;
  applyDoorPose: (p: [number, number, number, number, number, number]) => void;
  applyMarkerState: (m: MarkerState) => void;
  applyP2gState: (p: P2gState) => void;
  reset: () => void;
  clearMotionData: () => void;
  clearVisionData: () => void;
};

export const useTelemetry = create<TelemetryStore>((set) => ({
  motionConn: 'disconnected',
  visionConn: 'disconnected',
  sensorLayout: 'current',

  setMotionConn: (motionConn) => set({ motionConn }),
  setSensorLayout: (sensorLayout) => set({ sensorLayout }),
  applyRobotState: (robot) => set({ robot, lastRobotAt: Date.now() }),
  applyDeviceStates: (devices) => set({ devices, lastDeviceAt: Date.now() }),
  applyPduState: (pdu) => set((st) => {
    const now = Date.now();
    const plugged = pdu.chgS || pdu.chgE;
    return { pdu, chargeAvg: nextChargeAvg(st.chargeAvg ?? null, pdu.rails.chg.a, now),
      packAvg: nextChargeAvg(st.packAvg ?? null, packInAmps(pdu), now), plugSince: plugged ? (st.plugSince ?? now) : null };
  }),
  setVisionConn: (visionConn) => set({ visionConn }),
  applySensorStates: (sensors) => set({ sensors, lastSensorAt: Date.now() }),
  applyVisionPrograms: (visionPrograms) => set({ visionPrograms }),
  noteVisionProgramWire: (visionProgramWire) => set({ visionProgramWire }),
  applyDoorHandlePose: (doorHandlePose) => set({ doorHandlePose }),
  applyDoorPose: (doorPose) => set({ doorPose }),
  applyMarkerState: (markerState) => set({ markerState }),
  applyP2gState: (p2gState) => set({ p2gState }),
  clearMotionData: () => {
    motionFrames.forEach((h) => h.clear?.());
    set({ robot: undefined, devices: undefined, pdu: undefined,
          lastRobotAt: undefined, lastDeviceAt: undefined });
  },
  clearVisionData: () =>
    set({ sensors: undefined, lastSensorAt: undefined, visionPrograms: undefined, visionProgramWire: undefined,
          doorHandlePose: undefined, doorPose: undefined, markerState: undefined, p2gState: undefined }),
  reset: () => {
    motionFrames.forEach((h) => h.clear?.());
    set({
      motionConn: 'disconnected', robot: undefined, devices: undefined, pdu: undefined, chargeAvg: undefined, packAvg: undefined, plugSince: null, lastRobotAt: undefined, lastDeviceAt: undefined,
      visionConn: 'disconnected', sensors: undefined, lastSensorAt: undefined, visionPrograms: undefined, visionProgramWire: undefined,
      doorHandlePose: undefined, doorPose: undefined, markerState: undefined, p2gState: undefined,
    });
  },
}));

export const STALE_CLEAR_MS = 6000;

let motionStale: ReturnType<typeof setTimeout> | null = null;
let visionStale: ReturnType<typeof setTimeout> | null = null;

export function noteMotionLink(c: LinkConn) {
  const s = useTelemetry.getState();
  if (s.motionConn !== c) s.setMotionConn(c);
  if (c === 'connected') {
    if (motionStale) { clearTimeout(motionStale); motionStale = null; }
  } else if (!motionStale) {
    motionStale = setTimeout(() => { motionStale = null; useTelemetry.getState().clearMotionData(); }, STALE_CLEAR_MS);
  }
}

export function noteVisionLink(c: LinkConn) {
  const s = useTelemetry.getState();
  if (s.visionConn !== c) s.setVisionConn(c);
  if (c === 'connected') {
    if (visionStale) { clearTimeout(visionStale); visionStale = null; }
  } else if (!visionStale) {
    visionStale = setTimeout(() => { visionStale = null; useTelemetry.getState().clearVisionData(); }, STALE_CLEAR_MS);
  }
}
