import { create } from 'zustand';
import { getDemoBaseRobot } from '@/lib/demoFlag';
import { setAuthFailureHandler } from '@/lib/auth';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';
import type {
  RobotStatus, PcStatus, Trip, AggStatus, ConnectionState, GaitName, LogLine,
} from '@/types/robot';

const MAX_LOGS = 2000;
const MAX_ALERTS = 200;

const _logs: LogLine[] = [];
const _alerts: LogLine[] = [];
export const getLiveLogs = (): readonly LogLine[] => _logs;
export const getLiveAlerts = (): readonly LogLine[] => _alerts;

function computeAgg(r?: RobotStatus): AggStatus {
  if (!r) return 'amber';
  return r.imu && r.can_bus && r.find_pose && r.control_started ? 'green' : 'amber';
}

type RobotStore = {
  conn: ConnectionState;
  route: import('@/lib/connectionRoute').Route;
  via: 'direct' | 'rendezvous';
  relayProto: import('@/lib/connectionRoute').RelayProto;
  ip: string;
  visionIp: string;
  robot?: RobotStatus;
  pc?: PcStatus;
  trip?: Trip;
  logSeq: number;
  alertSeq: number;
  lastError: LogLine | null;
  owner: string;
  myIp: string;
  isMine: boolean;
  agg: AggStatus;
  gait: GaitName;
  battPct: number;
  battV: number;

  setIp: (ip: string) => void;
  setVisionIp: (v: string) => void;
  droppedAt?: number;
  setConn: (c: ConnectionState) => void;
  setRoute: (r: import('@/lib/connectionRoute').Route, p?: import('@/lib/connectionRoute').RelayProto) => void;
  setVia: (v: 'direct' | 'rendezvous') => void;
  connError: string | null;
  setConnError: (e: string | null) => void;
  takeoverConflict: boolean;
  authNeeded: boolean;
  setAuthNeeded: (v: boolean) => void;
  authEpoch: number;
  bumpAuthEpoch: () => void;
  setTakeoverConflict: (v: boolean) => void;
  applyRobot: (r: RobotStatus) => void;
  applyPc: (p: PcStatus) => void;
  applyTrip: (t: Trip) => void;
  setOwner: (owner: string) => void;
  setOwnership: (o: { owner: string; myIp: string; isMine: boolean }) => void;
  pushLog: (l: LogLine) => void;
};

export const useRobot = create<RobotStore>()(
  persist(
    (set, get) => ({
  conn: 'disconnected',
  route: 'unknown',
  via: 'direct',
  relayProto: null,
  ip: '192.168.0.10',
  visionIp: '',
  logSeq: 0,
  alertSeq: 0,
  lastError: null,
  owner: '',
  myIp: '',
  isMine: false,
  agg: 'amber',
  gait: 'STANDING',
  battPct: 0,
  battV: 0,

  setIp: (ip) => set({ ip }),
  setVisionIp: (visionIp) => set({ visionIp }),
  setConn: (conn) => set((s) => ({ conn, droppedAt: s.conn === 'connected' && conn !== 'connected' ? Date.now() : s.droppedAt })),
  setRoute: (route, relayProto = null) => set({ route, relayProto }),
  setVia: (via) => set({ via }),
  connError: null,
  setConnError: (connError) => set({ connError }),
  takeoverConflict: false,
  authNeeded: false,
  setAuthNeeded: (authNeeded) => set({ authNeeded }),
  authEpoch: 0,
  bumpAuthEpoch: () => set((s) => ({ authEpoch: s.authEpoch + 1 })),
  setTakeoverConflict: (takeoverConflict) => set({ takeoverConflict }),
  applyRobot: (robot) =>
    set({
      robot,
      agg: computeAgg(robot),
      gait: robot.gait_name,
      battPct: robot.battery_pct,
      battV: robot.battery_voltage,
    }),
  applyPc: (pc) => set({ pc }),
  applyTrip: (trip) => set({ trip }),
  setOwner: (owner) => {
    const { myIp, owner: prevOwner, isMine: prevMine } = get();
    const isMine = !!owner && owner === myIp;
    if (owner === prevOwner && isMine === prevMine) return;
    set({ owner, isMine });
  },
  setOwnership: ({ owner, myIp, isMine }) => {
    const s = get();
    if (owner === s.owner && myIp === s.myIp && isMine === s.isMine) return;
    set({ owner, myIp, isMine });
  },
  pushLog: (l) => {
    const line: LogLine = { ...l, rxMs: l.rxMs ?? Date.now() };
    _logs.push(line);
    if (_logs.length > MAX_LOGS) _logs.splice(0, _logs.length - MAX_LOGS);
    const isAlert = line.level === 'WARNING' || line.level === 'ERROR' || line.level === 'FATAL';
    if (isAlert) {
      _alerts.push(line);
      if (_alerts.length > MAX_ALERTS) _alerts.splice(0, _alerts.length - MAX_ALERTS);
    }
    const isRed = line.level === 'ERROR' || line.level === 'FATAL';
    set((s) => ({
      logSeq: s.logSeq + 1,
      ...(isAlert ? { alertSeq: s.alertSeq + 1 } : null),
      ...(isRed ? { lastError: line } : null),
    }));
  },
    }),
    {
      name: 'rbq-robot',
      storage: createJSONStorage(() => AsyncStorage),
      partialize: (s) => getDemoBaseRobot() ?? { ip: s.ip, visionIp: s.visionIp },
    },
  ),
);

setAuthFailureHandler(() => useRobot.getState().setAuthNeeded(true));
