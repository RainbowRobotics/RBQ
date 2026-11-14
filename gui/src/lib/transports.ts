import { isDemo } from '@/lib/demoFlag';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useFeatures } from '@/store/capability';
import { useViewport } from '@/store/viewport';
import { useDockView } from '@/store/dockView';
import { connection } from './connection';
import { webrtcClient, useWebrtcStore } from './webrtcClient';
import { resolveEndpoints } from './resolveEndpoints';
import { robotTlsPlatform } from './robotTlsPlatform';
import { useSettings } from '@/store/settings';

(globalThis as any).__rbqStores = { robot: useRobot, telemetry: useTelemetry, features: useFeatures, video: useWebrtcStore, viewport: useViewport, dockView: useDockView };

export function connectAll(ip: string, visionIp?: string) {
  robotTlsPlatform.allowPlain();
  connection.connect(ip);
  if (isDemo()) return;
  const vision = resolveEndpoints(ip, visionIp).vision;
  webrtcClient.connect(vision);
}

export function detachAllForSim(): boolean {
  if (isDemo()) return false;
  if (connection.detachForSim() === null) return false;
  webrtcClient.disconnect();
  return true;
}

export function resumeCurrent() {
  robotTlsPlatform.allowPlain();
  connection.resecure();
}

export function disconnectAll() {
  robotTlsPlatform.forgetPlain();
  connection.clearInputs();
  connection.disconnect();
  webrtcClient.disconnect();
}

let _lastObsAvoidEnabled = useSettings.getState().obsAvoidEnabled;
useSettings.subscribe((s) => {
  if (s.obsAvoidEnabled === _lastObsAvoidEnabled) return;
  _lastObsAvoidEnabled = s.obsAvoidEnabled;
  if (!s.obsAvoidEnabled) return;
  if (useRobot.getState().conn !== 'connected') return;
  const { ip, visionIp } = useRobot.getState();
  webrtcClient.ensureConnected(resolveEndpoints(ip, visionIp).vision);
});

let _lastConn = useRobot.getState().conn;
let _obsAvoidSyncedThisConn = false;
useRobot.subscribe((s) => {
  if (s.conn === _lastConn) return;
  _lastConn = s.conn;
  if (s.conn !== 'connected') _obsAvoidSyncedThisConn = false;
});
useTelemetry.subscribe((s) => {
  if (_obsAvoidSyncedThisConn || !s.robot) return;
  _obsAvoidSyncedThisConn = true;
  if (useSettings.getState().obsAvoidEnabled !== s.robot.obsAvoidEnabled) {
    useSettings.getState().setObsAvoidEnabled(s.robot.obsAvoidEnabled);
  }
});
