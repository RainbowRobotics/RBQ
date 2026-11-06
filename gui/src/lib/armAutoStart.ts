import type { RobotState, PduState } from './robotState';
import { PROGRAM } from './robotState';
import { rest } from './rest';

export type ArmStartContext = {
  accessLevel: number; connected: boolean; switching: boolean; owner: string; isMine: boolean;
  robot?: RobotState; pdu?:PduState; lastRobotAt?: number; legStarting: boolean;
  robotKey: string; isSim: boolean; lastRobotStatusAt?: number; droppedAt?: number;
  acknowledgedRunning?: boolean;
};

export const ARM_ALREADY_STARTED = '팔 제어가 이미 시작돼 있습니다.';

export function armStartBlockReason(s: ArmStartContext, now = Date.now(), expectedKey = s.robotKey): string | null {
  if (!(s.accessLevel >= 2)) return '개발자 모드에서만 팔 자동 기동을 실행할 수 있습니다.';
  if (!s.connected || s.switching || s.isSim || !s.robotKey || s.robotKey !== expectedKey) return '로봇 연결을 확인하세요.';
  const fresh = (at?: number) => Number.isFinite(at) && at! > (s.droppedAt ?? 0)
    && now >= at! && now - at! <= 3000;
  if (!s.robot || !fresh(s.lastRobotAt) || !fresh(s.lastRobotStatusAt)) return '최신 로봇 상태를 기다리고 있습니다.';
  if (!s.robot.attached.arm) return '로봇팔이 인식되지 않았습니다.';
  if (s.owner && !s.isMine) return '로봇 제어권을 먼저 가져오세요.';
  if (s.legStarting) return '다리 자동 기동이 끝난 뒤 실행하세요.';
  if (s.robot.isFall || ![-1, 0, 1].includes(s.robot.gaitId)) return '로봇이 안정적으로 정지한 상태에서 실행하세요.';
  const arm = s.robot.joints.slice(12, 18);
  if (arm.length !== 6) return '최신 로봇 상태를 기다리고 있습니다.';
  if (!s.acknowledgedRunning
    && (s.robot.armStat.conStart || arm.some(j => j.run || j.locked)
      || s.robot.armStat.manualControl || s.robot.armStat.missionType !== 0
      || s.robot.armStat.motionCmd !== 0)) return ARM_ALREADY_STARTED;
  return null;
}

export type ArmStartAttempt = { phase: 'sending' | 'sent' | 'unknown'; error?: string; at: number };

export const ARM_START_RETRY_MS = 15000;

export function createArmStartRequests() {
  const attempts = new Map<string, ArmStartAttempt>();
  const inFlight = (key: string, now: number) => {
    const attempt = attempts.get(key);
    return !!attempt && (attempt.phase === 'sending' || now - attempt.at < ARM_START_RETRY_MS);
  };
  return {
    get: (key: string) => attempts.get(key),
    inFlight: (key: string, now = Date.now()) => inFlight(key, now),
    async request(context: ArmStartContext, expectedKey = context.robotKey) {
      const now = Date.now();
      const reason = armStartBlockReason(context, now, expectedKey);
      if (reason) throw new Error(reason);
      if (inFlight(context.robotKey, now)) throw new Error('이 앱에서 이미 팔 기동을 요청했습니다. 실제 상태와 로그를 확인하세요.');
      attempts.set(context.robotKey, { phase: 'sending', at: now });
      try {
        const response = await rest.commandStruct('', PROGRAM.Motion, 802, undefined, { replayOnReconnect: false });
        attempts.set(context.robotKey, { phase: 'sent', at: now });
        return response;
      } catch (e) {
        attempts.set(context.robotKey, { phase: 'unknown', at: now, error: e instanceof Error ? e.message : String(e) });
        throw e;
      }
    },
  };
}
export const armStartRequests = createArmStartRequests();
export const requestArmAutoStart = armStartRequests.request;

export function armStartFeedback(robot:RobotState|undefined,pdu:PduState|undefined,simulation=false) {
  const motors=simulation||pdu?.fetAdd===true?robot?.joints.slice(12,18)??[]:[];
  return [motors.filter(j=>j.connected).length,
    motors.filter(j=>j.connected&&j.run).length,
    motors.filter(j=>j.connected&&j.locked).length];
}
