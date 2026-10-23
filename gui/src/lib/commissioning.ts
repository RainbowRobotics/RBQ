import { rest } from './rest';
import { connection } from './connection';
import { PROGRAM } from './robotState';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { useRobots, useRobotReady, SIM_SERIAL } from '@/store/robots';
import { t } from './i18n';
import type { RobotState } from './robotState';

const DAEMON_CMD = {
  INIT_CHECK_DEVICE: 100,
  INIT_FIND_HOME: 101,
  SENSOR_IMU_NULL: 209,
  ACC_CALIBRATION: 225,
  GYRO_BIAS_SET: 240,
} as const;

export const CALIB_OK_GAIT_IDS = [-1, 0, 1];

function liveRobotNow(): RobotState | undefined {
  const r = useRobots.getState();
  const ready = useRobot.getState().conn === 'connected' && !r.switching && r.currentSerial !== SIM_SERIAL;
  return ready ? useTelemetry.getState().robot : undefined;
}
function useLiveRobot(): RobotState | undefined {
  const robot = useTelemetry((s) => s.robot);
  const sim = useRobots((s) => s.currentSerial === SIM_SERIAL);
  return useRobotReady() && !sim ? robot : undefined;
}
function useSimBlock(): CalibGuard | null {
  const sim = useRobots((s) => s.currentSerial === SIM_SERIAL);
  return sim ? { blocked: true, reason: t('시뮬레이터에서는 보정할 수 없습니다 — 실로봇을 고르세요') } : null;
}

export type CalibFix = 'sit' | 'stand' | 'autostart';
export type CalibGuard = { blocked: boolean; reason: string; fix?: CalibFix };

export type CalibOpts = { ignoreFall?: boolean };
function calibState(robot: RobotState | undefined, opts: CalibOpts = {}): CalibGuard {
  if (!robot) {
    return { blocked: true, reason: t('로봇 상태를 받고 있지 않음 — 보정 불가') };
  }
  if (robot.isFall && !opts.ignoreFall) return { blocked: true, reason: t('낙상 상태 — 보정 불가') };
  if (!CALIB_OK_GAIT_IDS.includes(robot.gaitId)) {
    return { blocked: true, reason: busyRun(robot.gaitId) ? t('무게 중심 보정·도킹 진행 중 — 끝난 뒤 실행하세요') : t('보행 중 — 정지(SIT/STAND/OFF) 상태에서만 보정할 수 있습니다'),
             ...(busyRun(robot.gaitId) ? {} : { fix: 'stand' as const }) };
  }
  return { blocked: false, reason: '' };
}

export function calibAllowed(opts: CalibOpts = {}): boolean {
  return !calibState(liveRobotNow(), opts).blocked;
}

export function useCalibGuard(opts: CalibOpts = {}): CalibGuard {
  const sim = useSimBlock();
  const st = calibState(useLiveRobot(), opts);
  return sim ?? st;
}

export const LEG_HOME_OK_GAIT_IDS = [-1, 0];
function legHomeState(robot: RobotState | undefined): CalibGuard {
  if (!robot) return { blocked: true, reason: t('로봇 상태를 받고 있지 않음 — 보정 불가') };
  if (robot.isFall) return { blocked: true, reason: t('낙상 상태 — 보정 불가') };
  if (robot.isStanding) {
    return { blocked: true, reason: t('로봇이 서 있습니다 — 앉힌 뒤 실행하세요(실행하면 제어가 끊겨 주저앉습니다)'),
             ...(busyRun(robot.gaitId) ? {} : { fix: 'sit' as const }) };
  }
  if (!LEG_HOME_OK_GAIT_IDS.includes(robot.gaitId)) {
    return { blocked: true, reason: t('앉히거나 제어를 끈 뒤 실행하세요 — 실행하면 제어가 끊겨 서 있으면 주저앉습니다'), fix: 'sit' };
  }
  return { blocked: false, reason: '' };
}

export function legHomeCalibAllowed(): boolean {
  return !legHomeState(liveRobotNow()).blocked;
}

export function useLegHomeGuard(): CalibGuard {
  const sim = useSimBlock();
  const st = legHomeState(useLiveRobot());
  return sim ?? st;
}

const GAIT_CONTROL_OFF = -1, GAIT_SITTING = 0, GAIT_ZMP_INIT = 8, GAIT_DOCKING = 10;

const busyRun = (gaitId: number) => gaitId === GAIT_ZMP_INIT || gaitId === GAIT_DOCKING;

function zmpState(robot: RobotState | undefined): CalibGuard {
  if (!robot) return { blocked: true, reason: t('로봇 상태를 받고 있지 않음 — 보정 불가') };
  if (robot.isFall) return { blocked: true, reason: t('낙상 상태 — 보정 불가') };
  if (robot.gaitId === GAIT_ZMP_INIT) return { blocked: true, reason: t('무게 중심 보정 진행 중') };
  if (robot.gaitId === GAIT_DOCKING) return { blocked: true, reason: t('도킹 중 — 도킹이 끝난 뒤 실행하세요') };
  if (robot.gaitId === GAIT_SITTING || robot.gaitId === GAIT_CONTROL_OFF) {
    return { blocked: true, reason: t('서 있어야 합니다 — 앉음·제어 꺼짐 상태에서는 무게 중심 보정을 시작할 수 없습니다'),
             fix: robot.gaitId === GAIT_CONTROL_OFF ? 'autostart' : 'stand' };
  }
  return { blocked: false, reason: '' };
}

export function zmpCalibAllowed(): boolean {
  return !zmpState(liveRobotNow()).blocked;
}

export function useZmpCalibGuard(): CalibGuard {
  const sim = useSimBlock();
  const st = zmpState(useLiveRobot());
  return sim ?? st;
}

function sitImuState(robot: RobotState | undefined, imuConnected: boolean | undefined): CalibGuard {
  if (!robot) return { blocked: true, reason: t('로봇 상태를 받고 있지 않음 — 보정 불가') };
  if (robot.isFall) return { blocked: true, reason: t('낙상 상태 — 보정 불가') };
  if (robot.isStanding) return { blocked: true, reason: t('로봇이 서 있음 — 앉힌 뒤 실행하세요'),
    ...(busyRun(robot.gaitId) ? {} : { fix: 'sit' as const }) };
  if (imuConnected === false) return { blocked: true, reason: t('IMU 가 연결되어 있지 않음 — 보정 불가') };
  return { blocked: false, reason: '' };
}

export function sitImuCalibAllowed(): boolean {
  return !sitImuState(liveRobotNow(), useRobot.getState().robot?.imu_connected).blocked;
}

export function useSitImuCalibGuard(): CalibGuard {
  const sim = useSimBlock();
  const st = sitImuState(useLiveRobot(), useRobot((s) => s.robot?.imu_connected));
  return sim ?? st;
}

export const commissioning = {
  canCheck: (ip: string) =>
    rest.commandStruct(ip, PROGRAM.Motion, DAEMON_CMD.INIT_CHECK_DEVICE, { char: [12] }),
  findHome: (ip: string) =>
    rest.commandStruct(ip, PROGRAM.Motion, DAEMON_CMD.INIT_FIND_HOME, { char: [12, 1] }),
  accCalibrate: (ip: string) =>
    rest.commandStruct(ip, PROGRAM.Motion, DAEMON_CMD.ACC_CALIBRATION),
  imuNull: (ip: string) =>
    rest.commandStruct(ip, PROGRAM.Motion, DAEMON_CMD.SENSOR_IMU_NULL),
  standIfStanding: () => {
    if (useTelemetry.getState().robot?.isStanding) connection.sendMotion('stand');
  },
  gyroBiasSet: (ip: string) =>
    rest.commandStruct(ip, PROGRAM.Motion, DAEMON_CMD.GYRO_BIAS_SET),
};
