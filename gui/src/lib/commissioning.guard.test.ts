import { describe, it, expect, vi, beforeEach } from 'vitest';

const st = vi.hoisted(() => ({
  robot: { conn: 'connected', robot: undefined as unknown },
  robots: { switching: false, currentSerial: 'R1' as string | null },
  tele: { robot: { gaitId: 0, isFall: false, isStanding: false } as unknown },
}));
vi.mock('./rest', () => ({ rest: {} }));
vi.mock('./connection', () => ({ connection: {} }));
vi.mock('./i18n', () => ({ t: (s: string) => s }));
vi.mock('@/store/robot', () => ({ useRobot: Object.assign((f: (s: unknown) => unknown) => f(st.robot), { getState: () => st.robot }) }));
vi.mock('@/store/robots', () => ({
  useRobots: Object.assign((f: (s: unknown) => unknown) => f(st.robots), { getState: () => st.robots }),
  useRobotReady: () => st.robot.conn === 'connected' && !st.robots.switching,
  SIM_SERIAL: '__sim__',
}));
vi.mock('@/store/telemetry', () => ({ useTelemetry: Object.assign((f: (s: unknown) => unknown) => f(st.tele), { getState: () => st.tele }) }));

import { calibAllowed, legHomeCalibAllowed, sitImuCalibAllowed } from './commissioning';

beforeEach(() => { st.robot.conn = 'connected'; st.robots.switching = false; st.robots.currentSerial = 'R1'; });

describe('보정 가드 — 연결·전환', () => {
  it('붙어 있고 앉아 있으면 통과', () => {
    expect(calibAllowed()).toBe(true);
    expect(legHomeCalibAllowed()).toBe(true);
    expect(sitImuCalibAllowed()).toBe(true);
  });
  it('로봇을 바꾸는 중이면 텔레메트리가 남아 있어도 막는다', () => {
    st.robots.switching = true;
    expect(calibAllowed()).toBe(false);
    expect(legHomeCalibAllowed()).toBe(false);
    expect(sitImuCalibAllowed()).toBe(false);
  });
  it('시뮬을 고른 동안은 실로봇에 붙어 있어도 막는다', () => {
    st.robots.currentSerial = '__sim__';
    expect(calibAllowed()).toBe(false);
  });
  it('연결이 끊겼으면 막는다', () => {
    st.robot.conn = 'connecting';
    expect(calibAllowed()).toBe(false);
  });
});
