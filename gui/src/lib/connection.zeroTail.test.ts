import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ owner: null, myIp: null }), setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({}), setState: () => {} }, noteMotionLink: () => {} }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({}), subscribe: () => () => {} } }));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({}) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({}) } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('@/lib/simEngine', () => ({ simEngine: { active: false } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, SIZEOF: {} }));
vi.mock('./rest', () => ({ rest: {}, actions: {}, HttpError: class {} }));
vi.mock('./commandBus', () => ({ setCommandSender: () => {} }));
vi.mock('./endpoints', () => ({ restBase: () => '' }));
vi.mock('./auth', () => ({ robotAuth: () => ({}) }));
vi.mock('./connectionRoute', () => ({ routeDetail: () => '', readStats: async () => ({}) }));
vi.mock('./connectTicket', () => ({ currentTicket: () => null }));
vi.mock('./rtcPeer', () => ({ RTCPeerConnection: class {}, RTCSessionDescription: class {} }));
vi.mock('./desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('./demo', () => ({ isDemo: () => false, startDemo: () => {}, demoRest: {} }));
vi.mock('./gait', () => ({ gait: {}, RL_GAIT: new Set() }));
vi.mock('./desktopWebrtc', () => ({ DesktopWebrtcBridge: class {} }));
vi.mock('./remoteIce', () => ({ modeField: () => '', rendezvousEnabled: () => false, rendezvousUrl: () => '', robotId: () => '', resolveIceServers: () => [] }));
vi.mock('./rendezvousClient', () => ({ rendezvousExchange: async () => null }));

import { connection } from './connection';

type Frame = { ly: number; rx: number };
let sent: Frame[] = [];

beforeEach(() => {
  vi.useFakeTimers();
  sent = [];
  const c = connection as unknown as Record<string, unknown>;
  c.lastOwnerPushAt = Date.now();
  c.lastOwnCheckAt = Date.now();
  c.motionDc = {
    readyState: 'open',
    bufferedAmount: 0,
    send: (buf: ArrayBuffer) => {
      const dv = new DataView(buf);
      sent.push({ ly: dv.getFloat32(4, true), rx: dv.getFloat32(8, true) });
    },
  };
});

afterEach(() => {
  connection.setAxes('L', 0, 0);
  connection.setAxes('R', 0, 0);
  vi.advanceTimersByTime(2000);
  vi.useRealTimers();
});

const TICK = 25;

describe('조이스틱 전송 루프 — 0 프레임 꼬리', () => {
  it('누르고 있는 동안은 5초가 지나도 멈추지 않는다(갑자기 안 가는 회귀 없음)', () => {
    connection.setAxes('L', 0, 0.6);
    vi.advanceTimersByTime(5000);
    expect(sent.length).toBeGreaterThanOrEqual(195);
    expect(sent.every((f) => f.ly !== 0)).toBe(true);
  });

  it('놓으면 0 프레임을 정확히 20장 보내고 멈춘다', () => {
    connection.setAxes('L', 0, 0.6);
    vi.advanceTimersByTime(TICK * 4);
    const moving = sent.length;
    connection.setAxes('L', 0, 0);
    vi.advanceTimersByTime(3000);
    const zeros = sent.slice(moving);
    expect(zeros.length).toBe(20);
    expect(zeros.every((f) => f.ly === 0 && f.rx === 0)).toBe(true);
  });

  it('0 을 보내는 도중 다시 밀면 곧바로 이어서 움직이고, 다시 놓으면 꼬리를 새로 센다', () => {
    connection.setAxes('L', 0, 0.6);
    vi.advanceTimersByTime(TICK * 4);
    connection.setAxes('L', 0, 0);
    vi.advanceTimersByTime(TICK * 5);
    const before = sent.length;
    connection.setAxes('R', 0.4, 0);
    vi.advanceTimersByTime(TICK * 3);
    expect(sent.slice(before).every((f) => f.rx === Math.fround(0.4))).toBe(true);
    const moving = sent.length;
    connection.setAxes('R', 0, 0);
    vi.advanceTimersByTime(3000);
    expect(sent.length - moving).toBe(20);
  });

  it('놓은 뒤에는 0 이 아닌 값이 한 장도 섞이지 않는다', () => {
    connection.setAxes('L', 0, -0.9);
    vi.advanceTimersByTime(TICK * 2);
    const moving = sent.length;
    connection.setAxes('L', 0, 0);
    vi.advanceTimersByTime(3000);
    expect(sent.slice(moving).some((f) => f.ly !== 0)).toBe(false);
  });
});
