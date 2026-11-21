import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const pushLog = vi.fn();
let demo = false;
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({
  owner: null, myIp: null, setIp: () => {}, setConnError: () => {}, setConn: () => {}, applyHealth: () => {},
  setOwnership: () => {}, pushLog,
}), setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({ setSensorLayout: () => {} }), setState: () => {} }, noteMotionLink: () => {} }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({}), subscribe: () => () => {} } }));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({ clearFeatures: () => {} }) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({}) } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('@/lib/simEngine', () => ({ simEngine: { active: false, intercept: () => null } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, SIZEOF: {}, CRUISE_GAIT_IDS: new Set([3, 6, 30, 42]) }));
vi.mock('./rest', () => ({
  rest: { version: () => new Promise(() => {}), getOwnership: () => new Promise(() => {}) }, actions: {},
  HttpError: class extends Error { status: number; constructor(status: number, m: string) { super(m); this.status = status; } },
}));
vi.mock('./commandBus', () => ({ setCommandSender: () => {} }));
vi.mock('./endpoints', () => ({ restBase: () => '' }));
vi.mock('./auth', () => ({ robotAuth: () => ({}) }));
vi.mock('./connectionRoute', () => ({ routeDetail: () => '', readStats: async () => ({}) }));
vi.mock('./rtcPeer', () => ({ RTCPeerConnection: class {}, RTCSessionDescription: class {} }));
vi.mock('./desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('./demoFlag', () => ({ isDemo: () => demo, demoDriver: () => (demo ? { rest: () => ({}) } : undefined) }));
vi.mock('./gait', () => ({ gait: {}, RL_GAIT: new Set() }));
vi.mock('./remoteIce', () => ({ modeField: () => '', resolveIceServers: () => [] }));
vi.mock('./robotTlsPlatform', () => ({ robotTlsPlatform: { probe: async () => ({ kind: 'down' }), engage: async () => {}, block: () => {}, kept: () => false, base: () => undefined } }));

import { connection } from './connection';

type Dc = { readyState: string; send: (s: string) => void } | null;
const c = connection as unknown as {
  connect: (ip: string) => void; disconnect: () => void; open: () => Promise<void>;
  sendMotion: (name: string) => void;
  estopDc: Dc; commandDc: Dc; cmdPending: Map<string, { path: string; body: { cmd?: string } }>; cmdTimer: unknown;
};
const CLOSED = 'E-Stop 전송 실패 — 채널이 닫혀 있습니다. 물리 비상정지를 사용하세요';
const notices = () => pushLog.mock.calls.map(([l]) => l.msg).filter((m: string) => m.startsWith('E-Stop'));
const dc = (readyState: string) => ({ readyState, send: vi.fn() });

beforeEach(() => {
  vi.useFakeTimers();
  c.open = async () => {};
  c.connect('10.0.0.1');
  c.cmdPending.clear();
  c.cmdTimer = null;
  pushLog.mockClear();
  demo = false;
});
afterEach(() => {
  c.estopDc = null; c.commandDc = null;
  c.disconnect();
  vi.useRealTimers();
});

describe('E-STOP — 보낼 채널이 없을 때', () => {
  it('둘 다 닫혀 있으면 누른 그 자리에서 알리고, 대기열에는 남긴다(다시 붙으면 나간다)', () => {
    c.estopDc = dc('closed'); c.commandDc = dc('connecting');
    c.sendMotion('estop');
    expect(notices()).toEqual([CLOSED]);
    expect([...c.cmdPending.values()].map((p) => p.body.cmd)).toEqual(['estop']);
  });

  it('채널 객체가 아직 없어도(끊긴 직후) 같다 — 8초 데드라인이 지나면 종전의 실패도 그대로 뜬다', async () => {
    c.estopDc = null; c.commandDc = null;
    c.sendMotion('estop');
    expect(notices()).toEqual([CLOSED]);
    await vi.advanceTimersByTimeAsync(9000);
    expect(notices()).toEqual([CLOSED, 'E-Stop 전송 실패 — 응답 없음. 물리 E-Stop을 사용하세요']);
  });

  it('전용 채널이 열려 있으면 알리지 않는다(바로 한 발 나갔다)', () => {
    const e = dc('open');
    c.estopDc = e; c.commandDc = null;
    c.sendMotion('estop');
    expect(e.send).toHaveBeenCalledTimes(1);
    expect(notices()).toEqual([]);
  });

  it('command 채널이 열려 있으면 알리지 않는다(그쪽으로 바로 나간다)', () => {
    const cmd = dc('open');
    c.estopDc = dc('closed'); c.commandDc = cmd;
    c.sendMotion('estop');
    expect(cmd.send).toHaveBeenCalledTimes(1);
    expect(notices()).toEqual([]);
  });

  it('데모는 채널 없이 구동기가 받는다 — 거짓 실패를 띄우지 않는다', () => {
    demo = true;
    c.estopDc = null; c.commandDc = null;
    c.sendMotion('estop');
    expect(notices()).toEqual([]);
  });
});
