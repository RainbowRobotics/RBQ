import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({
  owner: null, myIp: null, setIp: () => {}, setConnError: () => {}, setConn: () => {},
}), setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({ setSensorLayout: () => {} }), setState: () => {} }, noteMotionLink: () => {} }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({}), subscribe: () => () => {} } }));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({ clearFeatures: () => {} }) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({}) } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('@/lib/simEngine', () => ({ simEngine: { active: false, intercept: () => null } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, SIZEOF: {} }));
vi.mock('./rest', () => ({
  rest: { version: () => new Promise(() => {}) }, actions: {},
  HttpError: class extends Error { status: number; constructor(status: number, m: string) { super(m); this.status = status; } },
}));
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

type Pending = { method: string; path: string };
const c = connection as unknown as {
  connect: (ip: string) => void; disconnect: () => void;
  dcCommand: (m: string, p: string, b?: object) => Promise<unknown>;
  open: () => Promise<void>; cmdPending: Map<string, Pending>;
};

beforeEach(() => {
  vi.useFakeTimers();
  c.open = async () => {};
  c.cmdPending.clear();
});
afterEach(() => {
  c.cmdPending.clear();
  vi.useRealTimers();
});

describe('로봇을 바꿀 때 대기 명령', () => {
  it('다른 로봇으로 바꾸면 쓰기 명령은 응답 없음(0)으로 돌려주고 비운다 — 읽기(GET)는 둔다', async () => {
    c.connect('10.0.0.1');
    const post = c.dcCommand('POST', '/api/firmware/update', { board: 'all', power_down: true }).catch((e: unknown) => e);
    void c.dcCommand('GET', '/api/firmware/update').catch(() => {});
    c.disconnect();
    c.connect('10.0.0.2');
    const e = (await post) as { status: number; message: string };
    expect(e.status).toBe(0);
    expect(e.message).toContain('다른 로봇으로 바뀌어');
    expect([...c.cmdPending.values()].map((p) => `${p.method} ${p.path}`)).toEqual(['GET /api/firmware/update']);
  });
  it('같은 로봇으로 다시 붙으면 그대로 둔다 — 재연결 flush 가 보낸다(순단 중 E-STOP 같은 명령)', () => {
    c.connect('10.0.0.3');
    void c.dcCommand('POST', '/api/estop', {}).catch(() => {});
    c.connect('10.0.0.3');
    expect([...c.cmdPending.values()].map((p) => p.method)).toEqual(['POST']);
  });
});
