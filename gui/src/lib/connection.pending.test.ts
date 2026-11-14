import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({
  owner: null, myIp: null, setIp: () => {}, setConnError: () => {}, setConn: () => {}, applyHealth: () => {},
  setOwnership: () => {}, pushLog: () => {},
}), setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({ setSensorLayout: () => {} }), setState: () => {} }, noteMotionLink: () => {} }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({}), subscribe: () => () => {} } }));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({ clearFeatures: () => {} }) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({}) } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('@/lib/simEngine', () => ({ simEngine: { active: false, intercept: () => null } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, SIZEOF: {}, CRUISE_GAIT_IDS: new Set([3, 6, 30, 42]) }));
vi.mock('./rest', () => ({
  rest: { version: () => new Promise(() => {}) }, actions: {},
  HttpError: class extends Error { status: number; constructor(status: number, m: string) { super(m); this.status = status; } },
}));
vi.mock('./commandBus', () => ({ setCommandSender: () => {} }));
vi.mock('./endpoints', () => ({ restBase: () => '' }));
vi.mock('./auth', () => ({ robotAuth: () => ({}) }));
vi.mock('./connectionRoute', () => ({ routeDetail: () => '', readStats: async () => ({}) }));
vi.mock('./rtcPeer', () => ({ RTCPeerConnection: class {}, RTCSessionDescription: class {} }));
vi.mock('./desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('./demoFlag', () => ({ isDemo: () => false, demoDriver: () => undefined }));
vi.mock('./gait', () => ({ gait: {}, RL_GAIT: new Set() }));
vi.mock('./remoteIce', () => ({ modeField: () => '', resolveIceServers: () => [] }));
vi.mock('./robotTlsPlatform', () => ({ robotTlsPlatform: { probe: async () => ({ kind: 'down' }), engage: async () => {}, block: () => {}, kept: () => false, base: () => undefined } }));

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

describe('채널이 닫혀 있을 때 대기 명령', () => {
  it('쓰기는 대기열에 넣지 않고 응답 없음(0)으로 바로 돌려준다 — 읽기(GET)는 다른 로봇으로 바꿔도 둔다', async () => {
    c.connect('10.0.0.1');
    const post = c.dcCommand('POST', '/api/firmware/update', { board: 'all', power_down: true }).catch((e: unknown) => e);
    void c.dcCommand('GET', '/api/firmware/update').catch(() => {});
    const e = (await post) as { status: number; message: string };
    expect(e.status).toBe(0);
    expect(e.message).toContain('연결이 없어 보내지 않았습니다');
    c.disconnect();
    c.connect('10.0.0.2');
    expect([...c.cmdPending.values()].map((p) => `${p.method} ${p.path}`)).toEqual(['GET /api/firmware/update']);
  });
  it('E-STOP 은 같은 로봇으로 다시 붙을 때까지 기다린다(재연결 flush 가 보낸다) — 사용자가 끊으면 버린다', async () => {
    c.connect('10.0.0.3');
    const estop = c.dcCommand('POST', '/api/motion/command', { cmd: 'estop' }).catch((e: unknown) => e);
    c.connect('10.0.0.3');
    expect([...c.cmdPending.values()].map((p) => p.path)).toEqual(['/api/motion/command']);
    c.disconnect();
    expect(((await estop) as { status: number }).status).toBe(0);
    expect(c.cmdPending.size).toBe(0);
  });
});
