import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const links: string[] = vi.hoisted(() => []);

vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ owner: null, myIp: null }), setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({}), setState: () => {} }, noteMotionLink: (c: string) => links.push(c) }));
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
vi.mock('./rtcPeer', () => ({ RTCPeerConnection: class {}, RTCSessionDescription: class {} }));
vi.mock('./desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('./demoFlag', () => ({ isDemo: () => false, demoDriver: () => undefined }));
vi.mock('./gait', () => ({ gait: {}, RL_GAIT: new Set() }));
vi.mock('./remoteIce', () => ({ modeField: () => '', resolveIceServers: () => [] }));

import { connection } from './connection';

const c = connection as unknown as Record<string, any>;
beforeEach(() => {
  vi.useFakeTimers(); links.length = 0;
  c.pc = {}; c.onDrop = vi.fn();
  c.stopWatchdog(); c.startWatchdog();
});
afterEach(() => { c.stopWatchdog(); c.pc = null; vi.useRealTimers(); });

describe('상태 프레임 지연 표시', () => {
  it('터널 수신은 있는데 상태가 3 s 넘게 멎으면 연결중 — 재연결은 안 한다', () => {
    c.lastStateAt = Date.now();
    for (let i = 0; i < 4; i++) { vi.advanceTimersByTime(1000); c.lastRxAt = Date.now(); }
    vi.advanceTimersByTime(2500);
    expect(links).toContain('connecting');
    expect(c.onDrop).not.toHaveBeenCalled();
  });
  it('첫 상태 프레임 전에는 표시를 바꾸지 않는다', () => {
    c.lastStateAt = 0; c.lastRxAt = Date.now();
    vi.advanceTimersByTime(5000);
    expect(links).not.toContain('connecting');
  });
});
