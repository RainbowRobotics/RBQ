import { describe, it, expect, vi, beforeEach } from 'vitest';

const robot = vi.hoisted(() => ({ conflict: false, conn: '' }));
const own = vi.hoisted(() => ({ resp: { ownerIP: '', requesterIP: '10.0.0.5', IsOwner: false } as { ownerIP: string; requesterIP: string; IsOwner: boolean } }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({
  owner: null, myIp: null, setConn: (c: string) => { robot.conn = c; }, setTakeoverConflict: (v: boolean) => { robot.conflict = v; },
  setRoute: () => {}, setVia: () => {}, setConnError: () => {}, setOwnership: () => {},
}), setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({}), setState: () => {} }, noteMotionLink: () => {} }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({}), subscribe: () => () => {} } }));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({ clearFeatures: () => {} }) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({}) } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('@/lib/simEngine', () => ({ simEngine: { active: false } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, parseRadiation: () => null, SIZEOF: {}, RADIATION_SIZE: 0 }));
vi.mock('./rest', () => ({ rest: { getOwnership: async () => own.resp, version: async () => ({ version: '1' }) }, actions: {}, HttpError: class {} }));
vi.mock('./commandBus', () => ({ setCommandSender: () => {} }));
vi.mock('./endpoints', () => ({ restBase: () => '' }));
vi.mock('./auth', () => ({ BASIC_AUTH: '' }));
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

const c = connection as unknown as Record<string, unknown> & { onDrop: () => void; clearReconnect: () => void };

async function dropWith(resp: typeof own.resp) {
  own.resp = resp;
  robot.conflict = false;
  c.wantConnected = true;
  c.establishedAt = Date.now() - 5000;
  c.dropHistory = [];
  c.pc = null;
  c.ip = '192.168.0.10';
  c.onDrop();
  await new Promise((r) => setTimeout(r, 0));
  const r = { conflict: robot.conflict, keepsReconnecting: c.wantConnected === true };
  c.clearReconnect();
  c.wantConnected = false;
  return r;
}

beforeEach(() => { robot.conflict = false; });

describe('수립 세션 드롭 — 한 번은 그냥 재접속', () => {
  it('다른 기기가 제어권을 쥐고 있어도 팝업 없이 재접속(관전으로 붙는다)', async () => {
    expect(await dropWith({ ownerIP: '10.0.0.9', requesterIP: '10.0.0.5', IsOwner: false })).toEqual({ conflict: false, keepsReconnecting: true });
  });
  it('소유권이 공석이면(워치독 드롭 뒤 로봇이 비움) 팝업 없이 재접속', async () => {
    expect(await dropWith({ ownerIP: '', requesterIP: '10.0.0.5', IsOwner: false })).toEqual({ conflict: false, keepsReconnecting: true });
  });
  it('아직 내 소유면(텔레메트리만 멈춤) 팝업 없이 재접속', async () => {
    expect(await dropWith({ ownerIP: '10.0.0.5', requesterIP: '10.0.0.5', IsOwner: true })).toEqual({ conflict: false, keepsReconnecting: true });
  });
});
