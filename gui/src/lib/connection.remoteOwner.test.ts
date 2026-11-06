import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const robot = vi.hoisted(() => ({ conflict: false, conn: '' }));
const restOwnership = vi.hoisted(() => vi.fn(async (..._a: unknown[]) => ({ ownerIP: '', requesterIP: '10.0.0.5', IsOwner: false })));
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
vi.mock('./rest', () => ({ rest: { getOwnership: (...a: unknown[]) => restOwnership(...a), version: async () => ({ version: '1' }) }, actions: {}, HttpError: class {} }));
vi.mock('./commandBus', () => ({ setCommandSender: () => {} }));
vi.mock('./endpoints', () => ({ restBase: () => '' }));
vi.mock('./auth', () => ({ BASIC_AUTH: '' }));
vi.mock('./connectionRoute', () => ({ routeDetail: () => '', readStats: async () => ({}) }));
vi.mock('./rtcPeer', () => ({ RTCPeerConnection: class {}, RTCSessionDescription: class {} }));
vi.mock('./desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('./demoFlag', () => ({ isDemo: () => false, demoDriver: () => undefined }));
vi.mock('./gait', () => ({ gait: {}, RL_GAIT: new Set() }));
vi.mock('./remoteIce', () => ({ modeField: () => '', resolveIceServers: () => [] }));


import { connection } from './connection';
import { remote, type RemoteNet } from '@/modules/registry';

type Priv = { getOwnership: () => Promise<unknown>; claimIfFree: () => Promise<void>; dcCommand: (...a: unknown[]) => Promise<unknown>; claimOwnership: () => Promise<void> };
const c = connection as unknown as Priv;
const realDc = c.dcCommand;
const realClaim = c.claimOwnership;
const fakeNet = (on: boolean) => ({ active: () => on } as unknown as RemoteNet);

beforeEach(() => {
  restOwnership.mockClear();
  c.dcCommand = vi.fn(async () => ({ ownerIP: '', requesterIP: '10.0.0.5', IsOwner: false }));
  c.claimOwnership = vi.fn(async () => {});
});
afterEach(() => { remote.net = undefined; c.dcCommand = realDc; c.claimOwnership = realClaim; });

describe('소유권 — 원격이면 command DC, 조회 실패 시 자동 claim 금지', () => {
  it('원격이면 command DC 로 묻는다(REST 는 닿지 않는다)', async () => {
    remote.net = fakeNet(true);
    await c.getOwnership();
    expect(c.dcCommand).toHaveBeenCalledWith('GET', '/api/gamepad/ownership');
    expect(restOwnership).not.toHaveBeenCalled();
  });
  it('직결(원격 모듈 없음 포함)이면 REST 로 묻는다', async () => {
    await c.getOwnership();
    remote.net = fakeNet(false);
    await c.getOwnership();
    expect(restOwnership).toHaveBeenCalledTimes(2);
    expect(c.dcCommand).not.toHaveBeenCalled();
  });
  it('★원격에서 조회가 실패하면 claim 하지 않는다', async () => {
    remote.net = fakeNet(true);
    c.dcCommand = vi.fn(async () => { throw new Error('timeout'); });
    await c.claimIfFree();
    expect(c.claimOwnership).not.toHaveBeenCalled();
  });
  it('직결에서 조회가 실패하면 종전대로 claim 한다', async () => {
    restOwnership.mockRejectedValueOnce(new Error('timeout'));
    await c.claimIfFree();
    expect(c.claimOwnership).toHaveBeenCalled();
  });
});
