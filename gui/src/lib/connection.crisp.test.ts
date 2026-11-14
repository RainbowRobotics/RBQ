import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const h = vi.hoisted(() => {
  class FakeDc {
    readyState = 'connecting';
    sent: string[] = [];
    binaryType = '';
    onopen: (() => void) | null = null;
    onmessage: ((e: { data: unknown }) => void) | null = null;
    onclose: (() => void) | null = null;
    bufferedAmount = 0;
    send(d: string) { this.sent.push(d); }
    open() { this.readyState = 'open'; this.onopen?.(); }
  }
  class FakePc {
    dcs = new Map<string, FakeDc>();
    connectionState = 'new';
    iceGatheringState = 'complete';
    localDescription = { sdp: 'offer-sdp' };
    onconnectionstatechange: (() => void) | null = null;
    remoteSet = false;
    closed = false;
    constructor() { h.pcs.push(this); }
    createDataChannel(label: string) { const d = new FakeDc(); this.dcs.set(label, d); return d; }
    async createOffer() { return { type: 'offer', sdp: 'offer-sdp' }; }
    async setLocalDescription() {}
    async setRemoteDescription() { this.remoteSet = true; }
    close() { this.closed = true; }
    state(s: string) { this.connectionState = s; this.onconnectionstatechange?.(); }
  }
  type Offer = { answer: (features?: string[]) => void; signal?: AbortSignal };
  const robot = {
    conn: 'disconnected', owner: '', myIp: '', isMine: false, logs: [] as string[],
    setIp() {}, setConnError() {}, setVia() {}, setRoute() {}, applyHealth() {}, setOwnerSupport() {},
    setConn(c: string) { robot.conn = c; },
    setOwner(o: string) { robot.owner = o; },
    setOwnership(o: { owner: string; myIp: string; isMine: boolean }) { Object.assign(robot, o); },
    pushLog(l: { msg: string }) { robot.logs.push(l.msg); },
    setTakeoverConflict() {},
  };
  const tls = {
    probe: vi.fn(async (_h: string, _p: number): Promise<unknown> => ({ kind: 'down' })),
    engage: vi.fn(async () => {}), block: vi.fn(), kept: vi.fn((_h: string, _p: number) => false), pinnedFp: vi.fn((_h: string, _p: number): string | undefined => undefined), base: vi.fn((_h: string, _p: number): string | undefined => undefined), plainHeld: () => false,
  };
  return { FakePc, pcs: [] as FakePc[], offers: [] as Offer[], hangups: [] as unknown[], sent: [] as { url: string; auth: boolean }[], robot, tls };
});

vi.mock('@/store/robot', () => ({ useRobot: { getState: () => h.robot, setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({ setSensorLayout: () => {} }), setState: () => {} }, noteMotionLink: () => {}, noteJoyUnstable: () => {} }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({ webrtcToken: '', setRobotVersion: () => {} }), subscribe: () => () => {} } }));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({ clearFeatures: () => {}, setFeatures: () => {} }) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({ show: () => {} }) } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('@/lib/simEngine', () => ({ simEngine: { active: false, intercept: () => null } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, SIZEOF: {}, CRUISE_GAIT_IDS: new Set([3, 6, 30, 42]) }));
vi.mock('./rest', () => ({
  rest: { version: vi.fn(() => new Promise(() => {})), getOwnership: vi.fn(), robotFeatures: async () => ({}) },
  actions: { walkParams: vi.fn(async () => {}), setBodyTilt: vi.fn(async () => {}) },
  HttpError: class extends Error { status: number; constructor(status: number, m: string) { super(m); this.status = status; } },
}));
vi.mock('./commandBus', () => ({ setCommandSender: () => {} }));
vi.mock('./auth', () => ({ robotAuth: () => ({ Authorization: 'Basic test' }), reportAuthFailure: () => true }));
vi.mock('./connectionRoute', () => ({ routeDetail: () => ({}), readStats: () => [] }));
vi.mock('./rtcPeer', () => ({ RTCPeerConnection: h.FakePc, RTCSessionDescription: class { constructor(x: object) { Object.assign(this, x); } } }));
vi.mock('./robotTunnel', () => ({ attachTunnel: () => {} }));
vi.mock('./installId', () => ({ installId: () => 'test', installIdReady: Promise.resolve() }));
vi.mock('./demoFlag', () => ({ isDemo: () => false, demoDriver: () => undefined }));
vi.mock('./gait', () => ({ gait: {}, RL_GAIT: {} }));
vi.mock('./remoteIce', () => ({ modeField: () => ({}), resolveIceServers: async () => [] }));
vi.mock('./robotTlsPlatform', () => ({ robotTlsPlatform: h.tls }));

import { connection } from './connection';
import { isFetchFailure } from './fetchFailure';
import { rest, actions } from './rest';
import { setTlsPinSource, setTlsIssueHandler, type TlsIssue } from './robotTls';
import { remote, type RemoteNet } from '@/modules/registry';

const c = connection as unknown as Record<string, unknown>;
const getOwnership = rest.getOwnership as unknown as ReturnType<typeof vi.fn>;
const flush = () => vi.advanceTimersByTimeAsync(0);
const lastPc = () => h.pcs[h.pcs.length - 1];
const cmdDc = () => lastPc().dcs.get('command')!;

async function startConnect() {
  connection.connect('10.0.0.1');
  await flush();
}
async function connectFully(features: string[] = []) {
  await startConnect();
  h.offers[h.offers.length - 1].answer(features);
  await flush();
  lastPc().state('connected');
  cmdDc().open();
  await flush();
}

beforeEach(() => {
  vi.useFakeTimers();
  c.cmdTimer = null;
  h.pcs.length = 0; h.offers.length = 0; h.hangups.length = 0; h.robot.logs.length = 0;
  Object.assign(h.robot, { conn: 'disconnected', owner: '', myIp: '', isMine: false });
  getOwnership.mockReset();
  getOwnership.mockResolvedValue({ ownerIP: '', requesterIP: '10.0.0.5', IsOwner: false });
  (actions.walkParams as ReturnType<typeof vi.fn>).mockClear();
  h.sent.length = 0;
  vi.stubGlobal('fetch', vi.fn((url: string, init: { body: string; signal?: AbortSignal; headers?: Record<string, string> }) => {
    h.sent.push({ url, auth: !!init.headers?.Authorization });
    if (url.endsWith('/api/webrtc/hangup')) { h.hangups.push(JSON.parse(init.body)); return Promise.resolve({ ok: true }); }
    return new Promise((resolve) => h.offers.push({
      signal: init.signal,
      answer: (features = []) => resolve({ status: 200, ok: true, json: async () => ({ sdp: 'answer-sdp', features }) }),
    }));
  }));
});
afterEach(() => {
  connection.disconnect();
  vi.unstubAllGlobals();
  vi.useRealTimers();
});

describe('끊기는 그 자리에서 끝난다', () => {
  it('offer 대기 중에 끊으면 offer 를 취소하고 로봇에 hangup 을 보낸다 — 늦은 answer 가 되살리지 못한다', async () => {
    await startConnect();
    expect(h.robot.conn).toBe('connecting');
    const offer = h.offers[0];
    connection.disconnect();
    expect(h.robot.conn).toBe('disconnected');
    expect(offer.signal?.aborted).toBe(true);
    expect(h.hangups).toEqual([{ clientId: 'rbq-app-test', token: '' }]);
    offer.answer();
    await flush();
    expect(lastPc().remoteSet).toBe(false);
    lastPc().state('connected');
    expect(h.robot.conn).toBe('disconnected');
    await vi.advanceTimersByTimeAsync(20000);
    expect(h.pcs.length).toBe(1);
  });

  it('끊으면 소유권 표시·대기 쓰기·디바운스 PUT 이 함께 사라진다', async () => {
    await connectFully();
    Object.assign(h.robot, { owner: '10.0.0.5', myIp: '10.0.0.5', isMine: true });
    const write = connection.dcCommand('POST', '/api/robot/led', { on: true }).catch((e: { status: number }) => e);
    connection.putWalkParams({ body_height: 1, max_speed: 1, foot_height: 1 });
    connection.disconnect();
    expect(h.robot).toMatchObject({ owner: '', myIp: '', isMine: false });
    expect(((await write) as { status: number }).status).toBe(0);
    await vi.advanceTimersByTimeAsync(1000);
    expect(actions.walkParams).not.toHaveBeenCalled();
  });

  it('끊은 뒤에 도착한 소유권 조회 답은 아무것도 바꾸지 않는다', async () => {
    let answer: (v: unknown) => void = () => {};
    getOwnership.mockImplementation(() => new Promise((r) => { answer = r; }));
    await connectFully();
    connection.disconnect();
    answer({ ownerIP: '10.0.0.5', requesterIP: '10.0.0.5', IsOwner: true });
    await flush();
    expect(h.robot).toMatchObject({ owner: '', isMine: false });
  });
});

describe('새로 붙은 앱은 늘 관전 — 자동 claim 없음', () => {
  it('붙어도, 임자가 비어 있어도 잡지 않는다', async () => {
    const claim = vi.spyOn(connection, 'claimOwnership');
    await connectFully();
    expect(getOwnership).toHaveBeenCalled();
    expect(h.robot.myIp).toBe('10.0.0.5');
    for (let i = 0; i < 8; i++) {
      lastPc().dcs.get('log')!.onmessage?.({ data: JSON.stringify({ t: 'gamepad_status', owner: '' }) });
      await vi.advanceTimersByTimeAsync(1000);
    }
    expect(claim).not.toHaveBeenCalled();
    expect(cmdDc().sent.some((s) => s.includes('/api/gamepad/ownership'))).toBe(false);
    claim.mockRestore();
  });

  it('소유권 조회가 실패해도 잡지 않는다(직결)', async () => {
    getOwnership.mockRejectedValue(new Error('timeout'));
    const claim = vi.spyOn(connection, 'claimOwnership');
    await connectFully();
    await vi.advanceTimersByTimeAsync(2000);
    expect(claim).not.toHaveBeenCalled();
    claim.mockRestore();
  });
});

describe('예전에 누른 명령은 나중에 돌지 않는다', () => {
  it('연결이 없을 때 누른 쓰기는 바로 거절되고 다시 붙어도 안 나간다 — E-STOP 만 다시 열린 채널로 나간다', async () => {
    await connectFully();
    lastPc().state('failed');
    expect(h.robot.conn).toBe('connecting');
    const stand = await connection.dcCommand('POST', '/api/motion/command', { cmd: 'stand' }).catch((e: { status: number; message: string }) => e) as { status: number; message: string };
    expect(stand.status).toBe(0);
    expect(stand.message).toContain('연결이 없어 보내지 않았습니다');
    expect(h.robot.logs).toContain('연결이 없어 보내지 않았습니다');
    connection.sendMotion('estop');
    await vi.advanceTimersByTimeAsync(500);
    h.offers[1].answer();
    await flush();
    lastPc().state('connected');
    cmdDc().open();
    const sent = cmdDc().sent.join('\n');
    expect(sent).toContain('"cmd":"estop"');
    expect(sent).not.toContain('"cmd":"stand"');
  });
});

describe('"…중" 에 머물지 않는다', () => {
  it('20 초 안에 붙지 못하면 걷고 다시 시도한다', async () => {
    await startConnect();
    await vi.advanceTimersByTimeAsync(19900);
    expect(h.pcs.length).toBe(1);
    await vi.advanceTimersByTimeAsync(200);
    expect(h.pcs[0].closed).toBe(true);
    expect(h.offers[0].signal?.aborted).toBe(true);
    await vi.advanceTimersByTimeAsync(600);
    expect(h.pcs.length).toBe(2);
    expect(h.offers.length).toBe(2);
    expect(h.robot.conn).toBe('connecting');
  });

  it('짧게 세 번 끊겨도 재접속을 멈추지 않는다(옛 "30초 3회 = 충돌" 판정 없음)', async () => {
    await connectFully();
    for (let i = 0; i < 3; i++) {
      lastPc().state('failed');
      await vi.advanceTimersByTimeAsync(500);
      h.offers[h.offers.length - 1].answer();
      await flush();
      lastPc().state('connected');
    }
    expect(h.pcs.length).toBe(4);
    expect(h.robot.conn).toBe('connected');
  });
});

describe('생존 신호(liveness)', () => {
  const beats = () => cmdDc().sent.filter((s) => s === '{"t":"alive"}').length;
  it('광고한 로봇에는 열리자마자 한 번, 그 뒤 5 초마다 보낸다 — 끊기면 멈춘다', async () => {
    await connectFully(['command-dc', 'liveness']);
    expect(beats()).toBe(1);
    await vi.advanceTimersByTimeAsync(5000);
    expect(beats()).toBe(2);
    await vi.advanceTimersByTimeAsync(5000);
    expect(beats()).toBe(3);
    const dc = cmdDc();
    connection.disconnect();
    dc.readyState = 'open';
    await vi.advanceTimersByTimeAsync(10000);
    expect(dc.sent.filter((s) => s === '{"t":"alive"}').length).toBe(3);
  });
  it('앱이 25 초 넘게 멈췄다 풀리면(신호 사이가 벌어짐) 바로 새로 붙는다 — 로봇은 30 초에 그 세션을 닫았다', async () => {
    await connectFully(['command-dc', 'liveness']);
    const pc0 = lastPc();
    vi.setSystemTime(Date.now() + 30000);
    const orig = performance.now.bind(performance);
    const spy = vi.spyOn(performance, 'now').mockImplementation(() => orig() + 30000);
    const log = pc0.dcs.get('log')!;
    for (let i = 0; i < 4; i++) { log.onmessage?.({ data: JSON.stringify({ t: 'gamepad_status', owner: '' }) }); await vi.advanceTimersByTimeAsync(1000); }
    await vi.advanceTimersByTimeAsync(1000);
    spy.mockRestore();
    expect(pc0.closed).toBe(true);
    expect(h.robot.conn).not.toBe('connected');
  });
  it('기기 시각만 30 초 뛰면(NTP·로봇 AP 시각 보정) 멀쩡한 세션은 그대로 — 멈춤 판정은 단조 시계', async () => {
    await connectFully(['command-dc', 'liveness']);
    const pc0 = lastPc();
    vi.setSystemTime(Date.now() + 30000);
    const log = pc0.dcs.get('log')!;
    for (let i = 0; i < 6; i++) { log.onmessage?.({ data: JSON.stringify({ t: 'gamepad_status', owner: '' }) }); await vi.advanceTimersByTimeAsync(1000); }
    expect(pc0.closed).toBe(false);
    expect(h.robot.conn).toBe('connected');
  });
  it('광고하지 않는 옛 로봇에는 아무것도 보내지 않는다', async () => {
    await connectFully(['command-dc']);
    await vi.advanceTimersByTimeAsync(10000);
    expect(beats()).toBe(0);
  });
});

describe('지문 판정 — 비번 붙은 요청 전에, 어긋나면 멈춘다', () => {
  const pins = (p: Record<string, string>) => setTlsPinSource(() => ({ selectedKey: 'S', selectedSsid: null, pins: p, keyFor: async (s) => s, currentSsid: async () => null }));
  const tlsAt = (fps: Record<number, string>) => h.tls.probe.mockImplementation(async (_h, port) =>
    (fps[port] ? { kind: 'tls', fp: fps[port], serial: port === 8080 ? 'S' : null } : { kind: 'down' }));
  const issues: TlsIssue[] = [];
  beforeEach(() => { issues.length = 0; setTlsIssueHandler((i) => issues.push(i)); h.tls.probe.mockClear(); h.tls.block.mockClear(); h.tls.engage.mockClear(); });
  afterEach(() => { pins({}); h.tls.probe.mockReset(); h.tls.probe.mockImplementation(async () => ({ kind: 'down' })); });

  it('tls_changed_refuses_before_offer_and_stops', async () => {
    pins({ 'S@8080': 'G' });
    tlsAt({ 8080: 'F' });
    await startConnect();
    expect(h.offers.length).toBe(0);
    expect(h.robot.conn).toBe('disconnected');
    expect(issues).toEqual([{ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [] }]);
    await vi.advanceTimersByTimeAsync(20000);
    expect(h.offers.length).toBe(0);
    expect(h.tls.probe.mock.calls.filter(([, p]) => p === 8080).length).toBe(1);
  });

  it('tls_mixed_ports_refused_blocks_every_port', async () => {
    pins({ 'S@8082': 'G' });
    tlsAt({ 8080: 'F', 8081: 'F', 8082: 'F' });
    await startConnect();
    expect(h.offers.length).toBe(0);
    for (const port of [8080, 8081, 8082]) expect(h.tls.block).toHaveBeenCalledWith('10.0.0.1', port);
  });

  it('tls_version_waits_for_judgement', async () => {
    const version = rest.version as unknown as ReturnType<typeof vi.fn>;
    version.mockClear();
    let release: (v: unknown) => void = () => {};
    h.tls.probe.mockImplementationOnce(() => new Promise((r) => { release = r; }));
    connection.connect('10.0.0.1');
    await flush();
    expect(version).not.toHaveBeenCalled();
    expect(h.offers.length).toBe(0);
    release({ kind: 'down' });
    await flush();
    expect(version).toHaveBeenCalledWith('10.0.0.1');
    expect(h.offers.length).toBe(1);
  });

  it('tls_remote_skips_judgement', async () => {
    remote.net = { active: () => true, pass: async () => 'ticket', exchange: () => new Promise(() => {}) } as unknown as RemoteNet;
    try {
      connection.connect('robot-id');
      await flush();
      expect(h.tls.probe).not.toHaveBeenCalled();
      connection.resecure();
      expect(h.tls.probe).not.toHaveBeenCalled();
    } finally { remote.net = undefined; }
  });

  it('tls_stale_judgement_after_remember_does_not_reblock', async () => {
    pins({ 'S@8080': 'G' });
    tlsAt({ 8080: 'F' });
    let releaseA: (v: unknown) => void = () => {};
    h.tls.probe.mockImplementationOnce(() => new Promise((r) => { releaseA = r; }));
    connection.connect('10.0.0.1');
    await flush();
    pins({ 'S@8080': 'F' });
    connection.connect('10.0.0.1');
    await flush();
    expect(h.offers.length).toBe(1);
    releaseA({ kind: 'tls', fp: 'F', serial: 'S' });
    await flush();
    expect(issues).toEqual([]);
    expect(h.tls.block).not.toHaveBeenCalledWith('10.0.0.1', 8080);
    expect(h.offers.length).toBe(1);
  });

  it('tls_rejected_judgement_blocks_all', async () => {
    h.tls.probe.mockImplementationOnce(async () => { throw new Error('native'); });
    await startConnect();
    expect(h.offers.length).toBe(0);
    for (const port of [8080, 8081, 8082]) expect(h.tls.block).toHaveBeenCalledWith('10.0.0.1', port);
  });

  it('tls_suspend_blocks_relays_and_drops_late_engage_until_resume', async () => {
    tlsAt({ 8080: 'F', 8081: 'F', 8082: 'F' });
    await connectFully();
    h.tls.base.mockImplementation((host, port) => (port === 8082 ? `http://${host}:8082` : 'http://127.0.0.1:4000'));
    let release: (v: unknown) => void = () => {};
    h.tls.probe.mockImplementationOnce(() => new Promise((r) => { release = r; }));
    connection.resecure();
    connection.suspendRelays();
    expect(h.tls.block.mock.calls).toEqual([['10.0.0.1', 8080], ['10.0.0.1', 8081]]);
    h.tls.engage.mockClear();
    release({ kind: 'tls', fp: 'F', serial: 'S' });
    await flush();
    expect(h.tls.engage).not.toHaveBeenCalled();
    connection.resecure();
    await flush();
    expect(h.tls.engage).toHaveBeenCalledWith('10.0.0.1', 8080, 'F');
    h.tls.base.mockImplementation(() => undefined);
  });

  it('tls_dropped_judgement_sends_nothing_to_that_robot', async () => {
    const version = rest.version as unknown as ReturnType<typeof vi.fn>;
    const hang = () => { let r: (v: unknown) => void = () => {}; h.tls.probe.mockImplementationOnce(() => new Promise((res) => { r = res; })); return (v: unknown) => r(v); };
    const tlsA = { kind: 'tls', fp: 'F', serial: 'S' };
    const plainAuthToA = () => h.sent.filter((x) => x.url.startsWith('http://10.0.0.1:') && x.auth);

    version.mockClear();
    let release = hang();
    connection.connect('10.0.0.1');
    const w1 = connection.secured('10.0.0.1');
    await flush();
    connection.disconnect();
    release(tlsA);
    await flush();
    expect(await w1).toBe(false);
    expect(version).not.toHaveBeenCalledWith('10.0.0.1');
    expect(h.sent.some((x) => x.url.endsWith('/api/webrtc/hangup'))).toBe(true);
    expect(plainAuthToA()).toEqual([]);

    version.mockClear();
    release = hang();
    connection.connect('10.0.0.1');
    const w2 = connection.secured('10.0.0.1');
    await flush();
    connection.connect('10.0.0.9');
    release(tlsA);
    await flush();
    expect(await w2).toBe(false);
    expect(version).not.toHaveBeenCalledWith('10.0.0.1');
    expect(plainAuthToA()).toEqual([]);
    expect(version).toHaveBeenCalledWith('10.0.0.9');
    expect(await connection.secured('10.0.0.9')).toBe(true);
    connection.disconnect();

    version.mockClear();
    release = hang();
    connection.connect('10.0.0.1');
    const w3 = connection.secured('10.0.0.1');
    await flush();
    remote.net = { active: () => true, pass: async () => 'ticket', exchange: () => new Promise(() => {}) } as unknown as RemoteNet;
    try {
      lastPc().state('failed');
      await vi.advanceTimersByTimeAsync(600);
      release(tlsA);
      await flush();
      expect(await w3).toBe(false);
      expect(version).not.toHaveBeenCalledWith('10.0.0.1');
      expect(plainAuthToA()).toEqual([]);
    } finally { remote.net = undefined; }
  });

  it('tls_engage_landing_after_suspend_is_blocked', async () => {
    await connectFully();
    tlsAt({ 8080: 'F' });
    let land: () => void = () => {};
    h.tls.engage.mockImplementationOnce(() => new Promise<void>((r) => { land = r; }));
    connection.resecure();
    await flush();
    connection.suspendRelays();
    try {
      h.tls.block.mockClear();
      land();
      await flush();
      expect(h.tls.block).toHaveBeenCalledWith('10.0.0.1', 8080);
    } finally { connection.resecure(); }
  });

  it('tls_resecure_for_only_relay_or_blocked_base', async () => {
    await connectFully();
    h.tls.probe.mockClear();
    connection.resecureFor('http://10.0.0.1:8081');
    expect(h.tls.probe).not.toHaveBeenCalled();
    connection.resecureFor('https://10.0.0.1:8081');
    await flush();
    expect(h.tls.probe.mock.calls.map(([, p]) => p)).toEqual([8080, 8081, 8082]);
    h.tls.probe.mockClear();
    connection.resecureFor('http://127.0.0.1:4000');
    await flush();
    expect(h.tls.probe).toHaveBeenCalled();
  });

  it('tls_failure_rejudge_once_per_host_per_30s', async () => {
    await connectFully();
    const judged = async (f: () => void) => { h.tls.probe.mockClear(); f(); await flush(); return h.tls.probe.mock.calls.length; };
    expect(await judged(() => connection.resecureFor('https://10.0.0.9:8082'))).toBe(3);
    vi.setSystemTime(Date.now() + 29_000);
    expect(await judged(() => connection.resecureFor('https://10.0.0.9:8081'))).toBe(0);
    expect(await judged(() => connection.resecureAfterFailure('10.0.0.9'))).toBe(0);
    expect(await judged(() => connection.resecureFor('http://127.0.0.1:4100'))).toBe(3);
    expect(await judged(() => connection.resecure())).toBe(3);
    vi.setSystemTime(Date.now() + 1_001);
    expect(await judged(() => connection.resecureFor('https://10.0.0.9:8082'))).toBe(3);
  });

  it('tls_plain_on_pinned_route_masked_only_after_connected', async () => {
    pins({ 'S@8080': 'F' });
    h.tls.kept.mockImplementation((_h, port) => port === 8080);
    h.tls.pinnedFp.mockImplementation((_h, port) => (port === 8080 ? 'F' : undefined));
    try {
      h.tls.probe.mockImplementation(async (_h, port) => (port === 8080 ? { kind: 'plain' } : { kind: 'down' }));
      await startConnect();
      expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
      expect(h.offers.length).toBe(0);
      issues.length = 0;
      tlsAt({ 8080: 'F' });
      await connectFully();
      expect(h.robot.conn).toBe('connected');
      h.tls.probe.mockImplementation(async (_h, port) => (port === 8080 ? { kind: 'plain' } : { kind: 'down' }));
      connection.resecure();
      await flush();
      expect(issues).toEqual([]);
      expect(h.robot.conn).toBe('connected');
      connection.disconnect();
      h.tls.probe.mockClear();
      await startConnect();
      expect(h.tls.probe).toHaveBeenCalled();
      expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
    } finally {
      h.tls.kept.mockImplementation(() => false);
      h.tls.pinnedFp.mockImplementation(() => undefined);
    }
  });

  it('tls_motion_reopen_after_drop_stays_live', async () => {
    pins({ 'S@8080': 'F' });
    tlsAt({ 8080: 'F' });
    await connectFully();
    h.tls.kept.mockImplementation((_h, port) => port === 8080);
    h.tls.pinnedFp.mockImplementation((_h, port) => (port === 8080 ? 'F' : undefined));
    try {
      h.tls.probe.mockImplementation(async (_h, port) => (port === 8080 ? { kind: 'plain' } : { kind: 'down' }));
      const offers = h.offers.length;
      lastPc().state('failed');
      await vi.advanceTimersByTimeAsync(500);
      expect(issues).toEqual([]);
      expect(h.offers.length).toBe(offers + 1);
      expect(h.robot.conn).toBe('connecting');
    } finally {
      h.tls.kept.mockImplementation(() => false);
      h.tls.pinnedFp.mockImplementation(() => undefined);
    }
  });

  it('tls_resume_reengages_and_refuses_if_changed', async () => {
    await connectFully();
    h.tls.probe.mockClear();
    connection.resecure();
    await flush();
    expect(h.tls.probe.mock.calls.map(([, p]) => p)).toEqual([8080, 8081, 8082]);
    expect(h.robot.conn).toBe('connected');
    pins({ 'S@8080': 'G' });
    tlsAt({ 8080: 'F' });
    connection.resecure();
    await flush();
    expect(h.robot.conn).toBe('disconnected');
    expect(issues.map((i) => i.kind)).toEqual(['changed']);
    for (const port of [8080, 8081, 8082]) expect(h.tls.block).toHaveBeenCalledWith('10.0.0.1', port);
  });
});

describe('isFetchFailure — 재판정할 fetch 실패(F3)', () => {
  it('판정', () => {
    expect(isFetchFailure(new TypeError('Network request failed'))).toBe(true);
    expect(isFetchFailure(new Error('fetch failed: javax.net.ssl.SSLHandshakeException: … Trust anchor …'))).toBe(true);
    expect(isFetchFailure(new Error('fetch failed: The operation was aborted.'))).toBe(false);
    expect(isFetchFailure(new Error('fetch failed: Fetch request has been canceled'))).toBe(false);
    expect(isFetchFailure(new Error('fetch failed: javax.net.ssl.SSLHandshakeException: SSL handshake aborted: ssl=0x7b: I/O error during system call, Connection reset by peer'))).toBe(true);
    expect(isFetchFailure(new DOMException('x', 'AbortError'))).toBe(false);
    expect(isFetchFailure({ name: 'AbortError' })).toBe(false);
    expect(isFetchFailure(new Error('offer 502'))).toBe(false);
    expect(isFetchFailure(undefined)).toBe(false);
  });
});
