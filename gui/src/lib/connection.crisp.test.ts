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
  const tel = { joyHeld: false };
  return { FakePc, pcs: [] as FakePc[], offers: [] as Offer[], hangups: [] as unknown[], sent: [] as { url: string; auth: boolean }[], robot, tls, tel };
});

vi.mock('@/store/robot', () => ({ useRobot: { getState: () => h.robot, setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({ setSensorLayout: () => {} }), setState: (p: object) => { Object.assign(h.tel, p); } }, noteMotionLink: () => {}, noteJoyUnstable: () => {} }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({ webrtcToken: '', setRobotVersion: () => {} }), subscribe: () => () => {} } }));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({ clearFeatures: () => {}, setFeatures: () => {} }) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({ show: () => {} }) } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('@/lib/simEngine', () => ({ simEngine: { active: false, intercept: () => null } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, SIZEOF: {}, CRUISE_GAIT_IDS: new Set([3, 6, 30, 42]) }));
vi.mock('./rest', () => ({
  rest: { version: vi.fn(() => new Promise(() => {})), getOwnership: vi.fn(), serialNumber: vi.fn(), robotFeatures: async () => ({}) },
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
import { setSavedSeats } from './seatMemory';
import { setRobotGuard } from './robotGuard';

const c = connection as unknown as Record<string, unknown>;
const getOwnership = rest.getOwnership as unknown as ReturnType<typeof vi.fn>;
const serialNumber = rest.serialNumber as unknown as ReturnType<typeof vi.fn>;
const saved = new Set<string>();
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
  c.lastOwnCheckAt = 0; c.lastOwnerPushAt = 0;
  h.pcs.length = 0; h.offers.length = 0; h.hangups.length = 0; h.robot.logs.length = 0;
  Object.assign(h.robot, { conn: 'disconnected', owner: '', myIp: '', isMine: false });
  getOwnership.mockReset();
  getOwnership.mockResolvedValue({ ownerIP: '', requesterIP: '10.0.0.5', IsOwner: false });
  serialNumber.mockReset();
  serialNumber.mockResolvedValue({ serial_number: 'SN1' });
  saved.clear();
  h.tel.joyHeld = false;
  connection.clearInputs();
  setSavedSeats({ has: (k) => saved.has(k), set: (k, on) => { if (on) saved.add(k); else saved.delete(k); } });
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

describe('처음 붙는 앱은 관전 — 쥐고 있던 기억이 없으면 자동 claim 없음', () => {
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

  it('★첫 소유권 조회가 실패하면 push 가 와도 다시 물어 내 IP 를 배운다 — 조종 중인 기기가 관전으로 남지 않게', async () => {
    c.lastOwnCheckAt = 0;
    getOwnership.mockRejectedValueOnce(new Error('timeout'));
    getOwnership.mockResolvedValue({ ownerIP: '10.0.0.5', requesterIP: '10.0.0.5', IsOwner: true });
    await connectFully();
    expect(h.robot.myIp).toBe('');
    for (let i = 0; i < 3; i++) {
      lastPc().dcs.get('log')!.onmessage?.({ data: JSON.stringify({ t: 'gamepad_status', owner: '10.0.0.5' }) });
      await vi.advanceTimersByTimeAsync(1000);
    }
    expect(h.robot).toMatchObject({ myIp: '10.0.0.5', isMine: true });
    const n = getOwnership.mock.calls.length;
    for (let i = 0; i < 3; i++) {
      lastPc().dcs.get('log')!.onmessage?.({ data: JSON.stringify({ t: 'gamepad_status', owner: '10.0.0.5' }) });
      await vi.advanceTimersByTimeAsync(1000);
    }
    expect(getOwnership.mock.calls.length).toBe(n);
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

function answerCmd(method: string, path: string, body: object, status = 200) {
  for (const raw of cmdDc().sent) {
    let m: { id?: string; method?: string; path?: string } = {};
    try { m = JSON.parse(raw); } catch { continue; }
    if (m.method === method && m.path === path) cmdDc().onmessage?.({ data: JSON.stringify({ id: m.id, status, body }) });
  }
}
const MINE = { ownerIP: '10.0.0.5', requesterIP: '10.0.0.5', IsOwner: true };
async function dropAndReconnect() {
  lastPc().state('failed');
  await vi.advanceTimersByTimeAsync(500);
  h.offers[h.offers.length - 1].answer();
  await flush();
  lastPc().state('connected');
  cmdDc().open();
  await flush();
}

describe('쥐고 있던 자리는 다시 붙으면 되찾는다(11-16)', () => {
  it('제어권을 가져온 로봇에 다시 붙어 자리가 비어 있으면 묻지 않고 한 번 claim 한다', async () => {
    await connectFully();
    expect(h.robot.isMine).toBe(false);
    const took = connection.claimOwnership();
    await flush();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    expect(await took).toBe(true);
    expect([...saved]).toEqual(['SN1']);
    const claim = vi.spyOn(connection, 'claimOwnership');
    await dropAndReconnect();
    expect(claim).toHaveBeenCalledTimes(1);
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    getOwnership.mockResolvedValue(MINE);
    await flush();
    expect(h.robot).toMatchObject({ owner: '10.0.0.5', isMine: true });
    await vi.advanceTimersByTimeAsync(5000);
    expect(claim).toHaveBeenCalledTimes(1);
    claim.mockRestore();
  });

  it('같은 주소(.10)에 고른 로봇이 아닌 다른 로봇이 답하면 끊고 알린다 — 그 로봇 자리를 되찾지도 않는다(11-17)', async () => {
    saved.add('SN1');
    const mismatch = vi.fn();
    setRobotGuard({ expected: () => 'SN0', mismatch });
    const claim = vi.spyOn(connection, 'claimOwnership');
    try {
      await connectFully();
      await vi.advanceTimersByTimeAsync(4000);
      expect(mismatch).toHaveBeenCalledWith('SN1');
      expect(claim).not.toHaveBeenCalled();
    } finally {
      setRobotGuard({ expected: () => null, mismatch: () => {} });
      claim.mockRestore();
    }
  });

  it('남이 쥐고 있으면 되찾지 않고 관전 — 기억도 지운다', async () => {
    saved.add('SN1');
    getOwnership.mockResolvedValue({ ownerIP: '10.0.0.7', requesterIP: '10.0.0.5', IsOwner: false });
    const claim = vi.spyOn(connection, 'claimOwnership');
    await connectFully();
    await vi.advanceTimersByTimeAsync(4000);
    expect(claim).not.toHaveBeenCalled();
    expect(h.robot).toMatchObject({ owner: '10.0.0.7', isMine: false });
    expect(saved.has('SN1')).toBe(false);
    claim.mockRestore();
  });

  it('조종 중에 남이 가져가는 걸 보면 기억을 지운다 — 다시 붙어 빈 자리여도 관전', async () => {
    saved.add('SN1');
    await connectFully();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    await flush();
    expect(h.robot.isMine).toBe(true);
    lastPc().dcs.get('log')!.onmessage?.({ data: JSON.stringify({ t: 'gamepad_status', owner: '10.0.0.7' }) });
    await flush();
    expect(saved.has('SN1')).toBe(false);
    const claim = vi.spyOn(connection, 'claimOwnership');
    await dropAndReconnect();
    await vi.advanceTimersByTimeAsync(4000);
    expect(claim).not.toHaveBeenCalled();
    expect(h.robot.isMine).toBe(false);
    claim.mockRestore();
  });

  it('내 주소(myIp)를 모를 때 온 push 는 기억을 지우지 않는다 — 내 옛 세션이 아직 쥔 것일 수 있다', async () => {
    saved.add('SN1');
    getOwnership.mockImplementation(() => new Promise(() => {}));
    await connectFully();
    lastPc().dcs.get('log')!.onmessage?.({ data: JSON.stringify({ t: 'gamepad_status', owner: '10.0.0.5' }) });
    await flush();
    expect(saved.has('SN1')).toBe(true);
  });

  it('[연결 끊기](forgetSeat) 뒤에는 되찾지 않는다', async () => {
    saved.add('SN1');
    await connectFully();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    await flush();
    connection.forgetSeat();
    connection.disconnect();
    await flush();
    expect(saved.has('SN1')).toBe(false);
    const claim = vi.spyOn(connection, 'claimOwnership');
    await connectFully();
    await vi.advanceTimersByTimeAsync(4000);
    expect(claim).not.toHaveBeenCalled();
    claim.mockRestore();
  });

  it('command DC 가 3 s 안에 안 열리면 되찾지 않는다 — 늦은 claim 은 사람이 모르는 사이 자리를 잡는다', async () => {
    saved.add('SN1');
    const claim = vi.spyOn(connection, 'claimOwnership');
    await startConnect();
    h.offers[0].answer();
    await flush();
    lastPc().state('connected');
    await vi.advanceTimersByTimeAsync(3500);
    cmdDc().open();
    await vi.advanceTimersByTimeAsync(3000);
    expect(claim).not.toHaveBeenCalled();
    expect(h.robot.myIp).toBe('10.0.0.5');
    claim.mockRestore();
  });

  it('시리얼·채널을 기다린 사이 남이 잡으면 되찾지 않는다 — claim 직전에 한 번 더 묻는다', async () => {
    saved.add('SN1');
    const claim = vi.spyOn(connection, 'claimOwnership');
    await startConnect();
    h.offers[0].answer();
    await flush();
    lastPc().state('connected');
    await flush();
    getOwnership.mockResolvedValue({ ownerIP: '10.0.0.7', requesterIP: '10.0.0.5', IsOwner: false });
    await vi.advanceTimersByTimeAsync(1000);
    cmdDc().open();
    await vi.advanceTimersByTimeAsync(500);
    expect(claim).not.toHaveBeenCalled();
    expect(h.robot).toMatchObject({ owner: '10.0.0.7', isMine: false });
    expect(saved.has('SN1')).toBe(false);
    claim.mockRestore();
  });

  it('첫 소유권 조회가 실패하는 동안 남이 쥐는 걸 보면, 그 뒤 빈 자리가 돼도 되찾지 않는다', async () => {
    saved.add('SN1');
    getOwnership.mockRejectedValue(new Error('down'));
    const claim = vi.spyOn(connection, 'claimOwnership');
    await connectFully();
    const push = (owner: string) => lastPc().dcs.get('log')!.onmessage?.({ data: JSON.stringify({ t: 'gamepad_status', owner }) });
    push('10.0.0.7');
    await vi.advanceTimersByTimeAsync(1500);
    push('');
    getOwnership.mockResolvedValue({ ownerIP: '', requesterIP: '10.0.0.5', IsOwner: false });
    await vi.advanceTimersByTimeAsync(4000);
    expect(claim).not.toHaveBeenCalled();
    claim.mockRestore();
  });

  it('시리얼 조회가 실패하면 기억도 되찾기도 하지 않는다 — 실기에서 주소로 가리지 않는다', async () => {
    serialNumber.mockRejectedValue(new Error('down'));
    await connectFully();
    const took = connection.claimOwnership();
    await flush();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    expect(await took).toBe(true);
    expect(saved.size).toBe(0);
    const claim = vi.spyOn(connection, 'claimOwnership');
    await dropAndReconnect();
    await vi.advanceTimersByTimeAsync(4000);
    expect(claim).not.toHaveBeenCalled();
    claim.mockRestore();
  });

  it('시리얼이 없는 로봇(시뮬)은 이 실행 동안만 기억한다 — 저장 기억은 건드리지 않는다', async () => {
    serialNumber.mockResolvedValue({ serial_number: '' });
    await connectFully();
    const took = connection.claimOwnership();
    await flush();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    await took;
    expect(saved.size).toBe(0);
    const claim = vi.spyOn(connection, 'claimOwnership');
    await dropAndReconnect();
    expect(claim).toHaveBeenCalledTimes(1);
    connection.forgetSeat();
    claim.mockRestore();
  });

  it('원격(랑데부)도 같다 — 소유권·시리얼을 command DC 로 묻고 빈 자리면 되찾는다', async () => {
    remote.net = { active: () => true, pass: async () => 'ticket', exchange: async () => ({ sdp: 'answer-sdp', features: [] }) } as unknown as RemoteNet;
    try {
      saved.add('SN1');
      connection.connect('robot-id');
      await flush();
      lastPc().state('connected');
      cmdDc().open();
      await flush();
      expect(getOwnership).not.toHaveBeenCalled();
      answerCmd('GET', '/api/robot/serial_number', { serial_number: 'SN1' });
      answerCmd('GET', '/api/gamepad/ownership', { ownerIP: '', requesterIP: 'rbq-app-test', IsOwner: false });
      await flush();
      answerCmd('GET', '/api/gamepad/ownership', { ownerIP: '', requesterIP: 'rbq-app-test', IsOwner: false });
      await flush();
      const posts = cmdDc().sent.filter((m) => m.includes('"POST"') && m.includes('/api/gamepad/ownership'));
      expect(posts.length).toBe(1);
    } finally { remote.net = undefined; }
  });
});

describe('되찾은 직후 쥔 스틱은 가운데로 놓을 때까지 0(11-16)', () => {
  const motionDc = () => lastPc().dcs.get('motion-state')!;
  const stick = (lx: number, ly: number, rx = 0, ry = 0) => { connection.setAxes('L', lx, ly); connection.setAxes('R', rx, ry); };
  const frames = () => (motionDc().sent as unknown as ArrayBuffer[]).map((b) => {
    const v = new DataView(b);
    return { axes: [0, 4, 8, 12].map((o) => Math.round(v.getFloat32(o, true) * 100) / 100), btn0: new Uint8Array(b, 24, 1)[0] };
  });

  it('가상 스틱을 쥔 채 되찾으면 claim 답 전부터 0 — 가운데로 놓은 뒤엔 값이 나간다', async () => {
    saved.add('SN1');
    await connectFully();
    stick(0, 0.8);
    motionDc().open();
    await vi.advanceTimersByTimeAsync(100);
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    await vi.advanceTimersByTimeAsync(100);
    expect(h.robot.isMine).toBe(true);
    expect(h.tel.joyHeld).toBe(true);
    expect(frames().length).toBeGreaterThan(0);
    expect(frames().every((f) => f.axes.every((a) => a === 0))).toBe(true);
    stick(0, 0);
    expect(h.tel.joyHeld).toBe(false);
    motionDc().sent.length = 0;
    stick(0, 0.5);
    await vi.advanceTimersByTimeAsync(100);
    expect(frames().at(-1)!.axes).toEqual([0, 0.5, 0, 0]);
  });

  it('게임패드(오른 스틱 + 버튼)도 같다 — 막는 동안 스틱만 0, 버튼은 그대로 나간다', async () => {
    saved.add('SN1');
    await connectFully();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    await flush();
    const b = new Uint8Array(16); b[0] = 1;
    connection.setButtons(b);
    stick(0, 0, 0.6, 0);
    motionDc().open();
    await vi.advanceTimersByTimeAsync(100);
    expect(h.tel.joyHeld).toBe(true);
    expect(frames().length).toBeGreaterThan(0);
    expect(frames().every((f) => f.axes.every((a) => a === 0) && f.btn0 === 1)).toBe(true);
    stick(0, 0);
    motionDc().sent.length = 0;
    stick(0, 0, 0.6, 0);
    await vi.advanceTimersByTimeAsync(100);
    expect(frames().at(-1)!.axes).toEqual([0, 0, 0.6, 0]);
  });

  it('스틱마다 따로 풀린다 — 만지는 스틱만 보고하는 화면도 그 스틱을 놓았다 밀면 나간다', async () => {
    saved.add('SN1');
    await connectFully();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    await flush();
    connection.setAxes('L', 0, 0.7);
    motionDc().open();
    await vi.advanceTimersByTimeAsync(100);
    expect(h.tel.joyHeld).toBe(true);
    expect(frames().every((f) => f.axes.every((a) => a === 0))).toBe(true);
    connection.setAxes('L', 0, 0);
    motionDc().sent.length = 0;
    connection.setAxes('L', 0, 0.5);
    await vi.advanceTimersByTimeAsync(100);
    expect(h.tel.joyHeld).toBe(false);
    expect(frames().at(-1)!.axes).toEqual([0, 0.5, 0, 0]);
    connection.setAxes('R', 0.6, 0);
    motionDc().sent.length = 0;
    await vi.advanceTimersByTimeAsync(100);
    expect(h.tel.joyHeld).toBe(true);
    expect(frames().at(-1)!.axes).toEqual([0, 0.5, 0, 0]);
  });

  it('놓고 있던 사람은 막히지 않는다 — 밀기 시작할 때 데드존 안 값(0)이 먼저 와 풀린다 · 안내도 없다', async () => {
    saved.add('SN1');
    await connectFully();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    await flush();
    expect(h.tel.joyHeld).toBe(false);
    stick(0, 0);
    stick(0, 0.4);
    motionDc().open();
    await vi.advanceTimersByTimeAsync(100);
    expect(frames().at(-1)!.axes).toEqual([0, 0.4, 0, 0]);
  });

  it('사람이 [제어권 가져오기]를 누른 것은 막지 않는다', async () => {
    await connectFully();
    stick(0, 0.8);
    motionDc().open();
    const took = connection.claimOwnership();
    await flush();
    answerCmd('POST', '/api/gamepad/ownership', MINE);
    await took;
    motionDc().sent.length = 0;
    await vi.advanceTimersByTimeAsync(100);
    expect(h.tel.joyHeld).toBe(false);
    expect(frames().at(-1)!.axes).toEqual([0, 0.8, 0, 0]);
  });

  it('되찾기 claim 이 실패하면 막기도 푼다', async () => {
    saved.add('SN1');
    await connectFully();
    answerCmd('POST', '/api/gamepad/ownership', {}, 500);
    await flush();
    stick(0, 0.4);
    motionDc().open();
    await vi.advanceTimersByTimeAsync(100);
    expect(h.tel.joyHeld).toBe(false);
    expect(frames().at(-1)!.axes).toEqual([0, 0.4, 0, 0]);
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
