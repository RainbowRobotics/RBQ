import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ owner: null, myIp: null }), setState: () => {}, subscribe: () => () => {} } }));
const warn = vi.hoisted(() => ({ n: 0 }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { getState: () => ({}), setState: () => {} }, noteMotionLink: () => {}, noteJoyUnstable: () => { warn.n++; } }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({}), subscribe: () => () => {} } }));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({}) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({}) } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('@/lib/simEngine', () => ({ simEngine: { active: false } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, SIZEOF: {}, CRUISE_GAIT_IDS: new Set([3, 6, 30, 42]) }));
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
vi.mock('./robotTlsPlatform', () => ({ robotTlsPlatform: { probe: async () => ({ kind: 'down' }), engage: async () => {}, block: () => {}, kept: () => false, base: () => undefined } }));

import { connection } from './connection';
import { ID_JOY_LINK, JOY_LINK_STATS_BYTES, JoyLinkHealth, withJoyTail, parseJoyLinkStats, type JoyLinkStats } from './joyLink';

type Sent = { len: number; ly: number; seq?: number; sentMs?: number };
let sent: Sent[] = [];
let buffered = 0;

function linkFrame(stats: number[] = [0, 0, 0, 0, 0, 0]): ArrayBuffer {
  const buf = new ArrayBuffer(3 + JOY_LINK_STATS_BYTES);
  const u = new Uint8Array(buf);
  u[0] = 0x4d; u[1] = 0x53; u[2] = ID_JOY_LINK;
  const dv = new DataView(buf);
  stats.forEach((v, i) => (i < 5 ? dv.setUint32(3 + i * 4, v, true) : dv.setInt32(3 + i * 4, v, true)));
  return buf;
}
const hello = (s?: number[]) => (connection as unknown as { onMotionState(d: ArrayBuffer): void }).onMotionState(linkFrame(s));

beforeEach(() => {
  vi.useFakeTimers();
  sent = [];
  buffered = 0;
  const c = connection as unknown as Record<string, unknown>;
  c.lastOwnerPushAt = Date.now();
  c.lastOwnCheckAt = Date.now();
  c.joyV2 = false;
  c.joyLink = null;
  c.joyHealth = new JoyLinkHealth();
  c.lastStateAt = 0;
  warn.n = 0;
  c.motionDc = {
    readyState: 'open',
    get bufferedAmount() { return buffered; },
    send: (buf: ArrayBuffer) => {
      const dv = new DataView(buf);
      sent.push({ len: buf.byteLength, ly: dv.getFloat32(4, true),
        seq: buf.byteLength === 48 ? dv.getUint32(40, true) : undefined,
        sentMs: buf.byteLength === 48 ? dv.getUint32(44, true) : undefined });
    },
  };
});

afterEach(() => {
  connection.setAxes('L', 0, 0);
  connection.setAxes('R', 0, 0);
  vi.advanceTimersByTime(2000);
  vi.useRealTimers();
});

describe('조이스틱 링크 — 48 B 프레임', () => {
  it('링크 표시를 받기 전엔 40 B 만 보낸다(옛 로봇)', () => {
    connection.setAxes('L', 0, 0.5);
    vi.advanceTimersByTime(200);
    expect(sent.length).toBeGreaterThan(5);
    expect(sent.every((f) => f.len === 40)).toBe(true);
  });

  it('링크 표시를 받으면 0 프레임 3 장으로 기준을 잡고, 그 뒤 48 B 에 1 씩 느는 순서 번호', () => {
    hello();
    expect(sent.length).toBe(3);
    expect(sent.every((f) => f.len === 48 && f.ly === 0)).toBe(true);
    connection.setAxes('L', 0, 0.5);
    vi.advanceTimersByTime(250);
    const seqs = sent.map((f) => f.seq!);
    expect(sent.every((f) => f.len === 48)).toBe(true);
    expect(seqs.every((s, i) => i === 0 || s === seqs[i - 1] + 1)).toBe(true);
    expect(sent.slice(3).every((f) => f.ly === Math.fround(0.5))).toBe(true);
  });

  it('스틱을 밀고 있을 때 링크 표시가 오면 씨앗 0 을 보내지 않는다(조종이 멈칫하지 않게)', () => {
    connection.setAxes('L', 0, 0.5);
    vi.advanceTimersByTime(50);
    const before = sent.length;
    hello();
    expect(sent.length).toBe(before);
    vi.advanceTimersByTime(100);
    expect(sent.slice(before).every((f) => f.len === 48 && f.ly !== 0)).toBe(true);
  });

  it('로봇 통계를 읽어 둔다', () => {
    hello([200, 180, 12, 3, 5, 40]);
    expect((connection as unknown as { joyLink: unknown }).joyLink).toEqual({ received: 200, accepted: 180, lost: 12, reordered: 3, late: 5, delayMs: 40 });
  });

  it('송신 큐가 밀려 있으면(프레임 3 장 넘게) 쌓지 않고 버린다 — 풀리는 순간 옛 조종이 쏟아지지 않게', () => {
    hello();
    sent = [];
    buffered = 400;
    connection.setAxes('L', 0, 0.5);
    vi.advanceTimersByTime(200);
    expect(sent.length).toBe(0);
    buffered = 0;
    vi.advanceTimersByTime(50);
    expect(sent.length).toBeGreaterThan(0);
  });

  it('보낸 시각은 단조 ms — 프레임마다 줄지 않는다', () => {
    hello();
    connection.setAxes('L', 0, 0.5);
    vi.advanceTimersByTime(300);
    const t = sent.map((f) => f.sentMs!);
    expect(t.every((v, i) => i === 0 || v >= t[i - 1])).toBe(true);
  });
});

describe('joyLink 인코딩', () => {
  it('40 B 를 그대로 두고 꼬리 8 B 를 붙인다', () => {
    const f = new ArrayBuffer(40);
    new DataView(f).setFloat32(4, 0.25, true);
    const out = withJoyTail(f, 0xFFFFFFFF + 3, 123);
    const dv = new DataView(out);
    expect(out.byteLength).toBe(48);
    expect(dv.getFloat32(4, true)).toBe(0.25);
    expect(dv.getUint32(40, true)).toBe(2);
    expect(dv.getUint32(44, true)).toBe(123);
  });
  it('통계 24 B 를 로봇 배치대로 읽는다(delayMs 는 부호 있음)', () => {
    const dv = new DataView(linkFrame([1, 2, 3, 4, 5, -7]));
    expect(parseJoyLinkStats(dv, 3)).toEqual({ received: 1, accepted: 2, lost: 3, reordered: 4, late: 5, delayMs: -7 });
  });
});

describe('조종 연결 불안정 경고 — 전송 루프', () => {
  it('미는 동안 송신 큐가 0.3 s 막히면 경고, 그 안에 풀리면 안 한다', () => {
    buffered = 400;
    connection.setAxes('L', 0, 0.5);
    vi.advanceTimersByTime(250);
    expect(warn.n).toBe(0);
    vi.advanceTimersByTime(100);
    expect(warn.n).toBeGreaterThan(0);
    buffered = 0;
  });

  it('새 로봇이 통계를 1 s 넘게 안 보내면 경고 — 밀고 있을 때만', () => {
    hello();
    vi.advanceTimersByTime(2000);
    expect(warn.n).toBe(0);
    connection.setAxes('L', 0, 0.5);
    vi.advanceTimersByTime(900);
    expect(warn.n).toBe(0);
    vi.advanceTimersByTime(200);
    expect(warn.n).toBeGreaterThan(0);
  });

  it('통계가 0.5 s 마다 오면 경고하지 않는다', () => {
    hello();
    connection.setAxes('L', 0, 0.5);
    for (let i = 1; i <= 6; i++) { vi.advanceTimersByTime(500); hello([i * 20, i * 20, 0, 0, 0, 0]); }
    expect(warn.n).toBe(0);
  });

  it('옛 로봇(통계 없음)은 큐가 막힐 때만 경고한다', () => {
    connection.setAxes('L', 0, 0.5);
    vi.advanceTimersByTime(3000);
    expect(warn.n).toBe(0);
  });
});

describe('JoyLinkHealth — 로봇 통계 비율', () => {
  const st = (received: number, late = 0, lost = 0, reordered = 0): JoyLinkStats => ({ received, accepted: received - late - reordered, lost, reordered, late, delayMs: 0 });

  it('최근 2 s 동안 버림·유실이 10 % 이상이면 true', () => {
    const h = new JoyLinkHealth();
    expect(h.stats(0, st(0))).toBe(false);
    expect(h.stats(500, st(20))).toBe(false);
    expect(h.stats(1000, st(40, 2))).toBe(false);
    expect(h.stats(1500, st(60, 5, 2))).toBe(true);
  });

  it('2 s 가 지나면 옛 버림은 창 밖으로 나간다', () => {
    const h = new JoyLinkHealth();
    h.stats(0, st(0));
    h.stats(500, st(20, 10));
    for (let t = 1000; t <= 3000; t += 500) h.stats(t, st(20 + (t - 500) / 25, 10));
    expect(h.stats(3500, st(140, 10))).toBe(false);
  });

  it('10 장 미만은 판정하지 않는다', () => {
    const h = new JoyLinkHealth();
    h.stats(0, st(0));
    expect(h.stats(500, st(5, 3))).toBe(false);
  });
});
