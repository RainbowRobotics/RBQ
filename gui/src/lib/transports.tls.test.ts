import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const h = vi.hoisted(() => {
  class FakeDc { readyState = 'connecting'; binaryType = ''; onopen = null; onmessage = null; onclose = null; bufferedAmount = 0; send() {} }
  class FakePc {
    iceGatheringState = 'complete';
    localDescription = { sdp: 'v=0' };
    connectionState = 'new';
    signalingState = 'stable';
    onconnectionstatechange: (() => void) | null = null;
    addEventListener() {}
    removeEventListener() {}
    addTransceiver() { return { sender: { track: null } }; }
    createDataChannel() { return new FakeDc(); }
    async createOffer() { return { type: 'offer', sdp: 'v=0' }; }
    async setLocalDescription() {}
    async setRemoteDescription() {}
    close() {}
  }
  const robot = {
    conn: 'disconnected', ip: '', visionIp: '', owner: '', myIp: '10.0.0.5', isMine: true,
    setIp(ip: string) { robot.ip = ip; }, setConnError() {}, setVia() {}, setRoute() {}, applyHealth() {}, setOwnerSupport() {},
    setConn(c: string) { robot.conn = c; }, setOwner() {}, setOwnership() {}, pushLog() {}, setTakeoverConflict() {},
  };
  const native = { probe: vi.fn(), relay: vi.fn(async (_h: string, port: number) => (port === 8080 ? 41234 : 41235)), close: vi.fn() };
  return { FakePc, robot, native, sent: [] as { url: string; auth: boolean }[] };
});

vi.mock('../../modules/robot-tls/src/RobotTlsModule', () => ({ default: h.native }));
vi.mock('react-native', () => ({ Platform: { OS: 'android' }, PermissionsAndroid: { check: async () => false, PERMISSIONS: {} } }));
vi.mock('react-native-webrtc', () => ({ RTCPeerConnection: h.FakePc, RTCSessionDescription: class {}, mediaDevices: {} }));
vi.mock('./rtcPeer', () => ({ RTCPeerConnection: h.FakePc, RTCSessionDescription: class {} }));
vi.mock('expo-device', () => ({ isDevice: true }));
vi.mock('./demoFlag', () => ({ isDemo: () => false, demoDriver: () => undefined }));
vi.mock('./auth', () => ({ robotAuth: () => ({ Authorization: 'Basic test' }), reportAuthFailure: () => true }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => h.robot, setState: () => {}, subscribe: () => () => {} } }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({ webrtcToken: '', audio: { robotSpk: 0, robotMic: 0 }, setRobotVersion: () => {} }), subscribe: () => () => {} } }));
vi.mock('@/store/telemetry', () => ({
  useTelemetry: { getState: () => ({ setSensorLayout: () => {} }), setState: () => {}, subscribe: () => () => {} },
  noteMotionLink: () => {}, noteJoyUnstable: () => {}, noteVisionLink: () => {},
}));
vi.mock('@/store/capability', () => ({ useFeatures: { getState: () => ({ clearFeatures: () => {}, setFeatures: () => {} }) } }));
vi.mock('@/store/dockView', () => ({ useDockView: { getState: () => ({ show: () => {} }) } }));
vi.mock('@/store/viewport', () => ({ useViewport: {} }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({ setDockOverride: () => {} }) } }));
vi.mock('./simEngine', () => ({ simEngine: { active: false, intercept: () => null } }));
vi.mock('./robotState', () => ({ parseRobotState: () => null, parseDeviceStates: () => null, parsePduState: () => null, SIZEOF: {}, CRUISE_GAIT_IDS: new Set() }));
vi.mock('./rest', () => ({
  rest: { version: vi.fn(() => new Promise(() => {})), getOwnership: vi.fn(async () => ({})), robotFeatures: async () => ({}) },
  actions: {}, HttpError: class extends Error {},
}));
vi.mock('./commandBus', () => ({ setCommandSender: () => {}, setStreamerCommandSender: () => {}, setVisionRequestSender: () => {} }));
vi.mock('./connectionRoute', () => ({ routeDetail: () => ({}), readStats: () => [], inboundVideoFromStats: () => null }));
vi.mock('./robotTunnel', () => ({ attachTunnel: () => {}, attachPortTunnel: () => {} }));
vi.mock('./installId', () => ({ installId: () => 'test', installIdReady: Promise.resolve() }));
vi.mock('./gait', () => ({ gait: {}, RL_GAIT: {} }));
vi.mock('./remoteIce', () => ({ modeField: () => ({}), resolveIceServers: async () => [], routeKey: () => 'lan' }));
vi.mock('./spectating', () => ({ waitRoleKnown: async () => {} }));
vi.mock('./pointcloud', () => ({ attachPointcloudChannel: () => {}, resetAllClouds: () => {} }));
vi.mock('./elevationMap', () => ({ attachElevationChannel: () => {} }));
vi.mock('./wallMap', () => ({ attachWallmapChannel: () => {} }));
vi.mock('./visionStateDc', () => ({ handleVisionStateFrame: () => {} }));
vi.mock('./videoKeyframe', () => ({ setKeyframeProbe: () => {} }));
vi.mock('./iosAudioSession', () => ({ iosAudioSession: { arm: () => {}, setEnabled: () => {} } }));
vi.mock('./videoStale', () => ({ armVideoStaleClear: () => {}, cancelVideoStaleClear: () => {} }));

import { connectAll, disconnectAll, resumeCurrent } from './transports';
import { restBase, streamerBase } from './endpoints';
import { robotTlsPlatform } from './robotTlsPlatform';
import { webrtcClient } from './webrtcClient';
import { setTlsPinSource, setTlsIssueHandler, type TlsIssue } from './robotTls';

const R = '192.168.0.10';
const flush = async () => { for (let i = 0; i < 5; i++) await vi.advanceTimersByTimeAsync(0); };
const plainAuth = () => h.sent.filter((x) => x.url.startsWith(`http://${R}:`) && x.auth);
const hangups = () => h.sent.filter((x) => x.url.endsWith('/api/webrtc/hangup')).map((x) => x.url);
const noPins = () => ({ selectedKey: null as string | null, selectedSsid: null as string | null, pins: {} as Record<string, string>, keyFor: async (s: string) => s, currentSsid: async (): Promise<string | null> => null });
const issues: TlsIssue[] = [];

async function connectX(probe: object) {
  h.native.probe.mockImplementation(async () => probe);
  connectAll(R);
  await flush();
  expect(h.sent.filter((x) => x.url.endsWith('/api/webrtc/offer')).length).toBe(2);
  h.sent.length = 0;
}

beforeEach(() => {
  vi.useFakeTimers();
  h.sent.length = 0;
  h.native.close.mockClear();
  issues.length = 0;
  setTlsPinSource(noPins);
  setTlsIssueHandler((i) => issues.push(i));
  vi.stubGlobal('fetch', vi.fn((url: string, init?: { headers?: Record<string, string> }) => {
    h.sent.push({ url, auth: !!init?.headers?.Authorization });
    if (url.startsWith('https://')) return Promise.reject(new TypeError('Network request failed'));
    if (url.endsWith('/api/webrtc/hangup')) return Promise.resolve({ ok: true, json: async () => ({}) });
    return new Promise(() => {});
  }));
});
afterEach(() => { disconnectAll(); vi.unstubAllGlobals(); vi.useRealTimers(); });

describe('끊기·로봇 바꾸기 — 옛 로봇의 평문 경로로 비번이 안 나간다(R16)', () => {
  it('switch_from_plain_robot_sends_no_plain_auth_before_new_judgement', async () => {
    await connectX({ kind: 'plain' });
    expect(restBase(R)).toBe(`http://${R}:8080`);
    h.native.probe.mockImplementation(() => new Promise(() => {}));
    disconnectAll();
    connectAll(R);
    await flush();
    expect(hangups()).toEqual([`https://${R}:8080/api/webrtc/hangup`, `https://${R}:8081/api/webrtc/hangup`, `https://${R}:8081/api/webrtc/hangup`]);
    expect(plainAuth()).toEqual([]);
    expect(restBase(R)).toBe(`https://${R}:8080`);
    expect(streamerBase(R)).toBe(`https://${R}:8081`);
  });

  it('disconnect_from_plain_robot_sends_no_plain_auth', async () => {
    await connectX({ kind: 'plain' });
    disconnectAll();
    expect(hangups()).toEqual([`https://${R}:8080/api/webrtc/hangup`, `https://${R}:8081/api/webrtc/hangup`]);
    expect(plainAuth()).toEqual([]);
  });

  it('relays_survive_teardown', async () => {
    await connectX({ kind: 'tls', fp: 'F', serial: 'X' });
    disconnectAll();
    expect(restBase(R)).toBe('http://127.0.0.1:41234');
    expect(streamerBase(R)).toBe('http://127.0.0.1:41235');
    expect(hangups()).toEqual(['http://127.0.0.1:41234/api/webrtc/hangup', 'http://127.0.0.1:41235/api/webrtc/hangup']);
    expect(h.native.close).not.toHaveBeenCalled();
  });
});

describe('바꾸기 잠금 — 떠나는 로봇의 재판정은 닫기만 한다(R18)', () => {
  function startSwitch(src: Partial<ReturnType<typeof noPins>>) {
    setTlsPinSource(() => ({ ...noPins(), selectedKey: 'Y', pins: { 'Y@8080': 'F' }, ...src }));
    robotTlsPlatform.forgetPlain();
    h.native.probe.mockClear();
  }

  it('outgoing_rejudge_during_switch_cannot_reopen_plain', async () => {
    await connectX({ kind: 'plain' });
    startSwitch({ selectedSsid: 'RBQ_Y', currentSsid: async () => 'RBQ_X' });
    webrtcClient.disconnect(); webrtcClient.connect(R);
    await flush();
    await vi.advanceTimersByTimeAsync(1100); await flush();
    expect(h.native.probe).toHaveBeenCalled();
    expect(restBase(R)).toBe(`https://${R}:8080`);
    expect(streamerBase(R)).toBe(`https://${R}:8081`);
    expect(plainAuth()).toEqual([]);
    expect(issues).toEqual([]);
  });

  it('outgoing_rejudge_during_switch_raises_no_issue_and_keeps_session', async () => {
    await connectX({ kind: 'plain' });
    startSwitch({});
    webrtcClient.disconnect(); webrtcClient.connect(R);
    await flush();
    expect(h.native.probe).toHaveBeenCalled();
    expect(issues).toEqual([]);
    expect(h.robot.conn).not.toBe('disconnected');
    expect(restBase(R)).toBe(`https://${R}:8080`);
    expect(plainAuth()).toEqual([]);
  });

  it('connect_after_switch_releases_latch', async () => {
    await connectX({ kind: 'plain' });
    startSwitch({});
    setTlsPinSource(noPins);
    disconnectAll();
    connectAll(R);
    await flush();
    expect(restBase(R)).toBe(`http://${R}:8080`);
  });

  it('switch_revert_rejudges_x_back_to_plain', async () => {
    await connectX({ kind: 'plain' });
    startSwitch({});
    setTlsPinSource(noPins);
    resumeCurrent();
    await flush();
    expect(restBase(R)).toBe(`http://${R}:8080`);
    expect(streamerBase(R)).toBe(`http://${R}:8081`);
    expect(h.robot.conn).not.toBe('disconnected');
  });
});
