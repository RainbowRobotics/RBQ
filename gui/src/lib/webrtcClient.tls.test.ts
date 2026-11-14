import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const h = vi.hoisted(() => {
  class FakePc {
    iceGatheringState = 'complete';
    localDescription = { sdp: 'v=0' };
    connectionState = 'new';
    signalingState = 'stable';
    addEventListener() {}
    removeEventListener() {}
    addTransceiver() { return { sender: { track: null } }; }
    createDataChannel() { return {}; }
    async createOffer() { return { sdp: 'v=0' }; }
    async setLocalDescription() {}
    close() {}
  }
  return { FakePc, remote: false, secured: true, sent: [] as { url: string; auth: boolean }[] };
});
vi.mock('react-native', () => ({ Platform: { OS: 'android' }, PermissionsAndroid: { check: async () => false, PERMISSIONS: {} } }));
vi.mock('react-native-webrtc', () => ({ RTCPeerConnection: h.FakePc, RTCSessionDescription: class {}, mediaDevices: {} }));
vi.mock('expo-device', () => ({ isDevice: true }));
vi.mock('@/lib/demoFlag', () => ({ isDemo: () => false }));
vi.mock('@/lib/auth', () => ({ robotAuth: () => ({ Authorization: 'Basic test' }) }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({ webrtcToken: '', audio: { robotSpk: 0, robotMic: 0 } }), subscribe: () => () => {} } }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ isMine: true, myIp: '10.0.0.5', visionIp: '' }), subscribe: () => () => {} } }));
vi.mock('@/lib/spectating', () => ({ waitRoleKnown: async () => {} }));
vi.mock('@/lib/pointcloud', () => ({ attachPointcloudChannel: () => {}, resetAllClouds: () => {} }));
vi.mock('@/lib/elevationMap', () => ({ attachElevationChannel: () => {} }));
vi.mock('@/lib/wallMap', () => ({ attachWallmapChannel: () => {} }));
vi.mock('@/lib/commandBus', () => ({ setStreamerCommandSender: () => {}, setVisionRequestSender: () => {} }));
vi.mock('@/lib/visionStateDc', () => ({ handleVisionStateFrame: () => {} }));
vi.mock('@/store/telemetry', () => ({ noteVisionLink: () => {} }));
vi.mock('@/lib/remoteIce', () => ({ modeField: () => ({}), resolveIceServers: async () => [], routeKey: () => (h.remote ? 'remote' : 'lan') }));
vi.mock('@/modules/registry', () => ({ remote: {}, remoteActive: () => h.remote }));
vi.mock('@/lib/installId', () => ({ installId: () => 'test', installIdReady: Promise.resolve() }));
vi.mock('./rest', () => ({ rest: {} }));
vi.mock('./connection', () => ({ connection: { secured: vi.fn(async () => h.secured), resecureFor: vi.fn() } }));
vi.mock('./robotTunnel', () => ({ attachPortTunnel: () => {} }));
vi.mock('@/lib/connectionRoute', () => ({ inboundVideoFromStats: () => null, routeDetail: () => ({}), readStats: () => [] }));
vi.mock('@/lib/videoKeyframe', () => ({ setKeyframeProbe: () => {} }));
vi.mock('@/lib/iosAudioSession', () => ({ iosAudioSession: { arm: () => {}, setEnabled: () => {} } }));
vi.mock('./simEngine', () => ({ simEngine: {} }));
vi.mock('./videoStale', () => ({ armVideoStaleClear: () => {}, cancelVideoStaleClear: () => {} }));
vi.mock('./robotTlsPlatform', () => ({ robotTlsPlatform: { base: () => undefined } }));

import { webrtcClient } from './webrtcClient';
import { connection } from './connection';

const plainAuth = (host: string) => h.sent.filter((x) => x.url.startsWith(`http://${host}:`) && x.auth);

beforeEach(() => {
  vi.useFakeTimers();
  h.sent.length = 0; h.remote = false; h.secured = true;
  vi.stubGlobal('fetch', vi.fn(async (url: string, init?: { headers?: Record<string, string> }) => {
    h.sent.push({ url, auth: !!init?.headers?.Authorization });
    return { ok: true, status: 200, json: async () => ({}) };
  }));
});
afterEach(() => { webrtcClient.disconnect(); vi.unstubAllGlobals(); vi.useRealTimers(); });

describe('영상 피어 — 판정 안 된 :8081 로 비번이 평문으로 안 나간다(R14)', () => {
  it('video_retry_sends_no_plain_auth_to_untouched_8081', async () => {
    h.secured = false;
    webrtcClient.connect('10.0.0.1');
    await vi.advanceTimersByTimeAsync(0);
    await vi.advanceTimersByTimeAsync(1000);
    await vi.advanceTimersByTimeAsync(2000);
    expect(vi.mocked(connection.secured).mock.calls.length).toBeGreaterThanOrEqual(3);
    expect(h.sent.filter((x) => x.url.endsWith('/api/webrtc/hangup')).length).toBeGreaterThanOrEqual(2);
    expect(plainAuth('10.0.0.1')).toEqual([]);
    expect(h.sent.some((x) => x.url.endsWith('/api/webrtc/offer'))).toBe(false);
  });

  it('video_hangup_skipped_in_remote', async () => {
    webrtcClient.connect('10.0.0.1');
    await vi.advanceTimersByTimeAsync(0);
    h.sent.length = 0;
    h.remote = true;
    webrtcClient.disconnect();
    expect(h.sent.filter((x) => x.url.endsWith('/api/webrtc/hangup'))).toEqual([]);
  });
});

describe('영상 offer 실패 뒤 재판정(F3 — 네이티브 expo/fetch 는 TypeError 가 아니다)', () => {
  const offerFails = async (err: Error) => {
    vi.mocked(connection.resecureFor).mockClear();
    vi.stubGlobal('fetch', vi.fn(async (url: string) => {
      if (url.endsWith('/api/webrtc/offer')) throw err;
      return { ok: true, status: 200, json: async () => ({}) };
    }));
    webrtcClient.connect('10.0.0.1');
    await vi.advanceTimersByTimeAsync(0);
    await vi.advanceTimersByTimeAsync(10);
  };

  it('TypeError → resecureFor', async () => {
    await offerFails(new TypeError('Network request failed'));
    expect(connection.resecureFor).toHaveBeenCalledTimes(1);
  });
  it('expo 식 Error(fetch failed: …SSLHandshakeException…) → resecureFor', async () => {
    await offerFails(new Error('fetch failed: javax.net.ssl.SSLHandshakeException: Trust anchor for certification path not found'));
    expect(connection.resecureFor).toHaveBeenCalledTimes(1);
  });
  it('취소(expo 식 aborted) → 재판정 안 함', async () => {
    await offerFails(new Error('fetch failed: The operation was aborted.'));
    expect(connection.resecureFor).not.toHaveBeenCalled();
  });
});
