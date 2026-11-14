import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const h = vi.hoisted(() => {
  class FakePc {
    iceGatheringState = 'complete';
    localDescription = { sdp: 'v=0' };
    connectionState = 'new';
    addEventListener() {}
    removeEventListener() {}
    addTransceiver() { return {}; }
    createDataChannel() { return {}; }
    async createOffer() { return { sdp: 'v=0\r\n' }; }
    async setLocalDescription() {}
    async setRemoteDescription() {}
    close() {}
  }
  return { FakePc, secured: true, offer: { status: 200, body: {} as unknown }, sent: [] as string[] };
});
vi.mock('@/lib/auth', () => ({ robotAuth: () => ({ Authorization: 'Basic test' }) }));
vi.mock('@/lib/demoFlag', () => ({ isDemo: () => false }));
vi.mock('@/lib/endpoints', () => ({ streamerBase: () => '/streamer' }));
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => ({ webrtcToken: '', audio: { robotSpk: 0, robotMic: 0 } }), subscribe: () => () => {} } }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ isMine: true }), subscribe: () => () => {} } }));
vi.mock('@/lib/spectating', () => ({ waitRoleKnown: async () => {} }));
vi.mock('@/lib/pointcloud', () => ({ attachPointcloudChannel: () => {}, resetAllClouds: () => {} }));
vi.mock('@/lib/elevationMap', () => ({ attachElevationChannel: () => {} }));
vi.mock('@/lib/wallMap', () => ({ attachWallmapChannel: () => {} }));
vi.mock('@/lib/commandBus', () => ({ setStreamerCommandSender: () => {}, setVisionRequestSender: () => {} }));
vi.mock('@/lib/visionStateDc', () => ({ handleVisionStateFrame: () => {} }));
vi.mock('@/store/telemetry', () => ({ noteVisionLink: () => {} }));
vi.mock('@/lib/connectionRoute', () => ({ inboundVideoFromStats: () => null, readStats: () => [] }));
vi.mock('@/lib/videoKeyframe', () => ({ setKeyframeProbe: () => {} }));
vi.mock('./simEngine', () => ({ simEngine: {} }));
vi.mock('./videoStale', () => ({ armVideoStaleClear: () => {}, cancelVideoStaleClear: () => {} }));
vi.mock('./rest', () => ({ rest: {} }));
vi.mock('./robotTunnel', () => ({ attachPortTunnel: () => {} }));
vi.mock('@/lib/remoteIce', () => ({ resolveIceServers: async () => [] }));
vi.mock('@/modules/registry', () => ({ remote: {}, remoteActive: () => false }));
vi.mock('@/lib/installId', () => ({ installId: () => 'test', installIdReady: Promise.resolve() }));
vi.mock('./connection', () => ({ connection: { secured: vi.fn(async () => h.secured), resecureAfterFailure: vi.fn() } }));

import { webrtcClient } from './webrtcClient.web';
import { connection } from './connection';

beforeEach(() => {
  vi.useFakeTimers();
  h.sent.length = 0; h.secured = true; h.offer = { status: 200, body: {} };
  vi.mocked(connection.resecureAfterFailure).mockClear();
  vi.stubGlobal('RTCPeerConnection', h.FakePc);
  vi.stubGlobal('fetch', vi.fn(async (url: string) => {
    h.sent.push(url);
    const { status, body } = url.endsWith('/api/webrtc/offer') ? h.offer : { status: 200, body: {} };
    return { ok: status < 400, status, headers: { get: () => 'application/json' }, json: async () => body };
  }));
});
afterEach(() => { webrtcClient.disconnect(); vi.unstubAllGlobals(); vi.useRealTimers(); });

describe('웹 영상 피어 — 지문 판정(R14)·막힌 포트 재판정(R10)', () => {
  it('web_video_offer_waits_for_judgement', async () => {
    h.secured = false;
    webrtcClient.connect('10.0.0.1');
    await vi.advanceTimersByTimeAsync(0);
    await vi.advanceTimersByTimeAsync(1000);
    expect(vi.mocked(connection.secured)).toHaveBeenCalledWith('10.0.0.1');
    expect(vi.mocked(connection.secured).mock.calls.length).toBeGreaterThanOrEqual(2);
    expect(h.sent.some((u) => u.endsWith('/api/webrtc/offer'))).toBe(false);
  });

  it('web_video_tls_refused_rejudges', async () => {
    h.offer = { status: 502, body: { error: 'tls-refused' } };
    webrtcClient.connect('10.0.0.1');
    await vi.advanceTimersByTimeAsync(0);
    expect(h.sent.filter((u) => u.endsWith('/api/webrtc/offer')).length).toBe(1);
    expect(connection.resecureAfterFailure).toHaveBeenCalledWith('10.0.0.1');
    expect(connection.resecureAfterFailure).toHaveBeenCalledTimes(1);
  });

  it('web_video_plain_upstream_failure_does_not_rejudge', async () => {
    h.offer = { status: 502, body: { error: 'upstream ECONNREFUSED' } };
    webrtcClient.connect('10.0.0.1');
    await vi.advanceTimersByTimeAsync(0);
    expect(connection.resecureAfterFailure).not.toHaveBeenCalled();
  });
});
