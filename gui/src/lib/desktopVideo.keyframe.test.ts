import { describe, it, expect, vi, beforeEach } from 'vitest';

const kf = vi.hoisted(() => ({ n: 0, wanted: true }));
vi.mock('./videoKeyframe', () => ({ noteKeyframe: () => { kf.n++; }, keyframeWanted: () => kf.wanted }));
vi.mock('./webrtcClient', () => ({ useWebrtcStore: { getState: () => ({ imgMode: true }), setState: () => {} } }));
vi.mock('./commandBus', () => ({ setStreamerCommandSender: () => {}, setVisionRequestSender: () => {} }));
vi.mock('./desktopAudio', () => ({ desktopAudio: { playPcm: () => {} } }));
vi.mock('@/lib/pointcloud', () => ({ handlePointcloudFrame: () => {}, resetAllClouds: () => {} }));
vi.mock('./visionStateDc', () => ({ handleVisionStateFrame: () => {} }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({}) } }));
vi.mock('@/store/telemetry', () => ({ noteVisionLink: () => {} }));
vi.mock('./simEngine', () => ({ simEngine: { active: false } }));
vi.mock('./videoStale', () => ({ armVideoStaleClear: () => {}, cancelVideoStaleClear: () => {} }));

import { desktopVideo } from './desktopVideo';

const v = desktopVideo as unknown as { onFrame(d: ArrayBuffer): void; pendingUrl: string | null; idrPending: boolean; imgEl: unknown };
const frame = () => new Uint8Array([0x00, 0xff, 0xd8, 0xff, 0xd9]).buffer;
const ctrl = (o: object) => { const j = new TextEncoder().encode(JSON.stringify(o)); const b = new Uint8Array(j.length + 1); b[0] = 0xff; b.set(j, 1); return b.buffer; };

let n = 0;
beforeEach(() => {
  kf.n = 0; kf.wanted = true; v.pendingUrl = null; v.idrPending = false; v.imgEl = null;
  globalThis.URL.createObjectURL = () => `blob:${++n}`;
  globalThis.URL.revokeObjectURL = () => {};
});

describe('desktopVideo — IDR 신호와 <img>', () => {
  it('<img> 미등록이면 프레임을 pendingUrl 에 남기고, IDR 신호는 그릴 때까지 미룬다', () => {
    v.onFrame(ctrl({ t: 'idr' }));
    v.onFrame(frame());
    expect(v.pendingUrl).toMatch(/^blob:/);
    expect(kf.n).toBe(0);
    expect(v.idrPending).toBe(true);
  });

  it('<img> 가 있으면 src 에 꽂고 pendingUrl 은 건드리지 않으며, decode 뒤 키프레임을 알린다', async () => {
    const el = { src: '', decode: () => Promise.resolve() };
    v.imgEl = el;
    v.onFrame(ctrl({ t: 'idr' }));
    v.onFrame(frame());
    expect(el.src).toMatch(/^blob:/);
    expect(v.pendingUrl).toBeNull();
    await Promise.resolve(); await Promise.resolve();
    expect(kf.n).toBe(1);
    v.onFrame(frame());
    await Promise.resolve();
    expect(kf.n).toBe(1);
    expect(v.pendingUrl).toBeNull();
  });

  it('기다리는 쪽이 없으면 decode() 를 부르지 않는다 — WebKitGTK 는 그 순간 <img> 를 한 프레임 비운다(덱 하얀 깜빡임)', () => {
    let decoded = 0;
    v.imgEl = { src: '', decode: () => { decoded++; return Promise.resolve(); } };
    kf.wanted = false;
    v.onFrame(ctrl({ t: 'idr' }));
    v.onFrame(frame());
    expect(decoded).toBe(0);
    expect(kf.n).toBe(1);
  });
});
