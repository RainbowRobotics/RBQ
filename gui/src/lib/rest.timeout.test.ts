import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

vi.mock('./auth', () => ({ robotAuth: () => ({}), reportAuthFailure: () => {} }));
vi.mock('./endpoints', () => ({ restBase: () => 'http://10.0.0.9:8080', streamerBase: () => '' }));
vi.mock('@/lib/demoFlag', () => ({ isDemo: () => false, demoDriver: () => undefined }));
vi.mock('./commandBus', () => ({ dcCommand: () => Promise.resolve({}), streamerCommand: () => Promise.resolve({}) }));
vi.mock('./robotTunnel', () => ({ RemoteUnsupportedError: class extends Error {}, tunnelFetch: vi.fn(), tunnelReady: () => true, viaRemote: () => false }));

import { robotFetch, fetchWithin, rest, HttpError, REST_TIMEOUT_MS, DOWNLOAD_TIMEOUT_MS } from './rest';
import { isFetchFailure } from './fetchFailure';

const hung = vi.fn((_url: string, init?: RequestInit) => new Promise<Response>((_, reject) => {
  init?.signal?.addEventListener('abort', () => reject(Object.assign(new Error('Aborted'), { name: 'AbortError' })));
}));

beforeEach(() => { vi.useFakeTimers(); hung.mockClear(); vi.stubGlobal('fetch', hung); });
afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); });

function track<T>(p: Promise<T>) {
  const box: { v: unknown } = { v: 'pending' };
  p.then((v) => { box.v = v; }, (e) => { box.v = e; });
  return box;
}

describe('robotFetch 상한', () => {
  it(`보통 호출은 ${REST_TIMEOUT_MS / 1000}s 에 '응답 없음'(HttpError 0)으로 끝난다 — 호출부 catch 로 간다`, async () => {
    const r = track(rest.version('10.0.0.9'));
    await vi.advanceTimersByTimeAsync(REST_TIMEOUT_MS - 1);
    expect(r.v).toBe('pending');
    await vi.advanceTimersByTimeAsync(1);
    expect(r.v).toBeInstanceOf(HttpError);
    expect((r.v as HttpError).status).toBe(0);
    expect((r.v as HttpError).message).toBe('/api/version — 응답 없음(10s)');
  });

  it('파일 받기(DOWNLOAD_TIMEOUT_MS)는 보통 상한을 넘겨도 기다리고 그 상한에 끝난다', async () => {
    const r = track(rest.systemlogByDate('10.0.0.9', '20261116'));
    await vi.advanceTimersByTimeAsync(REST_TIMEOUT_MS * 3);
    expect(r.v).toBe('pending');
    await vi.advanceTimersByTimeAsync(DOWNLOAD_TIMEOUT_MS);
    expect((r.v as HttpError).status).toBe(0);
  });

  it('0 이면 상한 없음 — signal 을 붙이지 않는다', async () => {
    const r = track(robotFetch('http://10.0.0.9:8080/api/x', {}, 0));
    await vi.advanceTimersByTimeAsync(DOWNLOAD_TIMEOUT_MS * 2);
    expect(r.v).toBe('pending');
    expect(hung.mock.calls[0][1]?.signal).toBeUndefined();
  });

  it('부르는 쪽이 끊으면 시한 오류가 아니라 그 취소 그대로', async () => {
    const ac = new AbortController();
    const r = track(robotFetch('http://10.0.0.9:8080/api/x', { signal: ac.signal }));
    ac.abort();
    await vi.advanceTimersByTimeAsync(0);
    expect((r.v as Error).name).toBe('AbortError');
  });

  it('영상·SLAM offer(fetchWithin, 협상 시한)도 시한에 끝나고, 그 오류는 지문 재판정 대상이 아니다', async () => {
    const r = track(fetchWithin('https://127.0.0.1:41234/api/webrtc/offer', { method: 'POST' }, 20_000));
    await vi.advanceTimersByTimeAsync(20_000);
    expect((r.v as HttpError).message).toBe('/api/webrtc/offer — 응답 없음(20s)');
    expect(isFetchFailure(r.v)).toBe(false);
  });

  it('네트워크 실패는 바꾸지 않는다 — 재판정(resecureFor)이 보는 오류 그대로', async () => {
    vi.stubGlobal('fetch', vi.fn(async () => { throw new TypeError('Network request failed'); }));
    const e = await fetchWithin('https://127.0.0.1:41234/api/webrtc/offer', {}, 20_000).catch((x) => x);
    expect(e).toBeInstanceOf(TypeError);
    expect(isFetchFailure(e)).toBe(true);
  });

  it('제때 오면 그대로 돌려준다', async () => {
    const res = new Response('{"version":"v1.2.3"}', { status: 200 });
    vi.stubGlobal('fetch', vi.fn(async () => res));
    await expect(rest.version('10.0.0.9')).resolves.toMatchObject({ version: '1.2.3' });
  });
});
