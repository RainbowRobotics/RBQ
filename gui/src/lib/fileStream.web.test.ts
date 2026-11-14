import { describe, it, expect, vi, afterEach } from 'vitest';

vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
import { readFileAt, openFileStream, rememberPicked } from './fileStream.web';

afterEach(() => vi.unstubAllGlobals());

describe('웹 위치 지정 읽기 — 고른 파일', () => {
  it('시스템 선택기로 고른 파일은 원본 File 을 잘라 읽는다 — blob: URL 을 fetch 하지 않는다', async () => {
    const fetchSpy = vi.fn(() => Promise.reject(new Error('fetch')));
    vi.stubGlobal('fetch', fetchSpy);
    rememberPicked('blob:http://x/1', new Blob([new Uint8Array([1, 2, 3, 4, 5, 6])]));
    expect([...(await readFileAt('blob:http://x/1', 2, 3))]).toEqual([3, 4, 5]);
    const r = openFileStream('blob:http://x/1', { offset: 1, length: 2 });
    expect([...((await r.read()).value ?? [])]).toEqual([2, 3]);
    expect(fetchSpy).not.toHaveBeenCalled();
  });
});
