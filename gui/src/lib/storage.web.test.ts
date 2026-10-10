import { describe, it, expect, vi, afterEach } from 'vitest';
import { freeBytes, ensureFreeSpace } from '@/lib/storage.web';

const MB = 1024 * 1024;

afterEach(() => vi.unstubAllGlobals());

describe('storage.web', () => {
  it('여유를 모른다고 답한다 — 브라우저는 다운로드 폴더를 못 본다', async () => {
    expect(await freeBytes()).toBeNull();
  });

  it('오리진 쿼터가 작아도 저장을 막지 않는다 — WebKit(덱)은 디스크와 무관한 상한을 준다', async () => {
    vi.stubGlobal('navigator', { storage: { estimate: () => Promise.resolve({ quota: 10 * MB, usage: 9 * MB }) } });
    await expect(ensureFreeSpace(2000 * MB)).resolves.toBeUndefined();
  });

  it('navigator 가 없어도 던지지 않는다', async () => {
    vi.stubGlobal('navigator', undefined);
    await expect(ensureFreeSpace(1 * MB)).resolves.toBeUndefined();
    expect(await freeBytes()).toBeNull();
  });
});
