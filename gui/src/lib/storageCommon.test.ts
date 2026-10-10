import { describe, it, expect } from 'vitest';
import { checkFree, isNoSpaceError, staleSaved, StorageFullError } from '@/lib/storageCommon';

const MB = 1024 * 1024;

describe('checkFree', () => {
  it('여유를 모르면(null) 통과시킨다 — 모른다고 막으면 멀쩡한 기기가 못 쓴다', () => {
    expect(() => checkFree(null, 500 * MB)).not.toThrow();
  });

  it('필요분 + 예비분(100MB)까지 있어야 통과한다', () => {
    expect(() => checkFree(300 * MB, 200 * MB)).not.toThrow();
    expect(() => checkFree(299 * MB, 200 * MB)).toThrow(StorageFullError);
  });

  it('need=0 이어도 예비분은 본다 — 꽉 찬 기기는 크기를 몰라도 막는다', () => {
    expect(() => checkFree(10 * MB, 0)).toThrow(StorageFullError);
  });

  it('문구에 필요·잔여를 함께 담는다(둘 다 알 때)', () => {
    expect(new StorageFullError(50 * MB, 400 * MB).message).toContain('필요 400MB, 남은 공간 50MB');
    expect(new StorageFullError(null, 0).message).not.toContain('—');
  });
});

describe('isNoSpaceError', () => {
  it('플랫폼별 공간부족 원문을 잡는다', () => {
    for (const m of ['ENOSPC: no space left', 'No space left on device', 'QuotaExceededError'])
      expect(isNoSpaceError(new Error(m))).toBe(true);
  });

  it('자기 자신도 공간부족으로 본다(사전 체크 → 재판정)', () => {
    expect(isNoSpaceError(new StorageFullError(0, 0))).toBe(true);
  });

  it('네트워크 실패는 공간부족이 아니다', () => {
    expect(isNoSpaceError(new Error('Network request failed'))).toBe(false);
    expect(isNoSpaceError(new Error('blackbox zip → HTTP 404'))).toBe(false);
  });
});

describe('staleSaved', () => {
  const f = (name: string, at: number) => ({ name, at });

  it('가장 최근 것 하나만 남긴다', () => {
    const files = [f('a', 100), f('c', 300), f('b', 200)];
    expect(staleSaved(files).map((x) => x.name).sort()).toEqual(['a', 'b']);
  });

  it('하나뿐이면 지우지 않는다 — 공유가 아직 안 끝났을 수 있다', () => {
    expect(staleSaved([f('only', 1)])).toEqual([]);
    expect(staleSaved([])).toEqual([]);
  });

  it('짧은 시간에 여러 번 저장해도 쌓이지 않는다(나이 기준의 구멍)', () => {
    const now = Date.now() / 1000;
    const burst = Array.from({ length: 10 }, (_, i) => f(`s${i}`, now + i));
    expect(staleSaved(burst)).toHaveLength(9);
    expect(staleSaved(burst).some((x) => x.name === 's9')).toBe(false);
  });

  it('시계가 뒤로 튀어도 남는 개수는 하나다', () => {
    const files = [f('a', 500), f('b', 100), f('c', 300)];
    expect(staleSaved(files)).toHaveLength(2);
  });
});
