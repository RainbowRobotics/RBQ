import { describe, it, expect } from 'vitest';
import { nextKeymap, KEYMAP_MIN_LEVEL } from '@/lib/accessKeymap';

describe('접근 레벨 → 키매핑', () => {
  it('개발자(2)로 올라가면 켠다', () => {
    expect(nextKeymap(1, 2, false)).toBe(true);
  });

  it('레벨 3 도 켠다', () => {
    expect(nextKeymap(1, 3, false)).toBe(true);
  });

  it('⚠일반(1)로 내려오면 반드시 끈다 — 전 버튼이 열린 채 남으면 안 된다', () => {
    expect(nextKeymap(2, 1, true)).toBe(false);
    expect(nextKeymap(3, 1, true)).toBe(false);
  });

  it('같은 레벨 재설정은 사람이 끈 선택을 덮지 않는다', () => {
    expect(nextKeymap(2, 2, false)).toBe(false);
    expect(nextKeymap(2, 2, true)).toBe(true);
  });

  it('기준 레벨은 개발자(2)', () => {
    expect(KEYMAP_MIN_LEVEL).toBe(2);
  });
});
