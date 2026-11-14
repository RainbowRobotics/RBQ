import { describe, it, expect, vi } from 'vitest';

vi.mock('react-native', () => ({ useWindowDimensions: () => ({ width: 1280, height: 800 }) }));
import { dashType } from './dashType';

describe('dashType', () => {
  it('1200 이하(덱 1024·폰 가로 731)는 기본값', () => {
    for (const w of [731, 1024, 1200]) {
      const ty = dashType(w);
      expect(ty.body).toBe(13);
      expect(ty.title).toBe(13);
      expect(ty.label).toBe(11);
    }
  });

  it('1280 에서 본문 14 — 표·요약 카드가 같은 값', () => {
    expect(dashType(1280).body).toBe(14);
    expect(dashType(1280).title).toBe(14);
  });

  it('상한 1.15 — 아무리 넓어도 본문 15 를 넘지 않는다(2열 관절 표 A 칸 한계)', () => {
    expect(dashType(1920).body).toBe(15);
    expect(dashType(3840).body).toBe(15);
  });

  it('모든 값은 0.5 단위 — 13.87 같은 값이 새지 않는다', () => {
    for (const w of [800, 1100, 1280, 1366, 1600, 1920]) {
      const ty = dashType(w);
      for (const [k, v] of Object.entries(ty)) if (k !== 'k') expect(v * 2).toBe(Math.round(v * 2));
    }
  });

  it('역할 사이 크기 순서가 유지된다 — 머리글 < 설명 < 버튼 < 본문', () => {
    for (const w of [1024, 1280, 1920]) {
      const ty = dashType(w);
      expect(ty.label).toBeLessThan(ty.caption);
      expect(ty.caption).toBeLessThanOrEqual(ty.button);
      expect(ty.button).toBeLessThan(ty.body);
    }
  });
});
