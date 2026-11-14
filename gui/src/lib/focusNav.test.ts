import { describe, expect, it } from 'vitest';
import { pickNext } from './focusNav.web';

const b = (x: number, y: number) => ({ x, y, w: 10, h: 10 });

describe('pickNext — 패드 십자키로 다음 버튼 고르기', () => {
  const grid = [b(0, 0), b(20, 0), b(40, 0), b(0, 20), b(20, 20), b(40, 20)];
  it('같은 줄·같은 칸의 이웃으로 간다', () => {
    expect(pickNext(b(20, 0), grid, 'right')).toBe(2);
    expect(pickNext(b(20, 0), grid, 'left')).toBe(0);
    expect(pickNext(b(20, 0), grid, 'down')).toBe(4);
    expect(pickNext(b(20, 20), grid, 'up')).toBe(1);
  });
  it('그 방향에 아무것도 없으면 -1(제자리)', () => {
    expect(pickNext(b(40, 0), grid, 'right')).toBe(-1);
    expect(pickNext(b(0, 0), grid, 'up')).toBe(-1);
  });
  it('옆으로 많이 어긋난 먼 칸보다 곧게 가까운 칸', () => {
    expect(pickNext(b(0, 0), [b(100, 0), b(30, 60)], 'right')).toBe(0);
  });
});
