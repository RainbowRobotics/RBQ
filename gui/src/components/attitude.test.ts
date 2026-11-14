import { describe, it, expect } from 'vitest';
import { horizonShift } from './attitude';

describe('인공수평 피치 부호', () => {
  it('로봇이 머리를 숙이면(피치 +) 수평선이 위로 간다', () => {
    expect(horizonShift(10, 60)).toBeLessThan(0);
  });
  it('머리를 들면(피치 −) 수평선이 아래로 간다', () => {
    expect(horizonShift(-10, 60)).toBeGreaterThan(0);
  });
  it('1°당 이동량은 원판 크기의 1/60', () => {
    expect(horizonShift(6, 120)).toBe(-12);
  });
});
