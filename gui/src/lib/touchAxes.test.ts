import { describe, it, expect } from 'vitest';
import { resolveTouchAxes } from './touchAxes';

describe('resolveTouchAxes', () => {
  it('oneStick=false일 때 원본 그대로', () => {
    const left = { x: 0.1, y: 0.2 };
    const right = { x: 0.3, y: 0.4 };
    expect(resolveTouchAxes(false, left, right)).toEqual({
      L: { x: 0.1, y: 0.2 },
      R: { x: 0.3, y: 0.4 },
    });
  });

  it('oneStick=true일 때 X축 교차, Y 유지', () => {
    const left = { x: 0.1, y: 0.2 };
    const right = { x: 0.3, y: 0.4 };
    expect(resolveTouchAxes(true, left, right)).toEqual({
      L: { x: 0.3, y: 0.2 },
      R: { x: 0.1, y: 0.4 },
    });
  });
});
