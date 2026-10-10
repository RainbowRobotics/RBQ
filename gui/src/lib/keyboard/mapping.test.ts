import { describe, it, expect } from 'vitest';
import { keysToLeftStick, keysToRightStick, resolveFrame } from './mapping';

describe('keysToRightStick — 방향키 + Q/E', () => {
  it('Q = 좌회전(-nx), E = 우회전(+nx)', () => {
    expect(keysToRightStick(new Set(['KeyQ']))).toEqual({ nx: -1, ny: 0 });
    expect(keysToRightStick(new Set(['KeyE']))).toEqual({ nx: 1, ny: 0 });
  });
  it('Q 와 ← 를 같이 눌러도 한 번(겹쳐 두 배가 되지 않음)', () => {
    expect(keysToRightStick(new Set(['KeyQ', 'ArrowLeft']))).toEqual({ nx: -1, ny: 0 });
  });
});

describe('keysToLeftStick', () => {
  it('아무 키도 안 눌리면 0', () => {
    expect(keysToLeftStick(new Set())).toEqual({ nx: 0, ny: 0 });
  });
  it('W = 전진(+ny)', () => {
    expect(keysToLeftStick(new Set(['KeyW']))).toEqual({ nx: 0, ny: 1 });
  });
  it('S = 후진(-ny)', () => {
    expect(keysToLeftStick(new Set(['KeyS']))).toEqual({ nx: 0, ny: -1 });
  });
  it('D = 우(+nx), A = 좌(-nx)', () => {
    expect(keysToLeftStick(new Set(['KeyD']))).toEqual({ nx: 1, ny: 0 });
    expect(keysToLeftStick(new Set(['KeyA']))).toEqual({ nx: -1, ny: 0 });
  });
  it('상충 키(W+S)는 상쇄', () => {
    expect(keysToLeftStick(new Set(['KeyW', 'KeyS']))).toEqual({ nx: 0, ny: 0 });
  });
  it('대각선(W+D)은 크기 1로 정규화', () => {
    const v = keysToLeftStick(new Set(['KeyW', 'KeyD']));
    expect(Math.hypot(v.nx, v.ny)).toBeCloseTo(1, 5);
    expect(v.nx).toBeCloseTo(Math.SQRT1_2, 5);
    expect(v.ny).toBeCloseTo(Math.SQRT1_2, 5);
  });
});

describe('keysToRightStick (방향키=회전)', () => {
  it('→ = 우 yaw(+nx), ← = 좌(-nx)', () => {
    expect(keysToRightStick(new Set(['ArrowRight']))).toEqual({ nx: 1, ny: 0 });
    expect(keysToRightStick(new Set(['ArrowLeft']))).toEqual({ nx: -1, ny: 0 });
  });
  it('↑ = +ny, ↓ = -ny', () => {
    expect(keysToRightStick(new Set(['ArrowUp']))).toEqual({ nx: 0, ny: 1 });
    expect(keysToRightStick(new Set(['ArrowDown']))).toEqual({ nx: 0, ny: -1 });
  });
  it('WASD는 우스틱에 영향 없음(축 분리)', () => {
    expect(keysToRightStick(new Set(['KeyW', 'KeyD']))).toEqual({ nx: 0, ny: 0 });
  });
});

describe('resolveFrame (안전 게이트)', () => {
  it('비암(disarm)이면 입력이 있어도 전축 0', () => {
    const r = resolveFrame(false, { nx: 1, ny: 1 }, { nx: -1, ny: 0.5 });
    expect(r).toEqual({ l: { nx: 0, ny: 0 }, r: { nx: 0, ny: 0 } });
  });
  it('암이면 입력 그대로 통과', () => {
    const r = resolveFrame(true, { nx: 0.3, ny: -0.2 }, { nx: 0.5, ny: 0 });
    expect(r).toEqual({ l: { nx: 0.3, ny: -0.2 }, r: { nx: 0.5, ny: 0 } });
  });
});
