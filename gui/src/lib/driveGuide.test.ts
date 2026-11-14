import { describe, expect, it } from 'vitest';
import { driveGuide, edgePoint, shoulderHalfWidth } from '@/lib/driveGuide';

const HW = shoulderHalfWidth(false);
const last = <T,>(a: T[]) => a[a.length - 1];

describe('driveGuide', () => {
  it('vx 만이면 앞으로 곧은 띠 5 m', () => {
    const g = driveGuide([1, 0, 0], HW);
    const e = last(g.samples);
    expect(e.x - g.samples[0].x).toBeCloseTo(5, 5);
    expect(e.y).toBeCloseTo(0, 6);
    expect(e.th).toBe(0);
    expect(g.ticks.map((t) => +(t.x - g.samples[0].x).toFixed(3))).toEqual([1, 2, 3, 4, 5]);
  });

  it('vx·vy 면 대각선 — 몸통 방향은 그대로', () => {
    const g = driveGuide([1.2, 1.2, 0], HW);
    const e = last(g.samples);
    expect(e.y).toBeGreaterThan(0);
    expect(e.y).toBeCloseTo(e.x - g.samples[0].x, 5);
    expect(e.th).toBe(0);
  });

  it('+wz 면 왼쪽으로 휘고 안쪽(왼쪽) 가장자리가 더 짧다', () => {
    const g = driveGuide([1, 0, 0.5], HW);
    const e = last(g.samples);
    expect(e.y).toBeGreaterThan(0);
    expect(e.th).toBeGreaterThan(0);
    const len = (off: number) => g.samples.slice(1).reduce((s, p, i) => {
      const [ax, ay] = edgePoint(g.samples[i], off), [bx, by] = edgePoint(p, off);
      return s + Math.hypot(bx - ax, by - ay);
    }, 0);
    expect(len(+HW)).toBeLessThan(len(-HW));
  });

  it('반 바퀴 넘게 감기지 않는다', () => {
    const g = driveGuide([1, 0, 3], HW);
    expect(Math.abs(last(g.samples).th)).toBeLessThanOrEqual(Math.PI + 1e-9);
  });

  it('전후만 있으면 직진 5 m — 뒤로면 뒤로', () => {
    const f = driveGuide([0.3, 0.01, -0.02], HW);
    expect(last(f.samples).x - f.samples[0].x).toBeCloseTo(5, 5);
    expect(last(f.samples).y).toBe(0);
    const b = driveGuide([-0.2, 0, 0], HW);
    expect(last(b.samples).x - b.samples[0].x).toBeCloseTo(-5, 5);
  });

  it('전후에 좌우·회전이 섞이면 명령 그대로 — 길이 5 m', () => {
    const g = driveGuide([0.3, 0.4, 0], HW);
    expect(last(g.samples).x - g.samples[0].x).toBeCloseTo(3, 5);
    expect(last(g.samples).y).toBeCloseTo(4, 5);
    expect(last(driveGuide([0.6, 0, 0.06], HW).samples).th).toBeCloseTo(0.5, 5);
    expect(last(driveGuide([0.2, 0, 0.02], HW).samples).th).toBe(0);
  });

  it('|vx| < 0.05 거나 값이 터무니없으면(옛 로봇 쓰레기) 앞으로 기본 직선 5 m', () => {
    for (const cmd of [[0, 0, 0], [0.04, 0.5, 0], [0, 0, 0.8], [-0.03, -0.4, 0.6], [NaN, NaN, NaN], [1e30, -7, 2e9]] as [number, number, number][]) {
      const g = driveGuide(cmd, HW);
      expect(last(g.samples).x - g.samples[0].x).toBeCloseTo(5, 5);
      expect(last(g.samples).y).toBeCloseTo(0, 6);
      expect(last(g.samples).th).toBe(0);
    }
  });

  it('뒤로 가면 뒤쪽 어깨 선에서 뒤로 뻗는다', () => {
    const g = driveGuide([-1.2, 0, 0], HW);
    expect(g.samples[0].x).toBeLessThan(0);
    expect(last(g.samples).x).toBeLessThan(g.samples[0].x);
  });
});
