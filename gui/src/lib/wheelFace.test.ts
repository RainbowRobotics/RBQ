import { describe, expect, it } from 'vitest';
import { wheelFaceGeos, WHEEL_MAX_R } from './wheelFace';

describe('wheelFace', () => {
  const g = wheelFaceGeos();

  it('어느 부품도 타이어 꼭대기 반지름을 넘지 않는다 — 넘으면 접지 높이가 틀어진다', () => {
    for (const [name, geo] of Object.entries(g)) {
      const p = geo.getAttribute('position');
      let r = 0;
      for (let i = 0; i < p.count; i++) r = Math.max(r, Math.hypot(p.getX(i), p.getY(i)));
      expect(r, name).toBeLessThanOrEqual(WHEEL_MAX_R);
    }
  });

  it('바깥 면(옆면 0.03)보다 튀어나오지 않는다', () => {
    for (const [name, geo] of Object.entries(g)) {
      geo.computeBoundingBox();
      expect(geo.boundingBox!.max.z, name).toBeLessThanOrEqual(0.03);
      expect(geo.boundingBox!.min.z, name).toBeGreaterThanOrEqual(-0.03);
    }
  });
});
