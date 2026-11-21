import { describe, it, expect } from 'vitest';
import { sensorLayoutFor } from './sensorState';

describe('sensorLayoutFor — /api/version → Vision 프레임 해석 세대', () => {
  it('태그 없는 개발 빌드·해석 불가는 current', () => {
    expect(sensorLayoutFor('0.0.0-dev')).toBe('current');
    expect(sensorLayoutFor('')).toBe('current');
    expect(sensorLayoutFor(undefined)).toBe('current');
    expect(sensorLayoutFor('unknown')).toBe('current');
  });
  it('정식 1.19 미만만 legacy118', () => {
    expect(sensorLayoutFor('1.18.1')).toBe('legacy118');
    expect(sensorLayoutFor('1.19.0')).toBe('current');
    expect(sensorLayoutFor('v1.20.0-134-gabcdef12')).toBe('current');
  });
});
