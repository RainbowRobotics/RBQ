import { describe, it, expect, vi, beforeAll } from 'vitest';

vi.mock('react-native', () => ({ Platform: { OS: 'web' } }));

import { autoRobotVersion, robotVersions, slam, slamVersionFor } from './registry';

describe('로봇 버전 자동 확정 — 깔린 옵션 먼저, 그다음 옛 로봇 표시', () => {
  robotVersions.splice(0, robotVersions.length,
    { key: 'a', label: 'A', feature: 'a_flag', option: 'aopt' },
    { key: 'b', label: 'B', option: 'bopt' },
    { key: 'c', label: 'C', feature: 'c_flag' });
  it('옵션 이름이 맞으면 그 버전', () => {
    expect(autoRobotVersion({ option: 'bopt' })?.key).toBe('b');
    expect(autoRobotVersion({ option: 'aopt', c_flag: true })?.key).toBe('a');
  });
  it('옵션이 없으면 옛 로봇 표시(feature)', () => {
    expect(autoRobotVersion({ option: '', c_flag: true })?.key).toBe('c');
    expect(autoRobotVersion({ a_flag: true })?.key).toBe('a');
  });
  it('둘 다 없으면 undefined(손으로 고른 값 유지)', () => {
    expect(autoRobotVersion({ option: 'zzz' })).toBeUndefined();
    expect(autoRobotVersion({})).toBeUndefined();
  });
});

describe('SLAM 명령 세트 — 깔린 옵션을 따른다', () => {
  beforeAll(() => {
    robotVersions.splice(0, robotVersions.length, { key: 'opt-a', label: 'OPT-A', option: 'opt-a' }, { key: 'k', label: 'K', option: 'kopt' });
    slam.variants.splice(0, slam.variants.length, { key: 'opt-a', label: 'OPT-A', slamVersion: 3, groups: () => [] });
  });
  it('옵션 A → 그 세트, 다른 옵션·옵션 없음 → 기본 0(앞 로봇의 옵션 A 세트가 남지 않게)', () => {
    expect(slamVersionFor({ option: 'opt-a' })).toBe(3);
    expect(slamVersionFor({ option: 'kopt' })).toBe(0);
    expect(slamVersionFor({ option: '' })).toBe(0);
  });
  it('옵션을 알려 오지 않는 옛 본체 → undefined(손으로 고른 값 유지)', () => {
    expect(slamVersionFor({})).toBeUndefined();
    expect(slamVersionFor({ rapid: true })).toBeUndefined();
  });
});
