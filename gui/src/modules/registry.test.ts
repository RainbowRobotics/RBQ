import { describe, it, expect, vi } from 'vitest';

vi.mock('react-native', () => ({ Platform: { OS: 'web' } }));

import { autoRobotVersion, robotVersions } from './registry';

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
