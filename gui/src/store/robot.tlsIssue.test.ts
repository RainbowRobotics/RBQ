import { describe, it, expect, vi, beforeEach } from 'vitest';

vi.mock('@/lib/demoFlag', () => ({ getDemoBaseRobot: () => null }));
vi.mock('@react-native-async-storage/async-storage', () => ({
  default: { getItem: vi.fn(async () => null), setItem: vi.fn(async () => {}), removeItem: vi.fn(async () => {}) },
}));

import { useRobot } from './robot';
import { reportTlsIssue } from '@/lib/robotTls';

describe('tlsIssue — 지문 경고 창 하나', () => {
  beforeEach(() => useRobot.getState().setTlsIssue(null));

  it('같은 로봇의 바뀐 포트들은 한 창에 모인다 — 다시 기억 한 번에 다 기억', () => {
    reportTlsIssue({ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [] });
    reportTlsIssue({ kind: 'changed', entryKey: 'S', port: 8081, fp: 'F', forget: [] });
    expect(useRobot.getState().tlsIssue).toEqual({ kind: 'changed', entryKey: 'S', fps: { 8080: 'F', 8081: 'F' }, forget: [] });
  });

  it('changed_merge_takes_newest_forget_minus_fps', () => {
    reportTlsIssue({ kind: 'changed', entryKey: 'S', port: 8080, fp: 'F', forget: [8081, 8082] });
    reportTlsIssue({ kind: 'changed', entryKey: 'S', port: 8081, fp: 'F', forget: [8082] });
    expect(useRobot.getState().tlsIssue).toEqual({ kind: 'changed', entryKey: 'S', fps: { 8080: 'F', 8081: 'F' }, forget: [8082] });
    reportTlsIssue({ kind: 'changed', entryKey: 'S', port: 8081, fp: 'F', forget: [8080] });
    expect(useRobot.getState().tlsIssue).toEqual({ kind: 'changed', entryKey: 'S', fps: { 8080: 'F', 8081: 'F' }, forget: [] });
    reportTlsIssue({ kind: 'changed', entryKey: 'T', port: 8082, fp: 'F', forget: [8080] });
    expect(useRobot.getState().tlsIssue).toEqual({ kind: 'changed', entryKey: 'S', fps: { 8080: 'F', 8081: 'F' }, forget: [] });
  });

  it('창이 떠 있는 동안 다른 경고는 버리고, 닫은 뒤엔 새로 뜬다', () => {
    reportTlsIssue({ kind: 'refuse-old', entryKey: 'A' });
    reportTlsIssue({ kind: 'refuse-old', entryKey: 'A' });
    reportTlsIssue({ kind: 'changed', entryKey: 'B', port: 8080, fp: 'F', forget: [] });
    expect(useRobot.getState().tlsIssue).toEqual({ kind: 'refuse-old', entryKey: 'A' });
    useRobot.getState().setTlsIssue(null);
    reportTlsIssue({ kind: 'changed', entryKey: 'B', port: 8080, fp: 'F', forget: [] });
    expect(useRobot.getState().tlsIssue).toEqual({ kind: 'changed', entryKey: 'B', fps: { 8080: 'F' }, forget: [] });
  });
});
