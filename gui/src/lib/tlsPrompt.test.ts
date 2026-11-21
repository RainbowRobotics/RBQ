import { describe, it, expect, vi, beforeEach } from 'vitest';

const h = vi.hoisted(() => ({
  robot: { conn: 'disconnected', tlsIssue: null as unknown, setTlsIssue: vi.fn((v: unknown) => { h.robot.tlsIssue = v; }) },
  robots: { rememberTlsPins: vi.fn(), select: vi.fn(async () => {}) },
  reconnectCurrent: vi.fn(() => true),
  disconnectByUser: vi.fn(),
}));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => h.robot } }));
vi.mock('@/store/robots', () => ({ useRobots: { getState: () => h.robots } }));
vi.mock('@/lib/connectNow', () => ({ reconnectCurrent: h.reconnectCurrent, disconnectByUser: h.disconnectByUser }));

import { rememberAgain, dismissTlsPrompt } from './tlsPrompt';

describe('지문 경고 창의 두 동작', () => {
  beforeEach(() => { vi.clearAllMocks(); h.robot.conn = 'disconnected'; h.robot.tlsIssue = { kind: 'changed' }; h.reconnectCurrent.mockReturnValue(true); });

  it('다시 기억 — 지문을 쓰고 창을 닫은 뒤 지금 대상으로 다시 붙는다', () => {
    rememberAgain('S', { 8080: 'F', 8081: 'F' }, []);
    expect(h.robots.rememberTlsPins).toHaveBeenCalledWith('S', { 8080: 'F', 8081: 'F' }, []);
    expect(h.robot.tlsIssue).toBeNull();
    expect(h.reconnectCurrent).toHaveBeenCalled();
    expect(h.robots.rememberTlsPins.mock.invocationCallOrder[0]).toBeLessThan(h.reconnectCurrent.mock.invocationCallOrder[0]);
    expect(h.robots.select).not.toHaveBeenCalled();
  });

  it('다시 기억 — 지금 대상이 없으면(saveConnected 가 끊었다) 그 로봇을 다시 고른다', () => {
    h.reconnectCurrent.mockReturnValue(false);
    rememberAgain('S', { 8080: 'F' }, []);
    expect(h.robots.select).toHaveBeenCalledWith('S');
  });

  it('remember_again_forgets_unchecked_ports', () => {
    rememberAgain('S', { 8080: 'N' }, [8081]);
    expect(h.robots.rememberTlsPins).toHaveBeenCalledWith('S', { 8080: 'N' }, [8081]);
  });

  it('붙지 않은 채 닫으면 사람이 끊은 것으로 — 붙어 있으면(스캔이 올린 경고) 건드리지 않는다', () => {
    dismissTlsPrompt();
    expect(h.robot.tlsIssue).toBeNull();
    expect(h.disconnectByUser).toHaveBeenCalledTimes(1);
    h.robot.conn = 'connected';
    dismissTlsPrompt();
    expect(h.disconnectByUser).toHaveBeenCalledTimes(1);
  });
});
