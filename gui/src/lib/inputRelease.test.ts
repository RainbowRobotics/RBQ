import { describe, it, expect, vi, beforeEach } from 'vitest';

const h = vi.hoisted(() => ({
  listener: (_s: string) => {},
  Platform: { OS: 'ios' as string },
  conn: { releaseAll: vi.fn(), resecure: vi.fn(), suspendRelays: vi.fn() },
}));
vi.mock('react-native', () => ({
  AppState: { currentState: 'active', addEventListener: (_e: string, fn: (s: string) => void) => { h.listener = fn; } },
  Platform: h.Platform,
}));
vi.mock('@/lib/gamepad/manager', () => ({ clearGamepadOutputs: vi.fn() }));
vi.mock('@/lib/connection', () => ({ connection: h.conn }));

import { installInputRelease } from './inputRelease';

describe('앱 활성 변화 — 지문 중계', () => {
  installInputRelease();
  beforeEach(() => vi.clearAllMocks());

  it('iOS: 뒤로 가면 중계 경로를 막고, 앞으로 오면 다시 판정한다', () => {
    h.Platform.OS = 'ios';
    h.listener('inactive');
    expect(h.conn.suspendRelays).not.toHaveBeenCalled();
    h.listener('background');
    expect(h.conn.suspendRelays).toHaveBeenCalledTimes(1);
    expect(h.conn.resecure).not.toHaveBeenCalled();
    h.listener('active');
    expect(h.conn.resecure).toHaveBeenCalledTimes(1);
    expect(h.conn.suspendRelays.mock.invocationCallOrder[0]).toBeLessThan(h.conn.resecure.mock.invocationCallOrder[0]);
  });

  it('Android: 뒤로 가도 중계는 그대로(거두지 않는다), 앞으로 오면 다시 판정', () => {
    h.Platform.OS = 'android';
    h.listener('background');
    expect(h.conn.suspendRelays).not.toHaveBeenCalled();
    h.listener('active');
    expect(h.conn.resecure).toHaveBeenCalledTimes(1);
  });
});
