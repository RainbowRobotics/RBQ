import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { armVideoStaleClear, cancelVideoStaleClear } from './videoStale';

describe('videoStale', () => {
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => { cancelVideoStaleClear(); vi.useRealTimers(); });

  it('짧은 끊김은 프레임을 남긴다 — 유예 안에 취소되면 안 치운다', () => {
    const clear = vi.fn();
    armVideoStaleClear(clear);
    vi.advanceTimersByTime(4000);
    cancelVideoStaleClear();
    vi.advanceTimersByTime(10000);
    expect(clear).not.toHaveBeenCalled();
  });

  it('로봇이 사라진 채 유예가 지나면 치운다', () => {
    const clear = vi.fn();
    armVideoStaleClear(clear);
    vi.advanceTimersByTime(5000);
    expect(clear).toHaveBeenCalledTimes(1);
  });

  it('재시도가 반복돼도 마감이 밀리지 않는다 — 첫 실패 시각 기준', () => {
    const clear = vi.fn();
    armVideoStaleClear(clear);
    vi.advanceTimersByTime(3000);
    armVideoStaleClear(clear);
    vi.advanceTimersByTime(2000);
    expect(clear).toHaveBeenCalledTimes(1);
  });

  it('치운 뒤 다시 걸 수 있다', () => {
    const clear = vi.fn();
    armVideoStaleClear(clear);
    vi.advanceTimersByTime(5000);
    armVideoStaleClear(clear);
    vi.advanceTimersByTime(5000);
    expect(clear).toHaveBeenCalledTimes(2);
  });
});
