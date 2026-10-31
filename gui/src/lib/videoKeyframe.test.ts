import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { noteKeyframe, revealOnKeyframe, setKeyframeProbe, REVEAL_CAP_MS } from './videoKeyframe';

let clock = 2_000_000_000_000;
beforeEach(() => { clock += 1_000_000; vi.useFakeTimers({ now: clock }); setKeyframeProbe(null); });
afterEach(() => { vi.useRealTimers(); });

describe('revealOnKeyframe', () => {
  it('기준 직후(150 ms 안) 키프레임은 무시하고 그 뒤 첫 키프레임에 걷는다', () => {
    const cb = vi.fn();
    revealOnKeyframe(Date.now(), cb);
    vi.advanceTimersByTime(50); noteKeyframe();
    expect(cb).not.toHaveBeenCalled();
    vi.advanceTimersByTime(200); noteKeyframe();
    expect(cb).toHaveBeenCalledTimes(1);
    noteKeyframe();
    expect(cb).toHaveBeenCalledTimes(1);
  });

  it('신호가 안 오면 상한에 걷는다', () => {
    const cb = vi.fn();
    revealOnKeyframe(Date.now(), cb);
    vi.advanceTimersByTime(REVEAL_CAP_MS - 1);
    expect(cb).not.toHaveBeenCalled();
    vi.advanceTimersByTime(1);
    expect(cb).toHaveBeenCalledTimes(1);
  });

  it('취소하면 부르지 않는다', () => {
    const cb = vi.fn();
    const cancel = revealOnKeyframe(Date.now(), cb);
    cancel();
    vi.advanceTimersByTime(300); noteKeyframe();
    vi.advanceTimersByTime(REVEAL_CAP_MS);
    expect(cb).not.toHaveBeenCalled();
  });

  it('WebRTC 경로: 기다리는 동안 keyFramesDecoded 증가를 키프레임으로 본다', async () => {
    let n = 10;
    const probe = vi.fn(async () => n);
    setKeyframeProbe(probe);
    const cb = vi.fn();
    revealOnKeyframe(Date.now(), cb);
    await vi.advanceTimersByTimeAsync(100);
    expect(cb).not.toHaveBeenCalled();
    n = 11;
    await vi.advanceTimersByTimeAsync(200);
    expect(cb).toHaveBeenCalledTimes(1);
    const calls = probe.mock.calls.length;
    await vi.advanceTimersByTimeAsync(500);
    expect(probe.mock.calls.length).toBe(calls);
  });
});
