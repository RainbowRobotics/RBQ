import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { idleAbort } from './idleAbort';

beforeEach(() => { vi.useFakeTimers(); });
afterEach(() => { vi.useRealTimers(); });

function body(signal: AbortSignal, every: number, count: number, stallAfter = Infinity) {
  let i = 0;
  return new ReadableStream<Uint8Array>({
    pull(c) {
      if (i >= count) { c.close(); return; }
      if (i >= stallAfter) return new Promise((_, reject) => signal.addEventListener('abort', () => reject(signal.reason)));
      i++;
      return new Promise((r) => setTimeout(() => { c.enqueue(new Uint8Array(1024)); r(); }, every));
    },
  });
}

async function readAll(stream: ReadableStream<Uint8Array>, kick: () => void) {
  const reader = stream.getReader();
  let n = 0;
  for (;;) {
    const { done, value } = await reader.read();
    if (done) return n;
    kick();
    n += value.byteLength;
  }
}

describe('idleAbort', () => {
  it('20 s 마다 조각이 오면 10 분이 걸려도 끝까지 받는다(전체 5 분 상한이었으면 끊겼다)', async () => {
    const ctl = new AbortController();
    const w = idleAbort(ctl, 30_000);
    const box: { n?: number; e?: unknown } = {};
    readAll(body(ctl.signal, 20_000, 30), () => w.kick()).then((n) => { box.n = n; }, (e) => { box.e = e; });
    await vi.advanceTimersByTimeAsync(30 * 20_000 + 1);
    w.stop();
    expect(box.n).toBe(30 * 1024);
    expect(w.fired).toBe(false);
    expect(ctl.signal.aborted).toBe(false);
  });

  it('조각이 멎으면 마지막 조각 30 s 뒤에 끊는다', async () => {
    const ctl = new AbortController();
    const w = idleAbort(ctl, 30_000);
    const box: { n?: number; e?: unknown } = {};
    readAll(body(ctl.signal, 1_000, 30, 3), () => w.kick()).then((n) => { box.n = n; }, (e) => { box.e = e; });
    await vi.advanceTimersByTimeAsync(3_000 + 30_000 - 1);
    expect(w.fired).toBe(false);
    await vi.advanceTimersByTimeAsync(1);
    expect(w.fired).toBe(true);
    expect(box.e).toBeDefined();
    expect(box.n).toBeUndefined();
  });

  it('응답(첫 조각)이 30 s 안에 안 와도 끊는다', async () => {
    const ctl = new AbortController();
    const w = idleAbort(ctl, 30_000);
    await vi.advanceTimersByTimeAsync(30_000);
    expect(w.fired).toBe(true);
    expect(ctl.signal.aborted).toBe(true);
  });
});
