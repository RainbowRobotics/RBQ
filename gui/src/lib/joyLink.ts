export const ID_JOY_LINK = 34;
export const JOY_BYTES = 40;
export const JOY_V2_BYTES = 48;
export const JOY_LINK_STATS_BYTES = 24;

export type JoyLinkStats = {
  received: number;
  accepted: number;
  lost: number;
  reordered: number;
  late: number;
  delayMs: number;
};

export function parseJoyLinkStats(dv: DataView, off: number): JoyLinkStats {
  return {
    received: dv.getUint32(off, true),
    accepted: dv.getUint32(off + 4, true),
    lost: dv.getUint32(off + 8, true),
    reordered: dv.getUint32(off + 12, true),
    late: dv.getUint32(off + 16, true),
    delayMs: dv.getInt32(off + 20, true),
  };
}

export function monoMs(): number {
  const now = typeof performance !== 'undefined' && performance.now ? performance.now() : Date.now();
  return Math.floor(now) >>> 0;
}

export function withJoyTail(frame40: ArrayBuffer, seq: number, sentMs: number): ArrayBuffer {
  const out = new ArrayBuffer(JOY_V2_BYTES);
  new Uint8Array(out).set(new Uint8Array(frame40, 0, JOY_BYTES));
  const dv = new DataView(out);
  dv.setUint32(JOY_BYTES, seq >>> 0, true);
  dv.setUint32(JOY_BYTES + 4, sentMs >>> 0, true);
  return out;
}

export const JOY_BLOCK_WARN_MS = 300;
export const JOY_SILENT_WARN_MS = 1000;
export const JOY_BAD_WINDOW_MS = 2000;
export const JOY_BAD_RATIO = 0.1;

export class JoyLinkHealth {
  private pushingSince = 0;
  private blockedSince = 0;
  private statsAt = 0;
  private hist: { at: number; s: JoyLinkStats }[] = [];

  tick(now: number, blocked: boolean, v2: boolean): boolean {
    if (!this.pushingSince) this.pushingSince = now;
    if (!blocked) this.blockedSince = 0;
    else if (!this.blockedSince) this.blockedSince = now;
    if (this.blockedSince && now - this.blockedSince >= JOY_BLOCK_WARN_MS) return true;
    return v2 && now - Math.max(this.statsAt, this.pushingSince) >= JOY_SILENT_WARN_MS;
  }

  idle() {
    this.pushingSince = 0;
    this.blockedSince = 0;
  }

  stats(now: number, s: JoyLinkStats): boolean {
    this.statsAt = now;
    this.hist.push({ at: now, s });
    while (this.hist.length > 1 && now - this.hist[1].at >= JOY_BAD_WINDOW_MS) this.hist.shift();
    const o = this.hist[0].s;
    const bad = s.late - o.late + (s.reordered - o.reordered) + (s.lost - o.lost);
    const total = s.received - o.received + (s.lost - o.lost);
    return total >= 10 && bad / total >= JOY_BAD_RATIO;
  }
}
