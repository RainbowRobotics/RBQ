let lastAt = 0;
const subs = new Set<(at: number) => void>();
let probe: (() => Promise<number | undefined>) | null = null;

export const keyframeWanted = () => subs.size > 0;

export function noteKeyframe() {
  lastAt = Date.now();
  subs.forEach((f) => f(lastAt));
}

export function setKeyframeProbe(fn: (() => Promise<number | undefined>) | null) { probe = fn; }

const POLL_MS = 100;
export const REVEAL_CAP_MS = 1500;
let pollTimer: ReturnType<typeof setInterval> | null = null;
let lastCount: number | undefined;
function syncPoll() {
  const want = subs.size > 0 && !!probe;
  if (want && !pollTimer) {
    lastCount = undefined;
    pollTimer = setInterval(() => {
      const p = probe; if (!p) return;
      p().then((n) => {
        if (n == null) return;
        if (lastCount != null && n > lastCount) noteKeyframe();
        lastCount = n;
      }).catch(() => {});
    }, POLL_MS);
  } else if (!want && pollTimer) { clearInterval(pollTimer); pollTimer = null; }
}

export function revealOnKeyframe(since: number, cb: () => void, minGapMs = 150, capMs = REVEAL_CAP_MS): () => void {
  let done = false;
  const fire = () => { if (done) return; done = true; cleanup(); cb(); };
  const onKf = (at: number) => { if (at - since >= minGapMs) fire(); };
  const cap = setTimeout(fire, Math.max(0, since + capMs - Date.now()));
  const cleanup = () => { clearTimeout(cap); subs.delete(onKf); syncPoll(); };
  if (lastAt - since >= minGapMs) { fire(); return () => {}; }
  subs.add(onKf); syncPoll();
  return () => { done = true; cleanup(); };
}
