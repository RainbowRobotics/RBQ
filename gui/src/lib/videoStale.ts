const GRACE_MS = 5000;

let timer: ReturnType<typeof setTimeout> | null = null;

export function armVideoStaleClear(clear: () => void) {
  if (timer) return;
  timer = setTimeout(() => { timer = null; clear(); }, GRACE_MS);
}

export function cancelVideoStaleClear() {
  if (timer) { clearTimeout(timer); timer = null; }
}
