export function idleAbort(ctl: AbortController, ms: number) {
  let timer: ReturnType<typeof setTimeout> | undefined;
  const w = {
    fired: false,
    kick() { clearTimeout(timer); timer = setTimeout(() => { w.fired = true; ctl.abort(); }, ms); },
    stop() { clearTimeout(timer); },
  };
  w.kick();
  return w;
}
