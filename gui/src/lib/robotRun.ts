export function robotRunView<T extends { run: number; state: number }>(
  info: T | undefined, startRun: number | null, seen: T | undefined, doneState: number,
): { cur: T | undefined; shown: T | undefined } {
  const now = startRun != null && info && info.run !== startRun ? info : undefined;
  const cur = now ?? seen;
  const shown = cur ? (cur.state === doneState ? cur : undefined)
    : startRun == null && info?.state === doneState ? info : undefined;
  return { cur, shown };
}
