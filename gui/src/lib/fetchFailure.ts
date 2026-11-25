const CANCELLED = /^fetch failed: (The operation was aborted\.|Fetch request has been canceled)/;

export function isFetchFailure(e: unknown): boolean {
  if (!(e instanceof Error)) return false;
  if (e.name === 'AbortError' || CANCELLED.test(e.message)) return false;
  return e instanceof TypeError || /^fetch failed:/.test(e.message);
}
