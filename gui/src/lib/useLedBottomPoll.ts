import { useCallback, useEffect, useRef, useState } from 'react';
import { useRobot } from '@/store/robot';
import { useLedBottom } from '@/store/ledBottom';
import { actions } from '@/lib/rest';
import { useFocusedInterval } from '@/lib/useFocusedInterval';
import { normalizeLedBottom, ledUsable, ledReportedAt, LED_SIDES, LED_POLL_MS, type LedRead } from '@/lib/ledBottom';
const LED_UNSUPPORTED_POLL_MS = 30_000;
export function statusOf(e: unknown): number {
  const s = (e as { status?: unknown } | null)?.status;
  return typeof s === 'number' ? s : 0;
}
export function useLedBottomPoll(enabled = true) {
  const ip = useRobot((s) => s.ip);
  const remember = useLedBottom((s) => s.remember);
  const setLive = useLedBottom((s) => s.setLive);
  const bindIp = useLedBottom((s) => s.bindIp);
  const [read, setRead] = useState<LedRead | null | undefined>(undefined);
  const [unsupported, setUnsupported] = useState(false);
  const [, setTick] = useState(0);
  const gen = useRef(0);
  const busy = useRef(false);
  useEffect(() => { gen.current++; busy.current = false; setRead(undefined); setUnsupported(false); bindIp(ip); }, [ip, bindIp]);
  const refresh = useCallback(async (): Promise<LedRead | null> => {
    const g = gen.current;
    let next: LedRead | null = null;
    let missing = false;
    try {
      const st = normalizeLedBottom(await actions.ledBottom(ip));
      next = st ? { st, receivedAt: Date.now() } : null;
    } catch (e) {
      missing = statusOf(e) === 404;
    }
    if (g !== gen.current) return null;
    setRead(next);
    setLive(next);
    if (missing) setUnsupported(true);
    else if (next) setUnsupported(false);
    if (ledUsable(next, Date.now())) for (const sd of LED_SIDES) remember(sd, next.st[sd], 'robot', ledReportedAt(next));
    return next;
  }, [ip, remember, setLive]);
  const lastTry = useRef(0);
  useFocusedInterval(() => {
    setTick((n) => n + 1);
    if (busy.current) return;
    if (unsupported && Date.now() - lastTry.current < LED_UNSUPPORTED_POLL_MS) return;
    lastTry.current = Date.now();
    busy.current = true;
    const g = gen.current;
    refresh().finally(() => { if (g === gen.current) busy.current = false; });
  }, LED_POLL_MS, enabled, () => { gen.current++; busy.current = false; setRead(undefined); });
  return { read, unsupported, refresh };
}
