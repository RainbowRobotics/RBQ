import { useEffect } from 'react';
import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { ptzQcApi } from '@/lib/ptzQcApi';
import { whenStreamerReady } from '@/lib/commandBus';
import { isDemo } from '@/lib/demoFlag';
import { isBusy, phaseOf, unseenFinished, type HistoryRow, type QcStatus } from '@/lib/ptzQc';
import { useRobots } from '@/store/robots';
import { useRobot } from '@/store/robot';
import { hasPtzNow } from '@/store/capability';

type QcState = {
  status: QcStatus | null;
  at: number;
  error: string | null;
  history: HistoryRow[];
  historyOf: string | null;
  historyAt: number;
  historyError: string | null;
  sentAt: number;
  watching: boolean;
  markSent: () => void;
};

export const usePtzQc = create<QcState>()((set) => ({
  status: null, at: 0, error: null, history: [], historyOf: null, historyAt: 0, historyError: null, sentAt: 0, watching: false,
  markSent: () => set({ sentAt: Date.now(), watching: true }),
}));

const usePtzQcSeen = create<{ seen: Record<string, string>; closed: Record<string, string> }>()(
  persist(() => ({ seen: {} as Record<string, string>, closed: {} as Record<string, string> }),
    { name: 'rbq-ptz-qc', storage: createJSONStorage(() => AsyncStorage) }),
);

const robotKey = () => useRobots.getState().currentSerial ?? '-';

export function markPtzQcSeen(stem: string) {
  usePtzQcSeen.setState((s) => ({ seen: { ...s.seen, [robotKey()]: stem } }));
}

export function closePtzQcNotice(startedAt: string) {
  usePtzQcSeen.setState((s) => ({ closed: { ...s.closed, [robotKey()]: startedAt } }));
}
export function usePtzQcClosed(): string | null {
  const serial = useRobots((s) => s.currentSerial) ?? '-';
  return usePtzQcSeen((s) => s.closed?.[serial] ?? null);
}

let inflight = false;
export async function refreshPtzQc(opts: { passive?: boolean } = {}): Promise<void> {
  if (inflight) return;
  inflight = true;
  const wasBusy = isBusy(phaseOf(usePtzQc.getState().status, usePtzQc.getState().sentAt));
  try {
    const s = await ptzQcApi.status(opts);
    usePtzQc.setState({ status: s, at: Date.now(), error: null });
    const busy = isBusy(phaseOf(s, usePtzQc.getState().sentAt));
    if (busy && !usePtzQc.getState().watching) usePtzQc.setState({ watching: true });
    if (wasBusy && !busy) {
      usePtzQc.setState({ watching: false });
      void refreshPtzQcHistory();
    }
  } catch (e) {
    usePtzQc.setState({ error: (e as Error)?.message ?? String(e) });
  } finally {
    inflight = false;
  }
}

export async function refreshPtzQcHistory(opts: { passive?: boolean } = {}): Promise<void> {
  const of = robotKey();
  try {
    const r = await ptzQcApi.history(opts);
    if (robotKey() !== of) return;
    usePtzQc.setState({ history: r.reports ?? [], historyOf: of, historyAt: Date.now(), historyError: null });
  } catch (e) {
    if (robotKey() === of) usePtzQc.setState({ historyError: (e as Error)?.message ?? String(e) });
  }
}

const NO_ROWS: HistoryRow[] = [];
export function usePtzQcHistory(): HistoryRow[] {
  const serial = useRobots((s) => s.currentSerial) ?? '-';
  return usePtzQc((s) => (s.historyOf === serial ? s.history : NO_ROWS));
}

export function usePtzQcUnseen(): HistoryRow | null {
  const serial = useRobots((s) => s.currentSerial) ?? '-';
  const seen = usePtzQcSeen((s) => s.seen[serial] ?? null);
  const rows = usePtzQcHistory();
  return unseenFinished(rows, seen);
}

export const usePtzQcDot = () => !!usePtzQcUnseen();

export function usePtzQcBusyWatch(on: boolean): boolean {
  useEffect(() => {
    if (!on) return;
    void refreshPtzQc();
    const id = setInterval(() => { void refreshPtzQc(); }, 5000);
    return () => clearInterval(id);
  }, [on]);
  return usePtzQc((s) => on && Date.now() - s.at < 15000 && isBusy(phaseOf(s.status, s.sentAt)));
}

let bg: ReturnType<typeof setInterval> | null = null;
usePtzQc.subscribe((s) => {
  if (s.watching && !bg) bg = setInterval(() => { if (isDemo() || useRobot.getState().conn === 'connected') void refreshPtzQc(); }, 15000);
  else if (!s.watching && bg) { clearInterval(bg); bg = null; }
});

let lastSerial = useRobots.getState().currentSerial;
useRobots.subscribe((s) => {
  if (s.currentSerial === lastSerial) return;
  lastSerial = s.currentSerial;
  usePtzQc.setState({ status: null, at: 0, error: null, history: [], historyOf: null, historyError: null, sentAt: 0, watching: false });
});

let lastConn = useRobot.getState().conn;
let connSeq = 0;
useRobot.subscribe((s) => {
  if (s.conn === lastConn) return;
  lastConn = s.conn;
  if (s.conn !== 'connected' || isDemo()) return;
  const seq = ++connSeq;
  void whenStreamerReady(30 * 60_000).then((ok) => {
    if (!ok || seq !== connSeq || useRobot.getState().conn !== 'connected' || !hasPtzNow()) return;
    void refreshPtzQc({ passive: true });
    void refreshPtzQcHistory({ passive: true });
  });
});
