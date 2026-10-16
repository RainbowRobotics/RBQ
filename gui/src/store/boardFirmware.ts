import { create } from 'zustand';
import { rest } from '@/lib/rest';
import { isDemo, onDemoFwChange } from '@/lib/demo';
import { parseFwStatus, type FwStatus } from '@/lib/boardFirmware';
import { useRobot } from './robot';
import { useFeatures } from './capability';

type BoardFwState = {
  status: FwStatus | null;
  at: number | null;
  error: string | null;
  checkedAt: number | null;
  panelOpen: boolean;
  setPanelOpen: (v: boolean) => void;
  dismissed: Record<string, true>;
  dismiss: (key: string) => void;
  powerOffRun: number | null;
  sent: { afterRunId: number; powerDown: boolean } | null;
  noteSent: (afterRunId: number, powerDown: boolean) => void;
  apply: (s: FwStatus) => void;
  fail: (e: string) => void;
  reset: () => void;
};

const EMPTY = { status: null, at: null, error: null, checkedAt: null, dismissed: {}, powerOffRun: null, sent: null };

export const useBoardFw = create<BoardFwState>((set, get) => ({
  ...EMPTY,
  panelOpen: false,
  setPanelOpen: (panelOpen) => set({ panelOpen }),
  dismiss: (key) => set({ dismissed: { ...get().dismissed, [key]: true } }),
  noteSent: (afterRunId, powerDown) => set({ sent: { afterRunId, powerDown } }),
  apply: (s) => {
    const { status: prev, sent } = get();
    let { powerOffRun } = get();
    const checked = prev != null && s.checkSeq > prev.checkSeq;
    if (s.run.stage === 'power_off') powerOffRun = s.run.id;
    const mine = sent && s.run.id > sent.afterRunId;
    if (mine && sent.powerDown) powerOffRun = s.run.id;
    set({ status: s, at: Date.now(), error: null, powerOffRun, ...(mine ? { sent: null } : null), ...(checked ? { checkedAt: Date.now() } : null) });
  },
  fail: (error) => set({ error }),
  reset: () => set(EMPTY),
}));

export const usePowerOffHint = () => useBoardFw((s) => !!s.status && s.powerOffRun === s.status.run.id);

let gen = 0;
let inflight: Promise<void> | null = null;

const canFetch = () => useRobot.getState().conn === 'connected' && !!useFeatures.getState().features?.fw_update;

export function refreshBoardFw(): Promise<void> {
  if (inflight) return inflight;
  if (!canFetch()) return Promise.resolve();
  const my = gen;
  const p: Promise<void> = rest.boardFw(useRobot.getState().ip)
    .then((raw) => {
      if (my !== gen) return;
      const s = parseFwStatus(raw);
      if (s) useBoardFw.getState().apply(s);
      else useBoardFw.getState().fail('bad response');
    })
    .catch((e: unknown) => { if (my === gen) useBoardFw.getState().fail(e instanceof Error ? e.message : String(e)); })
    .finally(() => { if (inflight === p) inflight = null; });
  inflight = p;
  return p;
}

const SLOW_MS = 10_000;
const BUSY_MS = 5_000;
let slowTimer: ReturnType<typeof setInterval> | null = null;
function slowPoll(on: boolean) {
  if (!on) { if (slowTimer) clearInterval(slowTimer); slowTimer = null; return; }
  if (slowTimer) return;
  slowTimer = setInterval(() => {
    const st = useBoardFw.getState();
    if (st.panelOpen || !canFetch()) return;
    const s = st.status;
    const moving = !s || s.checkSeq === 0 || s.busy !== 'none' || s.run.state === 'running' || s.run.state === 'checking';
    if (moving || Date.now() - (st.at ?? 0) >= SLOW_MS - 1000) void refreshBoardFw();
  }, BUSY_MS);
}

let prevConn = useRobot.getState().conn;
useRobot.subscribe((s) => {
  if (s.conn === prevConn) return;
  prevConn = s.conn;
  if (s.conn !== 'connected') { gen++; inflight = null; slowPoll(false); useBoardFw.getState().reset(); }
  else { slowPoll(true); void refreshBoardFw(); }
});

useFeatures.subscribe((s, prev) => {
  if (s.features?.fw_update && !prev.features?.fw_update) void refreshBoardFw();
});

if (useRobot.getState().conn === 'connected') { slowPoll(true); void refreshBoardFw(); }

onDemoFwChange(() => { if (isDemo()) void refreshBoardFw(); });

(globalThis as { __rbqBoardFw?: unknown }).__rbqBoardFw = useBoardFw;
