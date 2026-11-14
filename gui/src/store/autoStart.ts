import { screenState, jobLine, type JobBase, type JobSource } from './jobs';
import { useRobot } from './robot';
import { INIT_STATE, INIT_STEPS, type AutostartInfo } from '@/types/robot';
import { failReason } from '@/lib/autostartFail';
import { t } from '@/lib/i18n';

interface AutoStartRun {
  started: boolean;
  pending: boolean;
  sawRun: boolean;
  running: boolean;
  startMs: number | null;
  info: AutostartInfo | null;
  lost: boolean;
}

const INIT: AutoStartRun = { started: false, pending: false, sawRun: false, running: false, startMs: null, info: null, lost: false };
function busy(s: AutoStartRun) { return s.pending || s.running; }

export const useAutoStartJob = screenState<AutoStartRun>({ ...INIT }, busy);

let timer: ReturnType<typeof setTimeout> | null = null;
let stale: ReturnType<typeof setTimeout> | null = null;
const stopTimer = () => { if (timer) clearTimeout(timer); if (stale) clearTimeout(stale); timer = stale = null; };
const STALE_MS = 10_000;
function watchStale(running: boolean) {
  if (stale) clearTimeout(stale);
  stale = running ? setTimeout(() => {
    stale = null;
    if (useAutoStartJob.getState().running) useAutoStartJob.setState({ running: false, pending: false, lost: true });
  }, STALE_MS) : null;
}

function snapshot() {
  const info = useRobot.getState().robot?.autostart ?? null;
  return { info, running: !!info?.running, startMs: info?.start_unix_ms ?? null };
}

export function autoStartFresh() {
  if (busy(useAutoStartJob.getState())) return;
  stopTimer();
  const snap = snapshot();
  useAutoStartJob.setState({ ...INIT, ...snap, sawRun: snap.running });
  watchStale(snap.running);
}

export function autoStartSent() {
  stopTimer();
  useAutoStartJob.setState({ ...snapshot(), started: true, pending: true, sawRun: false });
  timer = setTimeout(() => { timer = null; useAutoStartJob.setState({ pending: false }); }, 8000);
}

useRobot.subscribe((r, p) => {
  if (r.ip !== p.ip) { stopTimer(); useAutoStartJob.setState({ ...INIT }); return; }
  const s = useAutoStartJob.getState();
  const info = r.robot?.autostart ?? null;
  if (info === s.info || (s.open === 0 && !busy(s))) return;
  const running = !!info?.running;
  const fresh = running || (info?.start_unix_ms != null && info.start_unix_ms !== s.startMs);
  useAutoStartJob.setState({ info, running, lost: false, ...(fresh ? { sawRun: true, pending: false } : {}) });
  watchStale(running);
});

const CTRL = INIT_STEPS.indexOf('control');
export function autoStartOk(s: AutoStartRun): boolean {
  const c = s.info?.steps?.[CTRL];
  return s.sawRun && !s.running && (c === INIT_STATE.pass || c === INIT_STATE.warn);
}

export function autoStartLine(s: AutoStartRun & JobBase) {
  const what = t('자동 기동');
  const ok = autoStartOk(s);
  const why = s.info?.steps ? failReason(s.info)?.title : undefined;
  return jobLine(s, {
    run: busy(s) ? t('자동 기동 진행 중') : null,
    bad: ok ? null
      : s.lost ? `${what} — ${t('로봇이 응답하지 않습니다. 연결을 확인하세요.')}`
      : s.sawRun ? `${what} — ${why ?? t('기동에 실패했습니다')}`
      : s.started ? `${what} — ${t('로봇이 응답하지 않습니다. 연결을 확인하세요.')}` : null,
    ok: !ok ? null
      : s.info?.emo_blocked ? `${what} — ${t('E-STOP이 눌려 있어 제어가 시작되지 않았습니다 — 해제하면 바로 시작됩니다.')}`
      : t('자동 기동 완료'),
  });
}

export const autoStartJob: JobSource = {
  key: 'autostart', route: '/hub?resume=autostart', icon: 'power', label: () => t('자동 기동 열기'),
  useLine: () => autoStartLine(useAutoStartJob()),
  dismiss: () => useAutoStartJob.setState({ unseen: false }),
};
