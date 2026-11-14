import { screenState, jobLine, type JobBase, type JobSource } from './jobs';
import { useRobot } from './robot';
import { ZMP_CALIB, type ZmpCalibInfo } from '@/types/robot';
import { t } from '@/lib/i18n';

export type ZmpPhase = 'pending' | 'aligning' | 'running' | 'done' | 'failed';

interface ZmpRun {
  startRun: number | null;
  cur: ZmpCalibInfo | null;
  timedOut: boolean;
  lost: boolean;
}

export function zmpPhase(s: ZmpRun): ZmpPhase | null {
  if (s.startRun == null) return null;
  if (s.lost) return 'failed';
  const cur = s.cur;
  if (!cur) return s.timedOut ? 'failed' : 'pending';
  return cur.state === ZMP_CALIB.running ? 'running'
    : cur.state === ZMP_CALIB.done ? 'done'
    : cur.state === ZMP_CALIB.failed ? 'failed'
    : 'aligning';
}

export function zmpBusy(s: ZmpRun): boolean {
  const p = zmpPhase(s);
  return p === 'pending' || p === 'aligning' || p === 'running';
}

const IDLE: ZmpRun = { startRun: null, cur: null, timedOut: false, lost: false };
export const useZmpJob = screenState<ZmpRun>({ ...IDLE }, zmpBusy);

const DEADLINE_MS = 90_000;
let timers: ReturnType<typeof setTimeout>[] = [];
const stopTimers = () => { timers.forEach(clearTimeout); timers = []; };
const lose = () => { stopTimers(); useZmpJob.setState({ lost: true, cur: null }); };

export function zmpBegin() {
  stopTimers();
  useZmpJob.setState({ ...IDLE, startRun: useRobot.getState().robot?.zmp_calib?.run ?? 0 });
  timers = [
    setTimeout(() => { if (!useZmpJob.getState().cur) useZmpJob.setState({ timedOut: true }); }, 8000),
    setTimeout(() => { if (zmpBusy(useZmpJob.getState())) lose(); }, DEADLINE_MS),
  ];
}

useRobot.subscribe((r, p) => {
  if (r.ip !== p.ip) { stopTimers(); useZmpJob.setState({ ...IDLE }); return; }
  const s = useZmpJob.getState(), z = r.robot?.zmp_calib;
  if (s.startRun == null || s.lost || !r.robot) return;
  if (s.cur && zmpBusy(s) && (z?.run ?? 0) < s.cur.run) { lose(); return; }
  if (!z || z === s.cur) return;
  if (s.cur ? z.run !== s.cur.run : z.run === s.startRun) return;
  useZmpJob.setState({ cur: z });
});

export function zmpFailHint(reason: number): string {
  switch (reason) {
    case 1: return t('로봇이 서 있어야 합니다.');
    case 2: return t('로봇이 완전히 멈춘 뒤 다시 실행하세요.');
    case 3: return t('로봇을 평지에 위치시켜 주십시오.');
    case 4: return t('다리 정렬 걸음이 끝나지 않았습니다. 주변 공간을 확인하고 다시 실행하세요.');
    case 5: return t('서 있는 자세(STAND)로 바꾼 뒤 다시 실행하세요.');
    case 6: return t('60초 안에 수렴하지 못했습니다. 바닥 상태를 확인하고 다시 실행하세요.');
    case -1: return t('로봇이 응답하지 않습니다. 연결을 확인하세요.');
    default: return t('로봇을 평지에 정지 상태로 두고 다시 실행하세요.');
  }
}

export function zmpLine(s: ZmpRun & JobBase) {
  const ph = zmpPhase(s);
  const what = t('무게 중심 보정');
  return jobLine(s, {
    run: ph === 'running' ? `${what} ${s.cur?.percent ?? 0}%`
      : ph === 'pending' || ph === 'aligning' ? `${what} — ${t('자세를 정렬하는 중입니다.')}` : null,
    bad: ph === 'failed' ? `${what} — ${zmpFailHint(s.cur ? s.cur.reason : -1)}` : null,
    ok: ph === 'done' ? t('무게중심 보정이 완료되었습니다.') : null,
  });
}

export const zmpJob: JobSource = {
  key: 'zmp', route: '/?resume=zmp', icon: 'wrench', label: () => t('무게 중심 보정 열기'),
  useLine: () => zmpLine(useZmpJob()),
  dismiss: () => useZmpJob.setState({ unseen: false }),
};
