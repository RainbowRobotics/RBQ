import { screenState, jobLine, type JobSource } from './jobs';
import { useRobot } from './robot';
import { robotRunView } from '@/lib/robotRun';
import {
  DQ_CHECK, JOINT_COMM, GAIT_CHECK,
  type RobotStatus, type DqCheckInfo, type JointCommCheckInfo, type GaitCheckInfo,
} from '@/types/robot';
import type { CameraCheck } from '@/lib/rest';
import { t } from '@/lib/i18n';

export const DRIFT_FB_WARN_CM = 10;
export const DRIFT_LR_WARN_CM = 5;
export const commBad = (r: JointCommCheckInfo | undefined, i: number) =>
  !!r && r.measured && (r.rx_hz_min[i] < r.hz_warn || r.gap_max_ms[i] > r.gap_warn_ms);

type RunInfo = { run: number; state: number; percent: number };
type Verdict = { bad: boolean; text: string };
type Stage<T extends RunInfo> = {
  no: string; title: string;
  pick: (r: RobotStatus | undefined) => T | undefined;
  active: number[]; done: number; failed: number;
  verdict: (r: T) => Verdict;
};
const stage = <T extends RunInfo>(s: Stage<T>) => s;
const nOf = (key: string, n: number) => t(key).replace('{n}', String(n));

export const RUN_STAGES = {
  dq: stage<DqCheckInfo>({
    no: '②', title: '모터 캘리브레이션 상태', pick: (r) => r?.dq_check,
    active: [DQ_CHECK.running], done: DQ_CHECK.done, failed: DQ_CHECK.failed,
    verdict: (r) => {
      const bad = r.result.filter((x) => x === 1).length, warn = r.result.filter((x) => x === 4).length;
      const text = [bad && nOf('불량 {n}개', bad), warn && nOf('주의 {n}개', warn)].filter(Boolean).join(' · ');
      return { bad: !!text, text: text || t('통과') };
    },
  }),
  comm: stage<JointCommCheckInfo>({
    no: '④', title: '관절 통신 상태', pick: (r) => r?.joint_comm_check,
    active: [JOINT_COMM.running], done: JOINT_COMM.done, failed: JOINT_COMM.failed,
    verdict: (r) => {
      if (!r.measured) return { bad: false, text: t('완료') };
      const n = r.rx_hz_min.filter((_, i) => commBad(r, i)).length;
      return n ? { bad: true, text: nOf('주의 {n}개', n) } : { bad: false, text: t('통과') };
    },
  }),
  drift: stage<GaitCheckInfo>({
    no: '⑤', title: '제자리 걸음 · 밀림', pick: (r) => (r?.gait_check && (r.gait_check.mode ?? 0) === 0 ? r.gait_check : undefined),
    active: [GAIT_CHECK.entering, GAIT_CHECK.walking, GAIT_CHECK.stopping], done: GAIT_CHECK.done, failed: GAIT_CHECK.failed,
    verdict: (r) => (Math.abs(r.x_mm) >= DRIFT_FB_WARN_CM * 10 || Math.abs(r.y_mm) >= DRIFT_LR_WARN_CM * 10
      ? { bad: true, text: t('주의') } : { bad: false, text: t('통과') }),
  }),
};
export type RunKey = keyof typeof RUN_STAGES;
export type RunInfoOf = { dq: DqCheckInfo; comm: JointCommCheckInfo; drift: GaitCheckInfo };
const KEYS = Object.keys(RUN_STAGES) as RunKey[];
const stageOf = (k: RunKey) => RUN_STAGES[k] as unknown as Stage<RunInfo>;

export type RunMark = { startRun: number; noReply: boolean; seen?: RunInfo };

export function runView<T extends RunInfo>(k: RunKey, m: RunMark | null, info: T | undefined) {
  const st = stageOf(k);
  const { cur, shown } = robotRunView(info, m?.startRun ?? null, m?.seen as T | undefined, st.done);
  const noReply = !!m?.noReply;
  const running = !!m && !noReply && (!cur || st.active.includes(cur.state));
  return { cur, shown, noReply, running, failed: noReply || cur?.state === st.failed, done: shown?.state === st.done };
}
const runningOf = (k: RunKey, m: RunMark | null) => runView(k, m, m?.seen).running;

export const useRobotCheckJob = screenState({
  dq: null as RunMark | null,
  comm: null as RunMark | null,
  drift: null as RunMark | null,
  last: null as RunKey | null,
}, (s) => KEYS.some((k) => runningOf(k, s[k])));
type S = ReturnType<typeof useRobotCheckJob.getState>;
const setMark = (k: RunKey, m: RunMark | null) => useRobotCheckJob.setState({ [k]: m } as Partial<S>);

const NO_REPLY_MS = 5000;
const STALE_MS = 10000;
const timers: Partial<Record<RunKey, ReturnType<typeof setTimeout>>> = {};
function arm(k: RunKey, ms: number) {
  clearTimeout(timers[k]);
  timers[k] = setTimeout(() => {
    const m = useRobotCheckJob.getState()[k];
    if (m && runningOf(k, m)) setMark(k, { ...m, noReply: true });
  }, ms);
}

export function beginRun(k: RunKey, send: () => void) {
  const startRun = stageOf(k).pick(useRobot.getState().robot)?.run ?? 0;
  useRobotCheckJob.setState({ [k]: { startRun, noReply: false }, last: k } as Partial<S>);
  arm(k, NO_REPLY_MS);
  send();
}

useRobot.subscribe((r, p) => {
  if (r.ip !== p.ip) {
    for (const k of KEYS) clearTimeout(timers[k]);
    useRobotCheckJob.setState({ dq: null, comm: null, drift: null, last: null, unseen: false });
    useCameraCheck.setState({ result: null, error: '', unseen: false });
    return;
  }
  const s = useRobotCheckJob.getState();
  for (const k of KEYS) {
    const m = s[k];
    if (!m || !runningOf(k, m)) continue;
    const { cur } = robotRunView(stageOf(k).pick(r.robot), m.startRun, m.seen, stageOf(k).done);
    if (cur && cur !== m.seen) { setMark(k, { ...m, seen: cur }); arm(k, STALE_MS); }
  }
});

export function robotCheckLine(s: S) {
  const name = (k: RunKey) => `${t('점검')} ${RUN_STAGES[k].no} ${t(RUN_STAGES[k].title)}`;
  const k = KEYS.find((x) => runningOf(x, s[x]));
  let run: string | null = null;
  if (k) {
    const cur = s[k]!.seen;
    const ask = k === 'dq' && cur && (cur as DqCheckInfo).ask_joint >= 0 ? (cur as DqCheckInfo) : undefined;
    run = ask ? `${name(k)} — ${t('응답이 필요합니다 ({n}초) — 눌러서 보기').replace('{n}', String(ask.ask_left))}`
      : `${name(k)} ${t('측정 중')} ${cur?.percent ?? 0}%`;
  }
  let res: Verdict | null = null;
  if (!k && s.last && s[s.last]) {
    const v = runView(s.last, s[s.last], s[s.last]?.seen);
    res = v.failed ? { bad: true, text: t('실패') } : v.done && v.shown ? stageOf(s.last).verdict(v.shown) : null;
  }
  const line = res && s.last ? `${name(s.last)} — ${res.text}` : null;
  return jobLine(s, { run, bad: res?.bad ? line : null, ok: res && !res.bad ? line : null });
}

export const robotCheckJob: JobSource = {
  key: 'robotCheck', route: '/maintenance?sec=chk', icon: 'pulse', label: () => t('로봇 점검 열기'),
  useLine: () => robotCheckLine(useRobotCheckJob()),
  dismiss: () => useRobotCheckJob.setState({ unseen: false }),
};

export const useCameraCheck = screenState({
  result: null as CameraCheck | null,
  error: '',
}, (s) => s.result?.state === 'running');
type C = ReturnType<typeof useCameraCheck.getState>;

export function cameraCheckLine(s: C) {
  const r = s.result;
  const name = t('카메라 시야 · FPS');
  const pct = r && r.duration_s > 0 ? Math.min(100, Math.round((r.elapsed_s / r.duration_s) * 100)) : 0;
  const fails = r?.state === 'done' ? r.sensors.filter((x) => x.verdict === 'fail').length : 0;
  return jobLine(s, {
    run: r?.state === 'running' ? `${name} ${t('측정 중')} ${pct}%` : null,
    bad: r?.state === 'failed' || r?.state === 'aborted' ? `${name} — ${t('실패')}`
      : fails ? `${name} — ${nOf('불량 {n}개', fails)}` : null,
    ok: r?.state === 'done' ? `${name} — ${t('통과')}` : null,
  });
}

export const cameraCheckJob: JobSource = {
  key: 'cameraCheck', route: '/maintenance?sec=chk', icon: 'camera', label: () => t('로봇 점검 열기'),
  useLine: () => cameraCheckLine(useCameraCheck()),
  dismiss: () => useCameraCheck.setState({ unseen: false }),
};
