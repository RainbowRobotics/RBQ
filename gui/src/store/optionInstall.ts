import { screenState, jobLine, type JobSource } from './jobs';
import type { OtaFile, OtaHandle, OtaProgress } from '@/lib/ota';
import type { OptionFile } from '@/lib/optionPackage';
import { t } from '@/lib/i18n';
import { useRobot } from './robot';

export interface OptionRowFile { file: OtaFile; opt: OptionFile; cached: boolean }

export const useOptionJob = screenState({
  picked: null as OptionRowFile | null,
  busy: null as string | null,
  busyTitle: '',
  prog: null as OtaProgress | null,
  err: null as string | null,
  done: null as string | null,
  removed: null as string | null,
  installed: null as string | null,
}, (s) => s.busy != null);

export const optionRefs = { handle: { current: null as OtaHandle | null } };

useRobot.subscribe((r, p) => {
  if (r.ip === p.ip || useOptionJob.getState().busy) return;
  useOptionJob.setState({ done: null, err: null, removed: null, installed: null, unseen: false });
});

type S = ReturnType<typeof useOptionJob.getState>;
export function optionLine(s: S) {
  const p = s.prog;
  const run = s.busy === 'remove' ? t('옵션 제거하는 중…')
    : s.busy ? `${s.busyTitle} ${p?.phase === 'applying' ? t('로봇이 옵션을 설치하는 중') : p?.phase === 'transfer' ? `${t('보내는 중')} ${p.percentage}%` : t('연결 중…')}`
    : null;
  return jobLine(s, { run, bad: s.err ? `${t('옵션')} — ${s.err}` : null, ok: s.done });
}

export const optionJob: JobSource = {
  key: 'option', route: '/update?tab=opt', icon: 'box', label: () => t('옵션 열기'),
  useLine: () => optionLine(useOptionJob()),
  dismiss: () => useOptionJob.setState({ unseen: false }),
};
