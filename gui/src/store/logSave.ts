import { screenState, jobLine, type JobSource } from './jobs';
import { t } from '@/lib/i18n';

export const useLogSave = screenState({
  saving: false,
  what: '',
  err: '',
  done: null as string | null,
  share: null as string | null,
}, (s) => s.saving);

export function logSaveCanShare(uri: string) {
  if (useLogSave.getState().open > 0) return true;
  useLogSave.setState({ share: uri });
  return false;
}

type S = ReturnType<typeof useLogSave.getState>;
export function logSaveLine(s: S) {
  return jobLine(s, {
    run: s.saving ? `${s.what} ${t('저장 중…')}` : null,
    bad: s.err ? `${s.what} ${t('저장 실패')} — ${s.err}` : null,
    ok: s.done ? `${s.what} ${t('저장했습니다')}${s.share ? ` — ${t('눌러서 공유')}` : ''}` : null,
  });
}

export const logSaveJob: JobSource = {
  key: 'logSave', route: '/log', icon: 'log', label: () => t('로그 열기'),
  useLine: () => logSaveLine(useLogSave()),
  dismiss: () => useLogSave.setState({ unseen: false, done: null, err: '', share: null }),
};
