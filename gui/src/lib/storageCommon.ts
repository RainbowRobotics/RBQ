import { t } from './i18n';

const RESERVE_BYTES = 100 * 1024 * 1024;

const fmtMB = (n: number) => `${Math.max(0, Math.round(n / 1048576))}MB`;

export class StorageFullError extends Error {
  constructor(free: number | null, need = 0) {
    const detail = [
      need > 0 ? `${t('필요')} ${fmtMB(need)}` : '',
      free !== null ? `${t('남은 공간')} ${fmtMB(free)}` : '',
    ].filter(Boolean).join(', ');
    super(t('기기 저장공간이 부족합니다{d}. 기기에서 파일을 지우고 다시 시도하세요.')
      .replace('{d}', detail ? ` — ${detail}` : ''));
    this.name = 'StorageFullError';
  }
}

export function isNoSpaceError(e: unknown) {
  if ((e as Error)?.name === 'StorageFullError') return true;
  return /ENOSPC|No space left|not enough (free )?space|insufficient (disk )?space|QuotaExceeded/i
    .test(String((e as Error)?.message ?? e));
}

export function checkFree(free: number | null, need: number) {
  if (free !== null && free < need + RESERVE_BYTES) throw new StorageFullError(free, need);
}

export function staleSaved<T extends { at: number }>(files: T[]): T[] {
  if (files.length <= 1) return [];
  const newest = files.reduce((a, b) => (b.at > a.at ? b : a));
  return files.filter((f) => f !== newest);
}
