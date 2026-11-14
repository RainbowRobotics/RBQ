import { createDownloadResumable, deleteAsync, getInfoAsync, moveAsync, readAsStringAsync, writeAsStringAsync } from 'expo-file-system/legacy';
import { freeBytes } from './storage';
import { checkFree, isNoSpaceError, StorageFullError } from './storageCommon';
import { t } from './i18n';

export type DlResult = { size: number } | { status: number };
export interface FileDl { cancel: () => void; done: Promise<DlResult> }

export function downloadToFile(
  url: string,
  uri: string,
  key: string,
  onProgress?: (written: number, total: number) => void,
): FileDl {
  const part = `${uri}.part`;
  const src = `${part}.src`;
  let cancelled = false;
  let task: ReturnType<typeof createDownloadResumable> | null = null;
  const drop = async () => {
    await deleteAsync(part, { idempotent: true }).catch(() => {});
    await deleteAsync(src, { idempotent: true }).catch(() => {});
  };

  const run = async (allowResume: boolean): Promise<DlResult> => {
    let from = 0;
    if (allowResume) {
      const info = await getInfoAsync(part).catch(() => null);
      const prev = await readAsStringAsync(src).catch(() => null);
      if (info?.exists && info.size && prev === key) from = info.size;
    }
    if (!from) await drop();
    await writeAsStringAsync(src, key);
    if (cancelled) throw new Error('cancelled');

    let expected = 0;
    let checked = false;
    let noSpace: Error | null = null;
    task = createDownloadResumable(url, part, {}, (p) => {
      expected = p.totalBytesExpectedToWrite;
      onProgress?.(p.totalBytesWritten, expected);
      if (!checked && expected > 0) {
        checked = true;
        freeBytes().then((free) => {
          try { checkFree(free, expected - p.totalBytesWritten); }
          catch (e) { noSpace = e as Error; task?.cancelAsync().catch(() => {}); }
        });
      }
    }, from ? String(from) : undefined);

    let r: Awaited<ReturnType<typeof task.downloadAsync>>;
    try {
      r = await task.downloadAsync();
    } catch (e) {
      if (isNoSpaceError(e)) { await drop(); throw new StorageFullError(await freeBytes()); }
      if (cancelled) throw new Error('cancelled');
      throw new Error(`${t('다운로드가 끊겼습니다 — 다시 누르면 받은 데부터 이어 받습니다')} (${(e as Error)?.message ?? e})`);
    }
    if (noSpace) { await drop(); throw noSpace; }
    if (cancelled || !r) throw new Error('cancelled');
    if (from && (r.status === 200 || r.status === 416)) { await drop(); return run(false); }
    if (r.status !== 200 && r.status !== 206 && r.status !== 0) { await drop(); return { status: r.status }; }
    const info = await getInfoAsync(part);
    if (!info.exists || !info.size) { await drop(); throw new Error(t('다운로드 파일이 비어 있습니다')); }
    if (expected > 0 && info.size !== expected) {
      if (info.size > expected) await drop();
      throw new Error(`${t('다운로드가 끊겼습니다 — 다시 누르면 받은 데부터 이어 받습니다')} (${info.size}/${expected} B)`);
    }
    await deleteAsync(uri, { idempotent: true });
    await moveAsync({ from: part, to: uri });
    await deleteAsync(src, { idempotent: true }).catch(() => {});
    return { size: info.size };
  };

  return {
    cancel: () => { cancelled = true; task?.cancelAsync().catch(() => {}); void drop(); },
    done: run(true),
  };
}
