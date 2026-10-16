import { robotAuth } from './auth';
import { isDemo } from './demoFlag';
import { restBase, MEDIA_ROOT, type MediaKind } from './rest';
import { ensureFreeSpace, freeBytes } from './storage';
import { isNoSpaceError, StorageFullError } from './storageCommon';
import { t } from './i18n';
import { libName } from './bbLibraryCommon';
import { libPut } from './bbLibrary';
import type { LogLine } from '@/types/robot';

function downloadBlob(blob: Blob, filename: string) {
  const url = URL.createObjectURL(blob);
  const a = document.createElement('a');
  a.href = url;
  a.download = filename;
  document.body.appendChild(a);
  a.click();
  a.remove();
  URL.revokeObjectURL(url);
}

function robotError(status: number, what: string) {
  return new Error(`${t('로봇 응답')} HTTP ${status} — ${what}`);
}

async function toBlob(res: Response) {
  try {
    return await res.blob();
  } catch (e) {
    if (isNoSpaceError(e)) throw new StorageFullError(await freeBytes());
    throw e;
  }
}

export async function saveLogsAsJsonl(filename: string, lines: LogLine[]) {
  const text =
    lines
      .map((l) => JSON.stringify({ timestamp: l.ts, application: l.process, level: l.level, message: l.msg }))
      .join('\n') + '\n';
  downloadBlob(new Blob([text], { type: 'application/x-ndjson' }), filename);
  return filename;
}

export async function saveSystemlogRaw(ip: string, date: string, expectedBytes = 0) {
  if (isDemo()) throw new Error(t('데모 모드 — 실로봇 파일 다운로드 없음'));
  const filename = `systemlog-${date}.jsonl`;
  await ensureFreeSpace(expectedBytes);
  const res = await fetch(`${restBase(ip)}/api/systemlog/date?date=${encodeURIComponent(date)}`, {
    headers: robotAuth(),
  });
  if (!res.ok) throw robotError(res.status, t('로그를 받지 못했습니다'));
  downloadBlob(await toBlob(res), filename);
  return filename;
}

function zipName(dir: string, serial: string) {
  const [date = '', session = ''] = dir.split(/[/\\]/);
  return libName(serial, date, session);
}

async function fileToLibrary(put: () => Promise<void>) {
  try { await put(); }
  catch (e: any) { throw new Error(`${t('zip 은 저장했지만 보관함에 넣지 못했습니다')} (${e?.message || e})`); }
}

export async function saveBlackboxZip(ip: string, dir: string, serial: string, expectedBytes = 0) {
  if (isDemo()) throw new Error(t('데모 모드 — 실로봇 파일 다운로드 없음'));
  const filename = zipName(dir, serial);
  await ensureFreeSpace(expectedBytes);
  const res = await fetch(`${restBase(ip)}/api/blackbox/zip?path=${encodeURIComponent(dir)}`, {
    headers: robotAuth(),
  });
  if (!res.ok) throw robotError(res.status, t('세션 zip 을 만들지 못했습니다'));
  const blob = await toBlob(res);
  downloadBlob(blob, filename);
  await fileToLibrary(() => libPut(serial, filename, { blob }));
  return filename;
}

export async function saveMediaFile(ip: string, kind: MediaKind, path: string, expectedBytes = 0) {
  if (isDemo()) throw new Error(t('데모 모드 — 실로봇 파일 다운로드 없음'));
  const filename = path.split('/').pop() || 'media';
  await ensureFreeSpace(expectedBytes);
  const res = await fetch(`${restBase(ip)}${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`, {
    headers: robotAuth(),
  });
  if (!res.ok) throw robotError(res.status, t('파일을 받지 못했습니다'));
  downloadBlob(await toBlob(res), filename);
  return filename;
}
