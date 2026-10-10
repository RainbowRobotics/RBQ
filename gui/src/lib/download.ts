import * as FileSystem from 'expo-file-system/legacy';
import * as Sharing from 'expo-sharing';
import { robotAuth } from './auth';
import { restBase, MEDIA_ROOT, type MediaKind } from './rest';
import { ensureFreeSpace, freeBytes } from './storage';
import { isNoSpaceError, staleSaved, StorageFullError } from './storageCommon';
import { t } from './i18n';
import { libName } from './bbLibraryCommon';
import { libPut } from './bbLibrary';
import type { LogLine } from '@/types/robot';

const SAVED_PREFIX = /^(blackbox-|systemlog-|realtime-log|media-)/;

async function purgeSaved() {
  const dir = FileSystem.cacheDirectory;
  if (!dir) return;
  try {
    const found: { uri: string; at: number }[] = [];
    for (const name of await FileSystem.readDirectoryAsync(dir)) {
      if (!SAVED_PREFIX.test(name)) continue;
      const info = await FileSystem.getInfoAsync(dir + name);
      if (info.exists && !info.isDirectory) found.push({ uri: info.uri, at: info.modificationTime ?? 0 });
    }
    for (const f of staleSaved(found)) await FileSystem.deleteAsync(f.uri, { idempotent: true }).catch(() => {});
  } catch { }
}

async function writeGuarded<T>(uri: string, write: () => Promise<T>): Promise<T> {
  try {
    return await write();
  } catch (e) {
    await FileSystem.deleteAsync(uri, { idempotent: true }).catch(() => {});
    if (isNoSpaceError(e)) throw new StorageFullError(await freeBytes());
    throw e;
  }
}

async function checkStatus(res: FileSystem.FileSystemDownloadResult, what: string) {
  if (res.status < 300) return res;
  await FileSystem.deleteAsync(res.uri, { idempotent: true }).catch(() => {});
  throw new Error(`${t('로봇 응답')} HTTP ${res.status} — ${what}`);
}

async function share(uri: string, mimeType: string, filename: string, UTI: string) {
  try {
    if (await Sharing.isAvailableAsync()) await Sharing.shareAsync(uri, { mimeType, dialogTitle: filename, UTI });
  } catch { }
}

export async function saveLogsAsJsonl(filename: string, lines: LogLine[]) {
  const text = lines
    .map((l) => JSON.stringify({ timestamp: l.ts, application: l.process, level: l.level, message: l.msg }))
    .join('\n') + '\n';
  const uri = (FileSystem.cacheDirectory || '') + filename;
  await purgeSaved();
  await ensureFreeSpace(text.length * 3);
  await writeGuarded(uri, () => FileSystem.writeAsStringAsync(uri, text));
  await share(uri, 'application/x-ndjson', filename, 'public.text');
  return uri;
}

export async function saveSystemlogRaw(ip: string, date: string, expectedBytes = 0) {
  const filename = `systemlog-${date}.jsonl`;
  const uri = (FileSystem.cacheDirectory || '') + filename;
  await purgeSaved();
  await ensureFreeSpace(expectedBytes);
  const res = await writeGuarded(uri, () => FileSystem.downloadAsync(
    `${restBase(ip)}/api/systemlog/date?date=${encodeURIComponent(date)}`,
    uri,
    { headers: robotAuth() },
  )).then((r) => checkStatus(r, t('로그를 받지 못했습니다')));
  await share(res.uri, 'application/x-ndjson', filename, 'public.text');
  return res.uri;
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
  const filename = zipName(dir, serial);
  const uri = (FileSystem.cacheDirectory || '') + filename;
  await purgeSaved();
  await ensureFreeSpace(expectedBytes * 2);
  const res = await writeGuarded(uri, () => FileSystem.downloadAsync(
    `${restBase(ip)}/api/blackbox/zip?path=${encodeURIComponent(dir)}`,
    uri,
    { headers: robotAuth() },
  )).then((r) => checkStatus(r, t('세션 zip 을 만들지 못했습니다')));
  await share(res.uri, 'application/zip', filename, 'public.zip-archive');
  await fileToLibrary(() => libPut(serial, filename, { uri: res.uri }));
  return res.uri;
}

const MEDIA_TYPE: Record<string, { mime: string; uti: string }> = {
  jpg: { mime: 'image/jpeg', uti: 'public.jpeg' },
  jpeg: { mime: 'image/jpeg', uti: 'public.jpeg' },
  mp4: { mime: 'video/mp4', uti: 'public.mpeg-4' },
  wav: { mime: 'audio/wav', uti: 'com.microsoft.waveform-audio' },
};

export async function saveMediaFile(ip: string, kind: MediaKind, path: string, expectedBytes = 0) {
  const base = path.split('/').pop() || 'media';
  const filename = `media-${base}`;
  const uri = (FileSystem.cacheDirectory || '') + filename;
  await purgeSaved();
  await ensureFreeSpace(expectedBytes);
  const res = await writeGuarded(uri, () => FileSystem.downloadAsync(
    `${restBase(ip)}${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`,
    uri,
    { headers: robotAuth() },
  )).then((r) => checkStatus(r, t('파일을 받지 못했습니다')));
  const ext = base.split('.').pop()?.toLowerCase() ?? '';
  const type = MEDIA_TYPE[ext] ?? { mime: 'application/octet-stream', uti: 'public.data' };
  await share(res.uri, type.mime, base, type.uti);
  return res.uri;
}
