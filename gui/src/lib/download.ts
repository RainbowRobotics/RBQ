import * as FileSystem from 'expo-file-system/legacy';
import * as Sharing from 'expo-sharing';
import { robotAuth } from './auth';
import { File, FileMode } from 'expo-file-system';
import { restBase, MEDIA_ROOT, viaRemote, RemoteUnsupportedError, type MediaKind } from './rest';
import { tunnelDownload, tunnelReady } from './robotTunnel';
import { ensureFreeSpace, freeBytes } from './storage';
import { isNoSpaceError, staleSaved, StorageFullError } from './storageCommon';
import { t } from './i18n';
import { libName } from './bbLibraryCommon';
import { libPut } from './bbLibrary';
import type { LogLine } from '@/types/robot';
import { useLogSave } from '@/store/logSave';
import { useMediaDl } from '@/store/mediaDownload';

const SAVED_PREFIX = /^(blackbox-|systemlog-|realtime-log|media-)/;

async function purgeSaved() {
  const dir = FileSystem.cacheDirectory;
  if (!dir) return;
  const waiting = new Set([useLogSave.getState().share, ...useMediaDl.getState().share]);
  try {
    const found: { uri: string; at: number }[] = [];
    for (const name of await FileSystem.readDirectoryAsync(dir)) {
      if (!SAVED_PREFIX.test(name) || waiting.has(dir + name)) continue;
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

export type CanShare = (uri: string) => boolean;
const always: CanShare = () => true;

async function share(uri: string, mimeType: string, filename: string, UTI: string) {
  try {
    if (await Sharing.isAvailableAsync()) await Sharing.shareAsync(uri, { mimeType, dialogTitle: filename, UTI });
  } catch { }
}

export async function saveLogsAsJsonl(filename: string, lines: LogLine[], canShare: CanShare = always) {
  const text = lines
    .map((l) => JSON.stringify({ timestamp: l.ts, application: l.process, level: l.level, message: l.msg }))
    .join('\n') + '\n';
  const uri = (FileSystem.cacheDirectory || '') + filename;
  await purgeSaved();
  await ensureFreeSpace(text.length * 3);
  await writeGuarded(uri, () => FileSystem.writeAsStringAsync(uri, text));
  if (canShare(uri)) await share(uri, 'application/x-ndjson', filename, 'public.text');
  return uri;
}

export async function robotDownloadAsync(url: string, uri: string, opts: { headers?: Record<string, string> }) {
  if (!viaRemote()) return FileSystem.downloadAsync(url, uri, opts);
  if (!tunnelReady()) throw new RemoteUnsupportedError(url.replace(/^https?:\/\/[^/]+/, ''));
  const f = new File(uri);
  if (f.exists) f.delete();
  f.create();
  const h = f.open(FileMode.WriteOnly);
  try {
    const res = await tunnelDownload(url, { headers: opts.headers }, (b) => h.writeBytes(b));
    const headers: Record<string, string> = {};
    res.headers.forEach((v, k) => { headers[k] = v; });
    return { uri, status: res.status, headers, mimeType: res.headers.get('content-type'), md5: undefined };
  } catch (e) {
    h.close();
    try { f.delete(); } catch { }
    throw e;
  } finally {
    try { h.close(); } catch { }
  }
}

export async function saveSystemlogRaw(ip: string, date: string, expectedBytes = 0, canShare: CanShare = always) {
  const filename = `systemlog-${date}.jsonl`;
  const uri = (FileSystem.cacheDirectory || '') + filename;
  await purgeSaved();
  await ensureFreeSpace(expectedBytes);
  const res = await writeGuarded(uri, () => robotDownloadAsync(
    `${restBase(ip)}/api/systemlog/date?date=${encodeURIComponent(date)}`,
    uri,
    { headers: robotAuth() },
  )).then((r) => checkStatus(r, t('로그를 받지 못했습니다')));
  if (canShare(res.uri)) await share(res.uri, 'application/x-ndjson', filename, 'public.text');
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

export async function saveBlackboxZip(ip: string, dir: string, serial: string, expectedBytes = 0, canShare: CanShare = always) {
  const filename = zipName(dir, serial);
  const uri = (FileSystem.cacheDirectory || '') + filename;
  await purgeSaved();
  await ensureFreeSpace(expectedBytes * 2);
  const res = await writeGuarded(uri, () => robotDownloadAsync(
    `${restBase(ip)}/api/blackbox/zip?path=${encodeURIComponent(dir)}`,
    uri,
    { headers: robotAuth() },
  )).then((r) => checkStatus(r, t('세션 zip 을 만들지 못했습니다')));
  if (canShare(res.uri)) await share(res.uri, 'application/zip', filename, 'public.zip-archive');
  await fileToLibrary(() => libPut(serial, filename, { uri: res.uri }));
  return res.uri;
}

const MEDIA_TYPE: Record<string, { mime: string; uti: string }> = {
  jpg: { mime: 'image/jpeg', uti: 'public.jpeg' },
  jpeg: { mime: 'image/jpeg', uti: 'public.jpeg' },
  mp4: { mime: 'video/mp4', uti: 'public.mpeg-4' },
  wav: { mime: 'audio/wav', uti: 'com.microsoft.waveform-audio' },
};

export async function saveMediaFile(ip: string, kind: MediaKind, path: string, expectedBytes = 0, canShare: CanShare = always) {
  const base = path.split('/').pop() || 'media';
  const filename = `media-${base}`;
  const uri = (FileSystem.cacheDirectory || '') + filename;
  await purgeSaved();
  await ensureFreeSpace(expectedBytes);
  const res = await writeGuarded(uri, () => robotDownloadAsync(
    `${restBase(ip)}${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`,
    uri,
    { headers: robotAuth() },
  )).then((r) => checkStatus(r, t('파일을 받지 못했습니다')));
  const ext = base.split('.').pop()?.toLowerCase() ?? '';
  const type = MEDIA_TYPE[ext] ?? { mime: 'application/octet-stream', uti: 'public.data' };
  if (canShare(res.uri)) await share(res.uri, type.mime, base, type.uti);
  return res.uri;
}

export async function shareSaved(uri: string) {
  if (!(await FileSystem.getInfoAsync(uri)).exists) throw new Error(t('파일이 지워졌습니다 — 다시 저장하세요'));
  const name = (uri.split('/').pop() || '').replace(/^media-/, '');
  const ext = name.split('.').pop()?.toLowerCase() ?? '';
  const type = ext === 'zip' ? { mime: 'application/zip', uti: 'public.zip-archive' }
    : ext === 'jsonl' ? { mime: 'application/x-ndjson', uti: 'public.text' }
    : MEDIA_TYPE[ext] ?? { mime: 'application/octet-stream', uti: 'public.data' };
  await share(uri, type.mime, name, type.uti);
}
