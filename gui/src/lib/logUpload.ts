import * as FileSystem from 'expo-file-system/legacy';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { Buffer } from 'buffer';
import { Deflate } from 'pako';
import { robotAuth } from './auth';
import { restBase } from './rest';
import { freeBytes } from './storage';
import { isNoSpaceError, StorageFullError } from './storageCommon';
import {
  objectPath, uploadUrl, uploadHeaders, fmtBytes, shouldQueue, PENDING_KEY,
  type UploadStage, type Pending,
} from './logUploadCommon';

const CHUNK = 3 * 512 * 1024;

async function gzipFile(srcUri: string, dstUri: string, onStage?: UploadStage) {
  const info = await FileSystem.getInfoAsync(srcUri);
  const total = info.exists && !info.isDirectory ? info.size : 0;

  const def = new Deflate({ gzip: true, level: 6 });
  const parts: Uint8Array[] = [];
  def.onData = (c) => parts.push(c as Uint8Array);

  if (total === 0) {
    def.push(new Uint8Array(0), true);
  } else {
    for (let pos = 0; pos < total; pos += CHUNK) {
      const length = Math.min(CHUNK, total - pos);
      const b64 = await FileSystem.readAsStringAsync(srcUri, { encoding: 'base64', position: pos, length });
      def.push(new Uint8Array(Buffer.from(b64, 'base64')), pos + length >= total);
      onStage?.(`압축 중… ${Math.round(((pos + length) / total) * 100)}%`);
    }
  }
  if (def.err) throw new Error(`압축 실패: ${def.msg}`);

  const size = parts.reduce((n, c) => n + c.length, 0);
  const merged = new Uint8Array(size);
  let o = 0;
  for (const c of parts) { merged.set(c, o); o += c.length; }
  await guarded(() => FileSystem.writeAsStringAsync(dstUri, Buffer.from(merged).toString('base64'), { encoding: 'base64' }));
  return size;
}

async function put(fileUri: string, path: string, contentType: string, onStage?: UploadStage) {
  onStage?.('업로드 중…');
  const res = await FileSystem.uploadAsync(uploadUrl(path), fileUri, {
    httpMethod: 'POST',
    uploadType: FileSystem.FileSystemUploadType.BINARY_CONTENT,
    headers: uploadHeaders(contentType),
  });
  if (res.status >= 300) throw new Error(`업로드 실패 (HTTP ${res.status}) ${(res.body || '').slice(0, 200)}`);
}

const rm = (uri: string) => FileSystem.deleteAsync(uri, { idempotent: true }).catch(() => {});

async function guarded<T>(write: () => Promise<T>): Promise<T> {
  try {
    return await write();
  } catch (e) {
    if (isNoSpaceError(e)) throw new StorageFullError(await freeBytes());
    throw e;
  }
}


const PENDING_DIR = `${FileSystem.documentDirectory || ''}pending-logs/`;

const PENDING_MAX_ITEMS = 20;
const PENDING_MAX_BYTES = 500 * 1024 * 1024;

async function readQueue(): Promise<Pending[]> {
  try { return JSON.parse((await AsyncStorage.getItem(PENDING_KEY)) || '[]'); } catch { return []; }
}
const writeQueue = (q: Pending[]) => AsyncStorage.setItem(PENDING_KEY, JSON.stringify(q));

async function enqueue(tmpUri: string, path: string, contentType: string, bytes: number, label: string) {
  if (bytes > PENDING_MAX_BYTES) throw new Error(`보류함에 담기엔 너무 큽니다 (${Math.round(bytes / 1048576)}MB)`);

  await FileSystem.makeDirectoryAsync(PENDING_DIR, { intermediates: true }).catch(() => {});
  const uri = `${PENDING_DIR}${path.replace(/\//g, '_')}`;
  await FileSystem.moveAsync({ from: tmpUri, to: uri });
  const item = { uri, path, contentType, bytes, label, at: Date.now() };

  const q = [...(await readQueue()), item];
  while (q.length > PENDING_MAX_ITEMS || q.reduce((n, p) => n + p.bytes, 0) > PENDING_MAX_BYTES) {
    const old = q.shift();
    if (!old || old === item) break;
    await rm(old.uri);
  }
  await writeQueue(q);
}

export const pendingCount = async () => (await readQueue()).length;

export async function flushPending(onStage?: UploadStage) {
  const q = await readQueue();
  const left: Pending[] = [];
  let sent = 0, dropped = 0;
  for (const [i, p] of q.entries()) {
    onStage?.(`보류함 전송 ${i + 1}/${q.length}…`);
    try {
      await put(p.uri, p.path, p.contentType);
      await rm(p.uri);
      sent++;
    } catch (e) {
      if (shouldQueue(e)) { left.push(p); continue; }
      await rm(p.uri);
      dropped++;
    }
  }
  await writeQueue(left);
  return { sent, failed: left.length, dropped };
}

export async function clearPending() {
  for (const p of await readQueue()) await rm(p.uri);
  await writeQueue([]);
}

export type UploadResult = { path: string; bytes: number; queued?: boolean };

async function putOrQueue(
  uri: string, path: string, contentType: string, bytes: number, label: string, onStage?: UploadStage,
): Promise<UploadResult> {
  try {
    await put(uri, path, contentType, onStage);
    return { path, bytes };
  } catch (e) {
    if (!shouldQueue(e)) throw e;
    await enqueue(uri, path, contentType, bytes, label);
    return { path, bytes, queued: true };
  }
}

export async function uploadSystemlog(ip: string, serial: string, date: string, onStage?: UploadStage) {
  const raw = `${FileSystem.cacheDirectory || ''}up-systemlog-${date}.jsonl`;
  const gz = `${raw}.gz`;
  try {
    onStage?.('로봇에서 받는 중…');
    const dl = await guarded(() => FileSystem.downloadAsync(
      `${restBase(ip)}/api/systemlog/date?date=${encodeURIComponent(date)}`,
      raw,
      { headers: robotAuth() },
    ));
    if (dl.status >= 300) throw new Error(`로봇 응답 HTTP ${dl.status}`);

    const bytes = await gzipFile(raw, gz, onStage);
    const path = objectPath(serial, 'systemlog', `${date}.jsonl.gz`);
    return await putOrQueue(gz, path, 'application/gzip', bytes, `시스템 로그 ${date}`, onStage);
  } finally {
    await rm(raw); await rm(gz);
  }
}

export async function uploadBlackbox(ip: string, serial: string, dir: string, onStage?: UploadStage) {
  const safe = dir.replace(/[/\\]/g, '_');
  const zip = `${FileSystem.cacheDirectory || ''}up-blackbox-${safe}.zip`;
  try {
    onStage?.('로봇에서 받는 중…');
    const dl = await guarded(() => FileSystem.downloadAsync(
      `${restBase(ip)}/api/blackbox/zip?path=${encodeURIComponent(dir)}`,
      zip,
      { headers: robotAuth() },
    ));
    if (dl.status >= 300) throw new Error(`로봇 응답 HTTP ${dl.status}`);

    const info = await FileSystem.getInfoAsync(zip);
    const bytes = info.exists && !info.isDirectory ? info.size : 0;
    const path = objectPath(serial, 'blackbox', `${safe}.zip`);
    return await putOrQueue(zip, path, 'application/zip', bytes, `블랙박스 ${dir}`, onStage);
  } finally {
    await rm(zip);
  }
}

export { fmtBytes };
