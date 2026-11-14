import { t } from '@/lib/i18n';
import type { FileRange } from './fileStream';

const picked = new Map<string, Blob>();
export function rememberPicked(uri: string, file: Blob): void {
  picked.set(uri, file);
}

let blobCache: { uri: string; blob: Promise<Blob> } | null = null;
function blobOf(uri: string): Promise<Blob> {
  const file = picked.get(uri);
  if (file) return Promise.resolve(file);
  if (blobCache?.uri !== uri) blobCache = { uri, blob: fetch(uri).then((r) => r.blob()) };
  return blobCache.blob;
}

function rangeFetch(uri: string, { offset, length }: FileRange): Promise<Response> {
  return fetch(uri, { headers: { Range: `bytes=${offset}-${offset + length - 1}` } }).then((r) => {
    if (r.status !== 206 || !r.body) throw new Error(`${t('파일 일부를 읽을 수 없습니다')} (HTTP ${r.status})`);
    return r;
  });
}

export async function readFileAt(uri: string, offset: number, length: number): Promise<Uint8Array> {
  const buf = uri.startsWith('blob:')
    ? await (await blobOf(uri)).slice(offset, offset + length).arrayBuffer()
    : await (await rangeFetch(uri, { offset, length })).arrayBuffer();
  if (buf.byteLength < length) throw new Error(t('파일이 예상보다 짧습니다 — USB 에 다시 복사하세요'));
  return new Uint8Array(buf);
}

export function openFileStream(uri: string, range?: FileRange): ReadableStreamDefaultReader<Uint8Array> {
  let inner: Promise<ReadableStreamDefaultReader<Uint8Array>> | null = null;
  const open = (): Promise<ReadableStreamDefaultReader<Uint8Array>> => {
    if (range && uri.startsWith('blob:')) return blobOf(uri).then((b) => b.slice(range.offset, range.offset + range.length).stream().getReader());
    if (range) return rangeFetch(uri, range).then((r) => r.body!.getReader());
    return fetch(uri).then((r) => {
      if (!r.ok || !r.body) throw new Error(`파일을 읽을 수 없습니다 (HTTP ${r.status})`);
      return r.body.getReader();
    });
  };
  const get = () => (inner ??= open());
  return {
    read: () => get().then((r) => r.read()),
    cancel: (reason?: unknown) => get().then((r) => r.cancel(reason)).catch(() => {}),
    releaseLock: () => { },
    get closed() { return get().then((r) => r.closed); },
  } as unknown as ReadableStreamDefaultReader<Uint8Array>;
}
