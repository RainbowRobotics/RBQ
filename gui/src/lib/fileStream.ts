import { File } from 'expo-file-system';
import { t } from '@/lib/i18n';

export type FileRange = { offset: number; length: number };

export function rememberPicked(_uri: string, _file: Blob): void {}
const CHUNK = 64 * 1024;
const SHORT = () => new Error(t('파일이 예상보다 짧습니다 — USB 에 다시 복사하세요'));

export async function readFileAt(uri: string, offset: number, length: number): Promise<Uint8Array> {
  const h = new File(uri).open();
  try {
    h.offset = offset;
    const b = h.readBytes(length);
    if (b.length < length) throw SHORT();
    return b;
  } finally {
    h.close();
  }
}

export function openFileStream(uri: string, range?: FileRange): ReadableStreamDefaultReader<Uint8Array> {
  if (!range) return new File(uri).readableStream().getReader();
  const h = new File(uri).open();
  h.offset = range.offset;
  let left = range.length;
  let open = true;
  const close = () => { if (open) { open = false; h.close(); } };
  return {
    read: async () => {
      if (left <= 0) { close(); return { done: true, value: undefined }; }
      const value = h.readBytes(Math.min(CHUNK, left));
      if (!value.length) { close(); throw SHORT(); }
      left -= value.length;
      return { done: false, value };
    },
    cancel: async () => close(),
    releaseLock: () => {},
  } as unknown as ReadableStreamDefaultReader<Uint8Array>;
}
