import { File } from 'expo-file-system';

export function openFileStream(uri: string): ReadableStreamDefaultReader<Uint8Array> {
  return new File(uri).readableStream().getReader();
}
