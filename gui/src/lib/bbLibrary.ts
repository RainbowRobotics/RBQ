import * as FileSystem from 'expo-file-system/legacy';
import { isNoSpaceError, StorageFullError } from './storageCommon';
import { freeBytes } from './storage';
import { serialKey, toEntries, type LibEntry } from './bbLibraryCommon';

const ROOT = (FileSystem.documentDirectory || '') + 'blackbox/';

export const libSupported = !!FileSystem.documentDirectory;

export async function libList(): Promise<LibEntry[]> {
  const files: { serial: string; name: string; size: number }[] = [];
  try {
    for (const serial of await FileSystem.readDirectoryAsync(ROOT)) {
      const dir = `${ROOT}${serial}/`;
      let names: string[] = [];
      try { names = await FileSystem.readDirectoryAsync(dir); } catch { continue; }
      for (const name of names) {
        const info = await FileSystem.getInfoAsync(dir + name);
        if (info.exists && !info.isDirectory) files.push({ serial, name, size: info.size ?? 0 });
      }
    }
  } catch { }
  return toEntries(files);
}

export async function libPut(serial: string, name: string, src: { uri: string } | { blob: Blob }): Promise<void> {
  if (!('uri' in src)) throw new Error('bbLibrary(native): uri 만 받는다');
  const dir = `${ROOT}${serialKey(serial)}/`;
  const to = dir + name;
  try {
    await FileSystem.makeDirectoryAsync(dir, { intermediates: true });
    await FileSystem.deleteAsync(to, { idempotent: true });
    await FileSystem.copyAsync({ from: src.uri, to });
  } catch (e) {
    await FileSystem.deleteAsync(to, { idempotent: true }).catch(() => {});
    if (isNoSpaceError(e)) throw new StorageFullError(await freeBytes());
    throw e;
  }
}

export async function libRead(e: LibEntry): Promise<Uint8Array> {
  const res = await fetch(`${ROOT}${e.serial}/${e.name}`);
  return new Uint8Array(await res.arrayBuffer());
}

export async function libRemove(e: LibEntry): Promise<void> {
  const dir = `${ROOT}${e.serial}/`;
  await FileSystem.deleteAsync(dir + e.name, { idempotent: true });
  try {
    if ((await FileSystem.readDirectoryAsync(dir)).length === 0) await FileSystem.deleteAsync(dir, { idempotent: true });
  } catch { }
}
