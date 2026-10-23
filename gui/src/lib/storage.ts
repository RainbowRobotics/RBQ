import * as FileSystem from 'expo-file-system/legacy';
import { checkFree } from './storageCommon';

export async function freeBytes(): Promise<number | null> {
  try { return await FileSystem.getFreeDiskStorageAsync(); } catch { return null; }
}

export async function ensureFreeSpace(need = 0) {
  checkFree(await freeBytes(), need);
}
