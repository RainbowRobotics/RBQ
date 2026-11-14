import { Platform } from 'react-native';
import { cacheDirectory, copyAsync, deleteAsync, getInfoAsync, readDirectoryAsync } from 'expo-file-system/legacy';
import { t } from '@/lib/i18n';
import type { CachedTar } from './firmwareRelease';

export const OPTION_FILE_RE = /^option-([a-z]{2,16})-(v[0-9A-Za-z.+]{1,40}(-[0-9]{4})?)-(u[0-9]{2}-(x86_64|aarch64))\.rbo$/;

export interface OptionFile { name: string; label: string; system: string }

export function parseOptionFile(fileName: string): OptionFile | null {
  if (fileName.includes('..')) return null;
  const m = OPTION_FILE_RE.exec(fileName);
  return m ? { name: m[1], label: m[2], system: m[4] } : null;
}

export type OptionRow = 'installed' | 'update' | 'installable' | 'otherInstalled' | 'otherSystem';

export interface RobotOption { option: string; optionVersion: string; system: string }

export function optionRowState(file: OptionFile, robot: RobotOption): OptionRow {
  if (robot.system && file.system !== robot.system) return 'otherSystem';
  if (!robot.option) return 'installable';
  if (robot.option !== file.name) return 'otherInstalled';
  return robot.optionVersion === file.label ? 'installed' : 'update';
}

const CORE_LABEL_RE = /^core-(v[0-9A-Za-z.+]{1,40}(?:-[0-9]{4})?)-(u[0-9]{2}-(?:x86_64|aarch64))\.rbq$/;

export function optionFileFor(coreFile: string, option: string): string | null {
  const m = CORE_LABEL_RE.exec(coreFile);
  if (!m) return null;
  const name = `option-${option}-${m[1]}-${m[2]}.rbo`;
  return parseOptionFile(name) ? name : null;
}

export function optionFollow(coreFile: string, robot: Pick<RobotOption, 'option' | 'optionVersion'>): { name: string; label: string; same: boolean } | null {
  if (!robot.option) return null;
  const name = optionFileFor(coreFile, robot.option);
  const f = name ? parseOptionFile(name) : null;
  return name && f ? { name, label: f.label, same: robot.optionVersion === f.label } : null;
}

export async function optionBeside(core: { uri: string }, name: string): Promise<{ uri: string; name: string; size: number } | null> {
  if (!parseOptionFile(name)) return null;
  if (core.uri.startsWith('/fw-file?')) {
    const q = new URLSearchParams(core.uri.slice('/fw-file?'.length));
    q.set('name', name);
    const uri = `/fw-file?${q.toString()}`;
    const r = await fetch(uri, { method: 'HEAD' }).catch(() => null);
    const size = Number(r?.ok ? r.headers.get('content-length') : 0);
    return size > 0 ? { uri, name, size } : null;
  }
  if (Platform.OS === 'web' || !cacheDirectory) return null;
  const uri = `${cacheDirectory}${name}`;
  const info = await getInfoAsync(uri).catch(() => null);
  return info?.exists && info.size ? { uri, name, size: info.size } : null;
}

export const optionTitle = (name: string) => `${name.toUpperCase()} ${t('옵션')}`;

export function optionErrorText(message: string): string {
  if (/^Option package signature (missing|invalid)/.test(message)) return t('서명이 없거나 맞지 않는 옵션 파일입니다 — 레인보우로보틱스가 배포한 파일을 쓰세요');
  return message;
}

export async function listCachedOptions(): Promise<CachedTar[]> {
  if (Platform.OS === 'web' || !cacheDirectory) return [];
  try {
    const out: CachedTar[] = [];
    for (const name of (await readDirectoryAsync(cacheDirectory)).filter((n) => parseOptionFile(n))) {
      const info = await getInfoAsync(`${cacheDirectory}${name}`);
      if (info.exists && info.size) out.push({ name, size: info.size });
    }
    return out;
  } catch {
    return [];
  }
}

export async function importOptionFile(picked: { uri: string; name: string; size: number }): Promise<CachedTar | null> {
  if (Platform.OS === 'web' || !cacheDirectory) return null;
  const to = `${cacheDirectory}${picked.name}`;
  if (picked.uri !== to) {
    await deleteAsync(to, { idempotent: true });
    await copyAsync({ from: picked.uri, to });
  }
  return { name: picked.name, size: picked.size };
}

export async function removeCachedOption(name: string): Promise<void> {
  if (Platform.OS === 'web' || !cacheDirectory || !parseOptionFile(name)) return;
  await deleteAsync(`${cacheDirectory}${name}`, { idempotent: true });
  await deleteAsync(`${cacheDirectory}${name}.id`, { idempotent: true });
}
