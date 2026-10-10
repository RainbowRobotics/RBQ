import { Linking, Platform } from 'react-native';
import Constants from 'expo-constants';
import { cacheDirectory, createDownloadResumable, getContentUriAsync } from 'expo-file-system/legacy';
import * as IntentLauncher from 'expo-intent-launcher';
import { isDesktop } from '@/lib/desktopBridge';
import { t } from '@/lib/i18n';
import { CHANNEL, type FwChannel, type FwRelease } from '@/lib/firmwareRelease';

import { DOWNLOAD_BASE as S3_BASE, downloadHeaders } from '@/lib/downloadBase';
const APK_NAME = 'rbq-controller.apk';
const APPIMAGE_NAME = 'rbq-controller-x86_64.AppImage';

const PLAY_URL = 'market://details?id=';
const ASC_APP_ID = '6792970388';

export interface AppRelease {
  tag: string;
  version: string;
  channel: FwChannel;
}

const assetBase = (rel: AppRelease): string =>
  rel.channel === 'nightly' ? `${S3_BASE}/nightly` : `${S3_BASE}/${rel.tag}`;

export const currentAppVersion = (): string => Constants.expoConfig?.version ?? '0.0.0';

function cmpBase(a: string, b: string): number {
  const pa = a.split('-')[0].split('.').map(Number);
  const pb = b.split('-')[0].split('.').map(Number);
  for (let i = 0; i < 3; i++) {
    const d = (pa[i] || 0) - (pb[i] || 0);
    if (d) return d;
  }
  return 0;
}

export function newerAppRelease(list: FwRelease[]): AppRelease | null {
  const top = list.find((r) => r.latest) ?? list[0];
  if (!top) return null;
  const cur = currentAppVersion();
  const isNew = top.channel === 'release'
    ? cmpBase(top.version, cur) > 0
    : top.version !== cur;
  return isNew ? { tag: top.tag, version: top.version, channel: top.channel } : null;
}

export type AppUpdateMode = 'apk' | 'appimage' | 'store' | 'none';

const isPlayBuild = (): boolean => Constants.expoConfig?.extra?.playBuild === true;

export async function appUpdateMode(): Promise<AppUpdateMode> {
  if (Platform.OS === 'ios') return 'store';
  if (Platform.OS === 'android') return isPlayBuild() ? 'store' : 'apk';
  if (Platform.OS === 'web' && isDesktop()) {
    try {
      const r = await fetch('/app-update?check=1', { headers: downloadHeaders });
      return r.status === 204 ? 'appimage' : 'none';
    } catch {
      return 'none';
    }
  }
  return 'none';
}

async function openStore(): Promise<string> {
  const pkg = Constants.expoConfig?.android?.package ?? 'com.rainbowrobotics.rbq';
  const url = Platform.OS === 'ios'
    ? `itms-apps://apps.apple.com/app/id${ASC_APP_ID}`
    : `${PLAY_URL}${pkg}`;
  await Linking.openURL(url).catch(() =>
    Linking.openURL(
      Platform.OS === 'ios'
        ? `https://apps.apple.com/app/id${ASC_APP_ID}`
        : `https://play.google.com/store/apps/details?id=${pkg}`,
    ),
  );
  return t('스토어에서 업데이트하세요');
}

export async function runAppUpdate(
  rel: AppRelease,
  mode: AppUpdateMode,
  onProgress?: (written: number, total: number) => void,
): Promise<string> {
  if (mode === 'store') return openStore();

  if (mode === 'apk') {
    const uri = `${cacheDirectory}${APK_NAME}`;
    const dr = createDownloadResumable(`${assetBase(rel)}/${APK_NAME}`, uri, {}, (p) =>
      onProgress?.(p.totalBytesWritten, p.totalBytesExpectedToWrite),
    );
    const r = await dr.downloadAsync();
    if (r?.status === 404 || r?.status === 403) throw new Error(t('이 릴리즈에는 앱 산출물이 없습니다 (구 릴리즈)'));
    if (!r || r.status !== 200) throw new Error(`${t('다운로드 실패')} (HTTP ${r?.status ?? '?'})`);
    const content = await getContentUriAsync(uri);
    await IntentLauncher.startActivityAsync('android.intent.action.INSTALL_PACKAGE', {
      data: content,
      flags: 1,
    });
    return t('설치 화면에서 진행하세요');
  }

  const q = `channel=${encodeURIComponent(rel.channel)}&tag=${encodeURIComponent(rel.tag)}`
    + `&asset=${encodeURIComponent(APPIMAGE_NAME)}`;
  const res = await fetch(`/app-update?${q}`, { headers: downloadHeaders });
  const body = await res.text().catch(() => '');
  if (!res.ok || !/(^|\n)OK\s*$/.test(body)) {
    const err = /ERR:(.*?)\s*$/.exec(body)?.[1] ?? (body || `HTTP ${res.status}`);
    if (/40[34]/.test(err)) throw new Error(t('이 릴리즈에는 앱 산출물이 없습니다 (구 릴리즈)'));
    throw new Error(err);
  }
  return t('교체 완료 — 앱을 재시작하면 새 버전으로 실행됩니다');
}

export async function appFetchOnly(rel: AppRelease, signal?: AbortSignal): Promise<void> {
  const q = `channel=${encodeURIComponent(rel.channel)}&tag=${encodeURIComponent(rel.tag)}`
    + `&asset=${encodeURIComponent(APPIMAGE_NAME)}&install=0`;
  const res = await fetch(`/app-update?${q}`, { signal, headers: downloadHeaders });
  const body = await res.text().catch(() => '');
  if (!res.ok || !/(^|\n)OK\s*$/.test(body)) {
    const err = /ERR:(.*?)\s*$/.exec(body)?.[1] ?? (body || `HTTP ${res.status}`);
    if (/40[34]/.test(err)) throw new Error(t('이 릴리즈에는 앱 산출물이 없습니다 (구 릴리즈)'));
    throw new Error(err);
  }
}

export interface AppBuild {
  name: string;
  current: boolean;
  app: boolean;
  version: string;
  tar: { name: string; size: number } | null;
}

export interface AppInstallStatus {
  appimage: boolean;
  installed: boolean;
  managed: boolean;
  current: string | null;
  builds: AppBuild[];
}

export async function appInstallStatus(): Promise<AppInstallStatus | null> {
  if (Platform.OS !== 'web' || !isDesktop()) return null;
  try {
    const r = await fetch(`/app-status?channel=${encodeURIComponent(CHANNEL ?? 'release')}`, { headers: downloadHeaders });
    if (!r.ok) return null;
    return (await r.json()) as AppInstallStatus;
  } catch {
    return null;
  }
}

export async function appSelectBuild(name: string): Promise<void> {
  const r = await fetch(`/app-select?channel=${encodeURIComponent(CHANNEL ?? 'release')}&build=${encodeURIComponent(name)}`);
  const body = await r.json().catch(() => ({}));
  if (!r.ok || !body.ok) throw new Error(body.error || `HTTP ${r.status}`);
}

export async function appRemoveBuild(name: string): Promise<void> {
  const r = await fetch(`/app-remove?channel=${encodeURIComponent(CHANNEL ?? 'release')}&build=${encodeURIComponent(name)}`);
  const body = await r.json().catch(() => ({}));
  if (!r.ok || !body.ok) throw new Error(body.error || `HTTP ${r.status}`);
}

export { CHANNEL };
