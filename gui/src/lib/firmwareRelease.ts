import { Platform } from 'react-native';
import Constants from 'expo-constants';
import { cacheDirectory, createDownloadResumable, deleteAsync, getInfoAsync, readDirectoryAsync,
  readAsStringAsync, writeAsStringAsync } from 'expo-file-system/legacy';
import { isDesktop } from '@/lib/desktopBridge';

import { DOWNLOAD_BASE as S3_BASE, downloadHeaders } from '@/lib/downloadBase';

const metaFetch = (path: string): Promise<Response> =>
  (Platform.OS === 'web' && isDesktop())
    ? fetch(`/s3?path=${encodeURIComponent(path)}`, { cache: 'no-store', headers: downloadHeaders })
    : fetch(`${S3_BASE}/${path}`, { cache: 'no-store' });

export type FwChannel = 'release' | 'nightly';

export const CHANNEL: FwChannel | null = ((): FwChannel | null => {
  const c = Constants.expoConfig?.extra?.channel;
  return c === 'release' || c === 'nightly' ? c : null;
})();

export interface FwRelease {
  tag: string;
  version: string;
  publishedAt: string;
  latest: boolean;
  channel: FwChannel;
}

const missing = (status: number) => status === 403 || status === 404;


export async function fetchFirmwareVersions(): Promise<FwRelease[]> {
  if (!CHANNEL) return [];
  if (CHANNEL === 'nightly') return fetchNightly();
  const res = await metaFetch('index.json');
  if (missing(res.status)) return [];
  if (!res.ok) throw new Error(`목록 응답 ${res.status}`);
  const body = await res.json();
  const list: string[] = Array.isArray(body?.versions)
    ? body.versions.filter((x: unknown) => typeof x === 'string')
    : [];
  return list.map((tag, i) => ({
    tag,
    version: tag.replace(/^v/, ''),
    publishedAt: '',
    latest: i === 0,
    channel: CHANNEL,
  }));
}

async function fetchNightly(): Promise<FwRelease[]> {
  const res = await metaFetch('nightly/nightly.txt');
  if (missing(res.status)) return [];
  if (!res.ok) throw new Error(`나이틀리 응답 ${res.status}`);
  const text = await res.text();
  const version = /^version:\s*(\S+)/m.exec(text)?.[1];
  if (!version) throw new Error('나이틀리 메타데이터를 해석할 수 없습니다');
  const builtAt = /^built_at:\s*(\S+)/m.exec(text)?.[1];
  return [{
    tag: version,
    version: version.replace(/^v/, ''),
    publishedAt: (builtAt ?? '').slice(0, 10),
    latest: true,
    channel: 'nightly',
  }];
}

export async function fetchReleaseNotes(tag: string): Promise<{ ko: string; en: string } | null> {
  if (!tag || tag.includes('-nightly.')) return null;
  try {
    const res = await metaFetch(`${tag}/release-notes.json`);
    if (!res.ok) return null;
    const body = await res.json();
    const ko = typeof body?.ko === 'string' ? body.ko : '';
    const en = typeof body?.en === 'string' ? body.en : '';
    return ko || en ? { ko, en } : null;
  } catch {
    return null;
  }
}

export interface CachedTar {
  name: string;
  size: number;
  id?: string;
}

export const cacheId = (rel: FwRelease): string => (rel.channel === 'nightly' ? rel.publishedAt : rel.tag);

export async function listCachedTars(): Promise<CachedTar[]> {
  if (Platform.OS === 'web' || !cacheDirectory) return [];
  try {
    const names = (await readDirectoryAsync(cacheDirectory))
      .filter((f) => /^RBQ-.+(\.tar\.gz|\.zip)$/.test(f));
    const out: CachedTar[] = [];
    for (const name of names) {
      const info = await getInfoAsync(`${cacheDirectory}${name}`);
      if (!info.exists || !info.size) continue;
      const id = await readAsStringAsync(`${cacheDirectory}${name}.id`).catch(() => undefined);
      out.push({ name, size: info.size, id: id?.trim() || undefined });
    }
    return out;
  } catch {
    return [];
  }
}

export async function removeCachedTar(name: string): Promise<void> {
  if (Platform.OS === 'web' || !cacheDirectory) return;
  await deleteAsync(`${cacheDirectory}${name}`, { idempotent: true });
  await deleteAsync(`${cacheDirectory}${name}.id`, { idempotent: true });
}

export interface FwFile { uri: string; name: string; size: number }

export interface FwDownload {
  cancel: () => void;
  done: Promise<FwFile & { legacy?: FwFile }>;
}

const tarName = (rel: FwRelease) =>
  rel.channel === 'nightly' ? 'RBQ-nightly.tar.gz' : `RBQ-${rel.tag}.tar.gz`;

const tarUrl = (rel: FwRelease) =>
  rel.channel === 'nightly'
    ? `${S3_BASE}/nightly/RBQ-nightly.tar.gz`
    : `${S3_BASE}/${rel.tag}/RBQ-${rel.tag}.tar.gz`;

export const pkgName = (rel: FwRelease) =>
  `RBQ-${rel.channel === 'nightly' ? 'nightly' : rel.tag}.zip`;
const pkgUrl = (rel: FwRelease) =>
  `${S3_BASE}/${rel.channel === 'nightly' ? 'nightly' : rel.tag}/${pkgName(rel)}`;


function downloadViaProxy(
  rel: FwRelease,
  onProgress: (written: number, total: number) => void,
): FwDownload {
  const q = `channel=${encodeURIComponent(rel.channel)}&tag=${encodeURIComponent(rel.tag)}`;
  const ctrl = new AbortController();
  const done = (async () => {
    const res = await fetch(`/fw-download?${q}`, { signal: ctrl.signal, headers: downloadHeaders });
    if (!res.ok || !res.body) throw new Error(`다운로드 실패 (HTTP ${res.status})`);
    const reader = res.body.getReader();
    const dec = new TextDecoder();
    let buf = '';
    let last = '';
    for (;;) {
      const { done: eof, value } = await reader.read();
      if (eof) break;
      buf += dec.decode(value, { stream: true });
      const lines = buf.split('\n');
      buf = lines.pop() ?? '';
      for (const ln of lines) {
        if (ln.startsWith('P ')) {
          const [, w, t] = ln.split(' ');
          onProgress(Number(w), Number(t));
        } else if (ln.trim()) last = ln.trim();
      }
    }
    if (buf.trim()) last = buf.trim();
    if (last.startsWith('ERR:')) throw new Error(last.slice(4));
    const m = /^OK (\S+) (\d+)$/.exec(last);
    if (!m) throw new Error('다운로드 실패 — 프록시 응답을 해석할 수 없습니다');
    return { uri: `/fw-file?${q}`, name: m[1], size: Number(m[2]) };
  })();
  return { cancel: () => ctrl.abort(), done };
}

export function downloadFirmware(
  rel: FwRelease,
  onProgress: (written: number, total: number) => void,
): FwDownload {
  if (Platform.OS === 'web') return downloadViaProxy(rel, onProgress);
  let cancelled = false;
  let current: ReturnType<typeof createDownloadResumable> | null = null;
  const staged: string[] = [];

  const attempt = async (name: string, url: string): Promise<FwFile | null> => {
    const uri = `${cacheDirectory}${name}`;
    current = createDownloadResumable(url, uri, {}, (p) =>
      onProgress(p.totalBytesWritten, p.totalBytesExpectedToWrite),
    );
    staged.push(uri);
    const r = await current.downloadAsync();
    if (cancelled) throw new Error('cancelled');
    if (r && missing(r.status)) {
      await deleteAsync(uri, { idempotent: true }).catch(() => {});
      return null;
    }
    if (!r || (r.status !== 200 && r.status !== 0)) throw new Error(`다운로드 실패 (HTTP ${r?.status ?? '?'})`);
    const info = await getInfoAsync(uri);
    if (!info.exists || !info.size) throw new Error('다운로드 파일이 비어 있습니다');
    await writeAsStringAsync(`${uri}.id`, cacheId(rel)).catch(() => {});
    staged.push(`${uri}.id`);
    return { uri, name, size: info.size };
  };

  const done = (async () => {
    const locked = await attempt(pkgName(rel), pkgUrl(rel));
    const legacy = await attempt(tarName(rel), tarUrl(rel));
    if (locked) return legacy ? { ...locked, legacy } : locked;
    if (legacy) return legacy;
    throw new Error('이 버전에는 로봇 패키지가 없습니다');
  })();

  return {
    cancel: () => {
      cancelled = true;
      current?.cancelAsync().catch(() => {});
      for (const uri of staged) deleteAsync(uri, { idempotent: true }).catch(() => {});
    },
    done,
  };
}
