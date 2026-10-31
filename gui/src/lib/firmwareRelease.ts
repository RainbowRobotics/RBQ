import { Platform } from 'react-native';
import Constants from 'expo-constants';
import { cacheDirectory, deleteAsync, getInfoAsync, readDirectoryAsync,
  readAsStringAsync, writeAsStringAsync } from 'expo-file-system/legacy';
import { downloadToFile, type FileDl } from '@/lib/fileDownload';

import { DOWNLOAD_BASE as S3_BASE, downloadHeaders } from '@/lib/downloadBase';

const metaFetch = (path: string): Promise<Response> => fetch(`${S3_BASE}/${path}`, { cache: 'no-store' });

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
  core?: string[];
}

const CORE_RE = /^core-v[0-9A-Za-z.+]{1,40}(?:-[0-9]{4})?-(u[0-9]{2}-(?:x86_64|aarch64))\.rbq$/;
export const coreSystemOf = (name: string): string | null => CORE_RE.exec(name)?.[1] ?? null;

export const CORE_SYSTEMS = ['u22-x86_64', 'u22-aarch64', 'u24-x86_64', 'u24-aarch64'] as const;
export const LEGACY_SYSTEM = 'legacy';

export function coreNames(rel: FwRelease): string[] {
  return rel.channel === 'nightly' ? (rel.core ?? []) : CORE_SYSTEMS.map((sys) => `core-${rel.tag}-${sys}.rbq`);
}

export function coreName(rel: FwRelease, system: string): string | null {
  if (rel.channel === 'nightly') return rel.core?.find((n) => coreSystemOf(n) === system) ?? null;
  return `core-${rel.tag}-${system}.rbq`;
}

export function pkgCandidates(rel: FwRelease, system: string | null | undefined): string[] {
  const base = `RBQ-${rel.channel === 'nightly' ? 'nightly' : rel.tag}`;
  const legacy = [`${base}.zip`, `${base}.tar.gz`];
  if (system === LEGACY_SYSTEM) return legacy;
  if (!system) return [...coreNames(rel), ...legacy];
  const core = coreName(rel, system);
  return core ? [core] : [];
}

export function pickPackage<T extends { name: string }>(
  files: T[], rel: FwRelease | undefined, system: string | null | undefined,
): T | undefined {
  if (rel) {
    for (const n of pkgCandidates(rel, system)) {
      const f = files.find((x) => x.name === n);
      if (f) return f;
    }
    return undefined;
  }
  if (system === LEGACY_SYSTEM) return files.find((f) => /^RBQ-/.test(f.name));
  return files.find((f) => (system ? coreSystemOf(f.name) === system : !!coreSystemOf(f.name)));
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
  const core = (/^core:\s*(.*)$/m.exec(text)?.[1] ?? '').split(/\s+/).filter((n) => coreSystemOf(n));
  return [{
    tag: version,
    version: version.replace(/^v/, ''),
    publishedAt: (builtAt ?? '').slice(0, 10),
    latest: true,
    channel: 'nightly',
    core,
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

export const cacheId = (rel: FwRelease): string => rel.tag;

export async function listCachedTars(): Promise<CachedTar[]> {
  if (Platform.OS === 'web' || !cacheDirectory) return [];
  try {
    const all = await readDirectoryAsync(cacheDirectory);
    const stale = Date.now() / 1000 - STALE_PART_DAYS * 86400;
    for (const f of all.filter((n) => /\.part(\.src)?$/.test(n))) {
      const info = await getInfoAsync(`${cacheDirectory}${f}`).catch(() => null);
      if (info?.exists && (info.modificationTime ?? 0) < stale) await deleteAsync(`${cacheDirectory}${f}`, { idempotent: true }).catch(() => {});
    }
    const names = all
      .filter((f) => /^RBQ-.+(\.tar\.gz|\.zip)$/.test(f) || coreSystemOf(f));
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

const STALE_PART_DAYS = 7;

export async function removeCachedTar(name: string): Promise<void> {
  if (Platform.OS === 'web' || !cacheDirectory) return;
  await deleteAsync(`${cacheDirectory}${name}`, { idempotent: true });
  await deleteAsync(`${cacheDirectory}${name}.id`, { idempotent: true });
  for (const ext of SIDECARS) await deleteAsync(`${cacheDirectory}${name}.${ext}`, { idempotent: true });
}

export interface FwSidecars { sha256?: string; signature?: string }
const SIDECARS = ['sha256', 'sig'] as const;
const SHA_RE = /^[0-9a-f]{64}$/;
const SIG_RE = /^[A-Za-z0-9+/]{86}==$/;

export function parseSidecars(sha?: string | null, sig?: string | null): FwSidecars {
  const h = sha?.trim().split(/\s+/)[0]?.toLowerCase();
  const g = sig?.trim();
  return { ...(h && SHA_RE.test(h) ? { sha256: h } : {}), ...(g && SIG_RE.test(g) ? { signature: g } : {}) };
}

export async function fileSidecars(file: { uri: string; name: string }): Promise<FwSidecars> {
  if (!coreSystemOf(file.name)) return {};
  const read = (ext: string): Promise<string | null> => {
    if (Platform.OS !== 'web') return readAsStringAsync(`${file.uri}.${ext}`);
    if (!file.uri.startsWith('/fw-file?')) return Promise.resolve(null);
    return fetch(`${file.uri}&sidecar=${ext}`).then((r) => (r.ok ? r.text() : null));
  };
  const [sha, sig] = await Promise.all(SIDECARS.map((ext) => read(ext).catch(() => null)));
  return parseSidecars(sha, sig);
}

export interface FwFile { uri: string; name: string; size: number }

export type FwDownloaded = FwFile & { alts?: FwFile[] };

const bundle = (files: FwFile[]): FwDownloaded | null =>
  files.length ? (files.length > 1 ? { ...files[0], alts: files.slice(1) } : files[0]) : null;

export interface FwDownload {
  cancel: () => void;
  done: Promise<FwDownloaded>;
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
const coreUrl = (rel: FwRelease, name: string) =>
  `${S3_BASE}/${rel.channel === 'nightly' ? 'nightly' : rel.tag}/${name}`;


function downloadViaProxy(
  rel: FwRelease,
  onProgress: (written: number, total: number, part?: FwPart) => void,
  system?: string | null,
): FwDownload {
  const q = `channel=${encodeURIComponent(rel.channel)}&tag=${encodeURIComponent(rel.tag)}`;
  const ctrl = new AbortController();
  const one = async (name?: string, part?: FwPart): Promise<FwFile> => {
    const qn = name ? `${q}&name=${encodeURIComponent(name)}` : q;
    const res = await fetch(`/fw-download?${qn}`, { signal: ctrl.signal, headers: downloadHeaders });
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
          onProgress(Number(w), Number(t), part);
        } else if (ln.trim()) last = ln.trim();
      }
    }
    if (buf.trim()) last = buf.trim();
    if (last.startsWith('ERR:')) throw new Error(last.slice(4));
    const m = /^OK (\S+) (\d+)$/.exec(last);
    if (!m) throw new Error('다운로드 실패 — 프록시 응답을 해석할 수 없습니다');
    return { uri: `/fw-file?${q}&name=${encodeURIComponent(m[1])}`, name: m[1], size: Number(m[2]) };
  };
  const done = (async (): Promise<FwDownloaded> => {
    if (system === LEGACY_SYSTEM) return one();
    if (system) {
      const name = coreName(rel, system);
      if (!name) throw noPackageError(system);
      return one(name);
    }
    const names = coreNames(rel);
    const got: FwFile[] = [];
    for (let i = 0; i < names.length; i++) {
      try { got.push(await one(names[i], { i: i + 1, n: names.length })); } catch (e) {
        if (ctrl.signal.aborted || !/HTTP 40[34]/.test((e as Error).message)) throw e;
      }
    }
    return bundle(got) ?? one();
  })();
  return { cancel: () => ctrl.abort(), done };
}

const noPackageError = (system: string) => new Error(`이 버전에는 ${system} 로봇용 패키지가 없습니다`);
function noPackage(system: string): FwDownload {
  return { cancel: () => {}, done: Promise.reject(noPackageError(system)) };
}

export type FwPart = { i: number; n: number };

export function downloadFirmware(
  rel: FwRelease,
  onProgress: (written: number, total: number, part?: FwPart) => void,
  system?: string | null,
): FwDownload {
  if (Platform.OS === 'web') return downloadViaProxy(rel, onProgress, system);
  const known = !!system && system !== LEGACY_SYSTEM;
  const core = known ? coreName(rel, system!) : null;
  if (known && !core) return noPackage(system!);
  let cancelled = false;
  let current: FileDl | null = null;
  const staged: string[] = [];
  let nth: FwPart | undefined;

  const attempt = async (name: string, url: string): Promise<FwFile | null> => {
    const uri = `${cacheDirectory}${name}`;
    if (cancelled) throw new Error('cancelled');
    current = downloadToFile(url, uri, `${url}#${cacheId(rel)}`, (w, t) => onProgress(w, t, nth));
    const r = await current.done;
    if (cancelled) throw new Error('cancelled');
    if ('status' in r) {
      if (missing(r.status)) return null;
      throw new Error(`다운로드 실패 (HTTP ${r.status})`);
    }
    staged.push(uri);
    for (const ext of SIDECARS) await deleteAsync(`${uri}.${ext}`, { idempotent: true }).catch(() => {});
    await writeAsStringAsync(`${uri}.id`, cacheId(rel)).catch(() => {});
    staged.push(`${uri}.id`);
    return { uri, name, size: r.size };
  };

  const sidecars = async (url: string, uri: string) => {
    for (const ext of SIDECARS) {
      if (cancelled) return;
      let txt: string | null = null;
      for (let i = 0; ; i++) {
        const r = await fetch(`${url}.${ext}`, { cache: 'no-store' }).catch(() => null);
        if (r?.ok) { txt = await r.text().catch(() => null); if (txt != null) break; }
        else if (r && missing(r.status)) break;
        if (i >= 2 || cancelled) throw new Error(`체크섬 파일을 받지 못했습니다 — 연결을 확인하고 다시 받으세요 (${ext})`);
        await new Promise((res) => setTimeout(res, 1500));
      }
      if (!txt) continue;
      await writeAsStringAsync(`${uri}.${ext}`, txt.trim()).catch(() => {});
      staged.push(`${uri}.${ext}`);
    }
  };

  const done = (async (): Promise<FwDownloaded> => {
    if (core) {
      const url = coreUrl(rel, core);
      const f = await attempt(core, url);
      if (f) { await sidecars(url, f.uri); return f; }
      throw noPackageError(system!);
    }
    if (!system) {
      const names = coreNames(rel);
      const got: FwFile[] = [];
      for (let i = 0; i < names.length; i++) {
        nth = { i: i + 1, n: names.length };
        const url = coreUrl(rel, names[i]);
        const f = await attempt(names[i], url);
        if (f) { await sidecars(url, f.uri); got.push(f); }
      }
      const b = bundle(got);
      if (b) return b;
    }
    nth = { i: 1, n: 2 };
    const locked = await attempt(pkgName(rel), pkgUrl(rel));
    nth = { i: 2, n: 2 };
    const legacy = await attempt(tarName(rel), tarUrl(rel));
    const b = bundle([locked, legacy].filter((f): f is FwFile => !!f));
    if (b) return b;
    throw new Error('이 버전에는 로봇 패키지가 없습니다');
  })();

  return {
    cancel: () => {
      cancelled = true;
      current?.cancel();
      for (const uri of staged) deleteAsync(uri, { idempotent: true }).catch(() => {});
    },
    done,
  };
}
