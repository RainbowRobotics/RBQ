import { create } from 'zustand';
import { rest, type LogFileEntry, type AudioRecordSource, type MediaKind } from '@/lib/rest';
import { useRobot } from '@/store/robot';
import { t } from '@/lib/i18n';
import { webrtcClient } from '@/lib/webrtcClient';
import { whenStreamerReady } from '@/lib/commandBus';
import { isDemo } from '@/lib/demoFlag';
import { resolveEndpoints } from '@/lib/resolveEndpoints';
import { assetBytes } from '@/lib/assetBytes';
import { Buffer } from 'buffer';
import type { BundledSound } from '@/lib/sounds';

export type { MediaKind } from '@/lib/rest';

async function streamerReady(ip: string) {
  if (isDemo()) return;
  if (!ip) throw new Error(t('로봇에 연결되지 않았습니다'));
  webrtcClient.ensureConnected(resolveEndpoints(ip, useRobot.getState().visionIp).vision);
  if (!(await whenStreamerReady(8000))) throw new Error(t('영상 서버에 연결하지 못했습니다 — 로봇의 비전이 켜져 있는지 확인해주세요'));
}

type ListState = {
  files: LogFileEntry[];
  loading: boolean;
  loaded: boolean;
  error: string | null;
};

const emptyList: ListState = { files: [], loading: false, loaded: false, error: null };

export type ShotResult = { at: number; ok: boolean; msg: string; sec?: MediaKind };

type MediaState = {
  lists: Record<MediaKind, ListState>;
  recordingPath: string | null;
  recordingSince: number;
  recordingSource: AudioRecordSource;
  playingPath: string | null;
  loop: boolean;
  shooting: boolean;
  videoSince: number;
  videoBusy: boolean;
  lastShot: ShotResult | null;
  lastRecQuiet: { path: string; source: AudioRecordSource; dbfs: number } | null;
  soundBusy: string | null;
  setLoop: (v: boolean) => void;
  setRecordingSource: (v: AudioRecordSource) => void;
};

export const useMedia = create<MediaState>((set) => ({
  lists: { photo: emptyList, clip: emptyList, archive: emptyList, audio: emptyList },
  recordingPath: null,
  recordingSince: 0,
  recordingSource: 'uplink',
  playingPath: null,
  loop: false,
  shooting: false,
  videoSince: 0,
  videoBusy: false,
  lastShot: null,
  lastRecQuiet: null,
  soundBusy: null,
  setLoop: (v) => set({ loop: v }),
  setRecordingSource: (v) => set({ recordingSource: v }),
}));

function patchList(kind: MediaKind, patch: Partial<ListState>) {
  useMedia.setState((s) => ({ lists: { ...s.lists, [kind]: { ...s.lists[kind], ...patch } } }));
}

function newestFirst(files: LogFileEntry[]): LogFileEntry[] {
  return [...files].sort((a, b) => b.mtime - a.mtime);
}

export const SOUNDS_REV = 2;
export const soundPath = (s: Pick<BundledSound, 'id'>) => `library/${s.id}-r${SOUNDS_REV}.wav`;

export function staleSoundPaths(id: string, current: string, paths: string[]): string[] {
  const re = new RegExp(`^library/${id.replace(/[.*+?^${}()|[\]\\]/g, '\\$&')}(-r\\d+)?\\.wav$`);
  return paths.filter((p) => p !== current && re.test(p));
}

export const SILENT_DBFS = -58;

const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

let playEndTimer: ReturnType<typeof setTimeout> | null = null;

export const media = {
  async refresh(kind: MediaKind) {
    const ip = useRobot.getState().ip;
    if (!ip) { patchList(kind, { error: t('로봇에 연결되지 않았습니다'), loaded: true }); return; }
    patchList(kind, { loading: true, error: null });
    try {
      const r = await rest.mediaList(ip, kind);
      patchList(kind, { files: newestFirst(r?.files ?? []), loading: false, loaded: true, error: null });
    } catch (e) {
      patchList(kind, { loading: false, loaded: true, error: describe(e) });
    }
  },

  async snapshot(label?: string): Promise<ShotResult> {
    const cur = useMedia.getState();
    if (cur.shooting) return cur.lastShot ?? { at: Date.now(), ok: false, msg: '' };
    useMedia.setState({ shooting: true });
    const ip = useRobot.getState().ip;
    const before = new Set(cur.lists.photo.files.map((f) => f.path));
    let res: ShotResult;
    try {
      await streamerReady(ip);
      await rest.snapshotSave(ip, label);
      if (isDemo()) {
        res = { at: Date.now(), ok: true, msg: t('사진을 저장했습니다'), sec: 'photo' };
      } else {
        let fresh = 0;
        for (const wait of [350, 900]) {
          await sleep(wait);
          await media.refresh('photo');
          fresh = useMedia.getState().lists.photo.files.filter((f) => !before.has(f.path)).length;
          if (fresh > 0) break;
        }
        res = fresh > 0
          ? { at: Date.now(), ok: true, msg: t('사진을 저장했습니다'), sec: 'photo' }
          : { at: Date.now(), ok: false, msg: t('사진이 저장되지 않았습니다 — 카메라 영상이 들어오는지 확인해주세요') };
      }
    } catch (e) {
      res = { at: Date.now(), ok: false, msg: (e as Error)?.message ?? t('사진을 찍지 못했습니다') };
    }
    useMedia.setState({ shooting: false, lastShot: res });
    return res;
  },

  async toggleVideo(): Promise<ShotResult> {
    const cur = useMedia.getState();
    if (cur.videoBusy) return cur.lastShot ?? { at: Date.now(), ok: false, msg: '' };
    const ip = useRobot.getState().ip;
    const starting = cur.videoSince === 0;
    useMedia.setState({ videoBusy: true });
    let res: ShotResult;
    try {
      await streamerReady(ip);
      const r = await rest.videoRecord(ip, starting ? 'start' : 'stop');
      if (starting) {
        useMedia.setState({ videoSince: r?.since && r.since > 0 ? r.since : Date.now() });
        res = { at: Date.now(), ok: true, msg: t('영상 녹화를 시작했습니다') };
      } else {
        const since = cur.videoSince;
        useMedia.setState({ videoSince: 0 });
        res = { at: Date.now(), ok: true, msg: t('영상을 저장했습니다'), sec: 'clip' };
        setTimeout(() => {
          void media.refresh('clip').then(() => {
            const got = useMedia.getState().lists.clip.files.some((f) => f.mtime >= since - 2000);
            if (!got) useMedia.setState({ lastShot: { at: Date.now(), ok: false, msg: t('녹화 파일이 보이지 않습니다 — 잠시 뒤 목록을 새로 고쳐 보세요') } });
          });
        }, 2500);
      }
    } catch (e) {
      const m = (e as Error)?.message ?? '';
      if (!starting && /not recording/.test(m)) useMedia.setState({ videoSince: 0 });
      res = { at: Date.now(), ok: false, msg:
        /event ring disabled/.test(m) ? t('로봇의 블랙박스 녹화가 꺼져 있어 녹화할 수 없습니다')
        : /no camera encoder/.test(m) ? t('카메라 녹화를 시작하지 못했습니다 — 카메라 영상이 들어오는지 확인해주세요')
        : /not recording/.test(m) ? t('이미 녹화가 멈춰 있습니다')
        : m || (starting ? t('녹화를 시작하지 못했습니다') : t('녹화를 멈추지 못했습니다')) };
    }
    useMedia.setState({ videoBusy: false, lastShot: res });
    return res;
  },

  async startRecord(source: AudioRecordSource) {
    const ip = useRobot.getState().ip;
    await streamerReady(ip);
    useMedia.setState({ lastRecQuiet: null });
    const r = await rest.audioRecord(ip, 'start', source);
    useMedia.setState({
      recordingPath: r?.path ?? r?.audio?.recording_path ?? '',
      recordingSince: Date.now(),
      recordingSource: source,
    });
  },

  async stopRecord() {
    const ip = useRobot.getState().ip;
    await streamerReady(ip);
    const r = await rest.audioRecord(ip, 'stop');
    const { recordingSource: source } = useMedia.getState();
    const quiet = r?.path && r.rms_dbfs != null && r.rms_dbfs < SILENT_DBFS ? { path: r.path, source, dbfs: r.rms_dbfs } : null;
    useMedia.setState({ recordingPath: null, recordingSince: 0, lastRecQuiet: quiet });
    await media.refresh('audio');
  },

  async play(file: LogFileEntry, loop = useMedia.getState().loop) {
    const ip = useRobot.getState().ip;
    if (useMedia.getState().playingPath === file.path) { await media.stopPlay(); return; }
    await streamerReady(ip);
    await rest.audioPlay(ip, file.path, loop);
    useMedia.setState({ playingPath: file.path });
    if (playEndTimer) clearTimeout(playEndTimer);
    playEndTimer = null;
    if (!loop) {
      const ms = clipSeconds(file.size) * 1000 + 400;
      playEndTimer = setTimeout(() => {
        if (useMedia.getState().playingPath === file.path) useMedia.setState({ playingPath: null });
      }, ms);
    }
  },

  async playSound(s: BundledSound, loop?: boolean) {
    const path = soundPath(s);
    if (useMedia.getState().playingPath === path) { await media.stopPlay(); return; }
    const name = path.replace(/^library\//, '').replace(/\.wav$/, '.mp3');
    await media.playLibrary(path, s.id, async () => new Uint8Array(await assetBytes(s.asset)), name, loop);
    const stale = staleSoundPaths(s.id, path, useMedia.getState().lists.audio.files.map((f) => f.path));
    if (stale.length) void rest.mediaDelete(useRobot.getState().ip, 'audio', stale).catch(() => {});
  },

  async uploadUserFile(name: string, bytes: Uint8Array): Promise<string> {
    if (bytes.byteLength > MAX_UPLOAD) throw new Error(t('파일이 너무 큽니다 — 10 MB 이하만 올릴 수 있습니다'));
    const ip = useRobot.getState().ip;
    await streamerReady(ip);
    useMedia.setState({ soundBusy: 'upload' });
    try {
      const path = await uploadBytes(ip, userSoundName(name), bytes);
      await media.refresh('audio');
      return path;
    } finally {
      useMedia.setState({ soundBusy: null });
    }
  },

  async playLibrary(path: string, busyKey: string, load: () => Promise<Uint8Array>, uploadName: string, loop?: boolean) {
    const ip = useRobot.getState().ip;
    await streamerReady(ip);
    const find = () => useMedia.getState().lists.audio.files.find((f) => f.path === path);
    if (!useMedia.getState().lists.audio.loaded) await media.refresh('audio');
    if (!find()) {
      useMedia.setState({ soundBusy: busyKey });
      try {
        await uploadBytes(ip, uploadName, await load());
        await media.refresh('audio');
      } finally {
        useMedia.setState({ soundBusy: null });
      }
    }
    const f = find();
    if (!f) throw new Error(t('로봇에 소리를 올리지 못했습니다'));
    await media.play(f, loop);
  },

  async deleteFiles(kind: MediaKind, paths: string[]): Promise<number> {
    const ip = useRobot.getState().ip;
    if (paths.includes(useMedia.getState().playingPath ?? '')) await media.stopPlay();
    let skipped = 0;
    try {
      for (let i = 0; i < paths.length; i += 200) {
        const r = await rest.mediaDelete(ip, kind, paths.slice(i, i + 200));
        skipped += r?.skipped?.length ?? 0;
      }
    } finally {
      await media.refresh(kind);
    }
    return skipped;
  },

  async stopPlay() {
    const ip = useRobot.getState().ip;
    if (playEndTimer) { clearTimeout(playEndTimer); playEndTimer = null; }
    useMedia.setState({ playingPath: null });
    await streamerReady(ip);
    await rest.audioPlayStop(ip);
  },
};

const MAX_UPLOAD = 10 * 1024 * 1024;
const UPLOAD_CHUNK = 48 * 1024;

async function uploadBytes(ip: string, name: string, bytes: Uint8Array): Promise<string> {
  let path = '';
  for (let off = 0; off < bytes.byteLength; off += UPLOAD_CHUNK) {
    const part = bytes.subarray(off, Math.min(off + UPLOAD_CHUNK, bytes.byteLength));
    const r = await rest.audioUpload(ip, { name, offset: off, total: bytes.byteLength, data: Buffer.from(part).toString('base64') });
    if (r?.path) path = r.path;
  }
  if (!path) throw new Error(t('로봇에 소리를 올리지 못했습니다'));
  return path;
}

export function userSoundName(name: string): string {
  const base = name.replace(/\.[^./\\]*$/, '').replace(/[^0-9A-Za-z가-힣_-]+/g, '_').replace(/^_+|_+$/g, '').slice(0, 60);
  const ext = (/\.([0-9A-Za-z]{1,5})$/.exec(name)?.[1] ?? 'bin').toLowerCase();
  return `my-${base || 'sound'}.${ext}`;
}

function describe(e: unknown): string {
  const name = (e as { name?: string } | null)?.name;
  if (name === 'RemoteUnsupportedError') {
    return t('원격(중계) 연결에서는 파일 목록을 받을 수 없습니다 — 같은 망에서 직접 연결해주세요');
  }
  const msg = (e as { message?: string } | null)?.message;
  return msg ? `${t('목록을 받지 못했습니다')} — ${msg}` : t('목록을 받지 못했습니다');
}


export function humanSize(bytes: number): string {
  if (bytes < 1024) return `${bytes} B`;
  if (bytes < 1024 * 1024) return `${(bytes / 1024).toFixed(0)} KB`;
  if (bytes < 1024 * 1024 * 1024) return `${(bytes / 1024 / 1024).toFixed(1)} MB`;
  return `${(bytes / 1024 / 1024 / 1024).toFixed(2)} GB`;
}

export function clipSeconds(bytes: number): number {
  return Math.max(0, (bytes - 44) / (48000 * 2));
}

export function mmss(sec: number): string {
  const s = Math.max(0, Math.floor(sec));
  return `${Math.floor(s / 60)}:${String(s % 60).padStart(2, '0')}`;
}

export function stampParts(path: string): { day: string; time: string; key: string } | null {
  const m = /(\d{4})(\d{2})(\d{2})_(\d{2})(\d{2})(\d{2})/.exec(path);
  if (!m) return null;
  return { day: `${m[1]}${m[2]}${m[3]}`, time: `${m[4]}:${m[5]}:${m[6]}`, key: `${m[1]}${m[2]}${m[3]}_${m[4]}${m[5]}${m[6]}` };
}

export function prefixOf(path: string): string {
  const base = path.split('/').pop() ?? '';
  return base.split('_')[0] ?? '';
}

export type Capture = {
  key: string; day: string; time: string; mtime: number;
  front?: LogFileEntry; rear?: LogFileEntry; other: LogFileEntry[];
};

export function groupCaptures(files: LogFileEntry[]): Capture[] {
  const by = new Map<string, Capture>();
  for (const f of files) {
    const st = stampParts(f.path);
    const key = st?.key ?? f.path;
    let cap = by.get(key);
    if (!cap) {
      cap = { key, day: st?.day ?? '', time: st?.time ?? '', mtime: f.mtime, other: [] };
      by.set(key, cap);
    }
    cap.mtime = Math.max(cap.mtime, f.mtime);
    const p = prefixOf(f.path);
    if (p === 'front' && !cap.front) cap.front = f;
    else if (p === 'rear' && !cap.rear) cap.rear = f;
    else cap.other.push(f);
  }
  return [...by.values()].sort((a, b) => b.mtime - a.mtime);
}

export function groupByDay<T>(items: T[], dayOf: (x: T) => string): { day: string; items: T[] }[] {
  const out: { day: string; items: T[] }[] = [];
  for (const it of items) {
    const d = dayOf(it);
    const last = out[out.length - 1];
    if (last && last.day === d) last.items.push(it);
    else out.push({ day: d, items: [it] });
  }
  return out;
}

export function dayLabel(day: string, now = new Date()): string {
  if (!/^\d{8}$/.test(day)) return t('날짜 미상');
  const d = new Date(Number(day.slice(0, 4)), Number(day.slice(4, 6)) - 1, Number(day.slice(6, 8)));
  const today = new Date(now.getFullYear(), now.getMonth(), now.getDate());
  const diff = Math.round((today.getTime() - d.getTime()) / 86400000);
  if (diff === 0) return t('오늘');
  if (diff === 1) return t('어제');
  const wd = [t('일'), t('월'), t('화'), t('수'), t('목'), t('금'), t('토')][d.getDay()];
  return `${d.getMonth() + 1}/${d.getDate()} (${wd})`;
}
