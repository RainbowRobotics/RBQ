import { screenState, jobLine, type JobSource } from './jobs';
import type { LogFileEntry, MediaKind } from '@/lib/rest';
import { t } from '@/lib/i18n';

export type MediaItem = { ip: string; kind: MediaKind; file: LogFileEntry };
export type MediaSaver = (ip: string, kind: MediaKind, path: string, size: number, canShare: (uri: string) => boolean) => Promise<string>;

export const useMediaDl = screenState({
  queue: [] as MediaItem[],
  busy: null as string | null,
  sec: 'photo' as MediaKind,
  err: null as { kind: MediaKind; text: string } | null,
  got: 0,
  share: [] as string[],
}, (s) => s.busy != null);

let saver: MediaSaver | null = null;

export function mediaDownload(items: MediaItem[], save: MediaSaver) {
  saver = save;
  const s = useMediaDl.getState();
  const same = (a: MediaItem, kind: MediaKind, path: string | null) => a.kind === kind && a.file.path === path;
  const fresh = items.filter((i) => !same(i, s.sec, s.busy) && !s.queue.some((q) => same(i, q.kind, q.file.path)));
  if (!fresh.length) return;
  if (s.busy) { useMediaDl.setState({ queue: [...s.queue, ...fresh] }); return; }
  useMediaDl.setState({ queue: fresh, err: null, got: 0 });
  void pump();
}

function take() {
  const [next, ...rest] = useMediaDl.getState().queue;
  if (next) useMediaDl.setState({ busy: next.file.path, sec: next.kind, queue: rest });
  return next;
}

const canShare = (uri: string) => {
  const s = useMediaDl.getState();
  if (s.open > 0) return true;
  if (!s.share.includes(uri)) useMediaDl.setState({ share: [...s.share, uri] });
  return false;
};

async function pump() {
  for (let it = take(); it; it = take()) {
    try {
      await saver!(it.ip, it.kind, it.file.path, it.file.size, canShare);
      useMediaDl.setState((s) => ({ got: s.got + 1 }));
    } catch (e) {
      useMediaDl.setState({ err: { kind: it.kind, text: (e as Error)?.message || t('파일을 받지 못했습니다') } });
    }
  }
  useMediaDl.setState({ busy: null });
}

const base = (p: string) => p.split('/').pop() ?? p;
type S = ReturnType<typeof useMediaDl.getState>;
export function mediaLine(s: S) {
  const run = s.busy ? `${t('받는 중…')} ${base(s.busy)}${s.queue.length ? ` +${s.queue.length}` : ''}` : null;
  return jobLine(s, {
    run,
    bad: s.err ? `${t('미디어')} — ${s.err.text}` : null,
    ok: s.got ? `${t('미디어를 받았습니다')}${s.got > 1 ? ` (${s.got})` : ''}${s.share.length ? ` — ${t('눌러서 공유')}` : ''}` : null,
  });
}

export const mediaJob: JobSource = {
  key: 'media',
  get route() { return `/media?sec=${useMediaDl.getState().sec}`; },
  icon: 'download', label: () => t('미디어 열기'),
  useLine: () => mediaLine(useMediaDl()),
  dismiss: () => useMediaDl.setState({ unseen: false, got: 0, err: null, share: [] }),
};
