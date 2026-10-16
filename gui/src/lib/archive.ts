import type { ArchiveSegment } from '@/lib/rest';
import { t } from '@/lib/i18n';

export const DAY_MS = 24 * 3600 * 1000;

export const ARCHIVE_SPEEDS = [1, 2, 4, 8, 0.5] as const;

export type ArchiveZoom = 'fit' | 'day' | 'hour' | 'min10';
export const ZOOM_SPAN: Record<Exclude<ArchiveZoom, 'fit'>, number> = { day: DAY_MS, hour: 3600_000, min10: 600_000 };
const FIT_MIN_MS = 600_000;

export type ViewWindow = { from: number; to: number };

export function viewWindow(zoom: ArchiveZoom, segs: ArchiveSegment[], cam: string, dayStartMs: number, pos: number | null): ViewWindow {
  const dayEnd = dayStartMs + DAY_MS;
  const mine = segs.filter((x) => x.cam === cam);
  const recFrom = mine.length ? Math.max(dayStartMs, Math.min(...mine.map((x) => x.start))) : dayStartMs;
  const recTo = mine.length ? Math.min(dayEnd, Math.max(...mine.map((x) => x.end))) : dayEnd;
  if (zoom === 'day' || (zoom === 'fit' && !mine.length)) return { from: dayStartMs, to: dayEnd };
  if (zoom === 'fit') {
    const span = Math.max(FIT_MIN_MS, (recTo - recFrom) * 1.1);
    return clampWindow((recFrom + recTo) / 2 - span / 2, span, dayStartMs, dayEnd);
  }
  const span = ZOOM_SPAN[zoom];
  return clampWindow((pos ?? (recFrom + recTo) / 2) - span / 2, span, dayStartMs, dayEnd);
}

function clampWindow(from: number, span: number, lo: number, hi: number): ViewWindow {
  const f = Math.min(Math.max(lo, from), hi - span);
  return { from: f, to: f + span };
}

export function needsRecenter(w: ViewWindow, pos: number): boolean {
  const m = (w.to - w.from) * 0.05;
  return pos < w.from + m || pos > w.to - m;
}

export function coverage(segs: ArchiveSegment[], cam: string, w: ViewWindow): { from: number; to: number }[] {
  const out: { from: number; to: number }[] = [];
  const span = w.to - w.from;
  const list = segs.filter((s) => s.cam === cam).sort((a, b) => a.start - b.start);
  for (const s of list) {
    const from = Math.max(0, (s.start - w.from) / span);
    const to = Math.min(1, (s.end - w.from) / span);
    if (to <= from) continue;
    const last = out[out.length - 1];
    if (last && from - last.to <= 0.001) last.to = Math.max(last.to, to);
    else out.push({ from, to });
  }
  return out;
}

export function timeAt(frac: number, w: ViewWindow): number {
  return w.from + Math.min(1, Math.max(0, frac)) * (w.to - w.from);
}

export function ticks(w: ViewWindow, dayStartMs: number): { frac: number; label: string }[] {
  const span = w.to - w.from;
  const step = [3 * 3600_000, 3600_000, 30 * 60_000, 10 * 60_000, 5 * 60_000, 2 * 60_000, 60_000]
    .filter((st) => span / st <= 10).pop() ?? 3 * 3600_000;
  const out: { frac: number; label: string }[] = [];
  for (let t = Math.ceil((w.from - dayStartMs) / step) * step + dayStartMs; t <= w.to + 1; t += step) {
    out.push({ frac: (t - w.from) / span, label: clock(t, dayStartMs).slice(0, 5) });
  }
  return out;
}

export function spans(segs: ArchiveSegment[], cam: string): { start: number; end: number }[] {
  const out: { start: number; end: number }[] = [];
  for (const s of segs.filter((x) => x.cam === cam).sort((a, b) => a.start - b.start)) {
    const last = out[out.length - 1];
    if (last && s.start - last.end <= 5000) last.end = Math.max(last.end, s.end);
    else out.push({ start: s.start, end: s.end });
  }
  return out;
}

export function durLabel(ms: number): string {
  const sec = Math.max(0, Math.round(ms / 1000));
  if (sec < 60) return `${sec}${t('초')}`;
  const m = Math.floor(sec / 60);
  if (m < 60) return `${m}${t('분')}`;
  return `${Math.floor(m / 60)}${t('시간')} ${m % 60}${t('분')}`;
}

export function firstStart(segs: ArchiveSegment[], cam: string): number | null {
  const s = segs.filter((x) => x.cam === cam).sort((a, b) => a.start - b.start)[0];
  return s ? s.start : null;
}

export function clock(ms: number, dayStartMs: number): string {
  const s = Math.max(0, Math.floor((ms - dayStartMs) / 1000));
  const hh = Math.floor(s / 3600);
  const mm = Math.floor((s % 3600) / 60);
  return `${String(hh).padStart(2, '0')}:${String(mm).padStart(2, '0')}:${String(s % 60).padStart(2, '0')}`;
}

export function livePos(posMs: number, at: number, now: number, speed: number, running: boolean): number {
  return running ? posMs + (now - at) * speed : posMs;
}

export const fmtDay = (d: string) => (/^\d{8}$/.test(d) ? `${d.slice(0, 4)}-${d.slice(4, 6)}-${d.slice(6, 8)}` : d);

export function camLabel(cam: string): string {
  if (cam === 'front') return t('전방');
  if (cam === 'rear') return t('후방');
  return cam;
}

export function archiveErrText(e: unknown): string {
  const m = (e as Error)?.message ?? String(e);
  if (/status 404/.test(m)) return t('이 로봇은 상시 녹화 재생을 지원하지 않습니다 — 로봇 소프트웨어를 업데이트해주세요');
  if (/status 429/.test(m)) return t('다른 기기에서 이미 재생 중입니다 — 로봇은 동시에 두 곳까지만 재생합니다');
  if (/status 409/.test(m)) return t('영상 연결이 아직 준비되지 않았습니다 — 잠시 뒤 다시 눌러주세요');
  if (/status 400/.test(m)) return t('영상 연결을 찾지 못했습니다 — 화면을 나갔다 다시 들어와 주세요');
  return m;
}
