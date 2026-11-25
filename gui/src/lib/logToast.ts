import type { LogLevel, LogLine } from '@/types/robot';

export type Toast = { id: number; ts: string; msg: string; level: LogLevel; process?: string; count: number };

export function fitToasts(ids: number[], heights: Record<number, number>, room: number, gap: number): Set<number> {
  const out = new Set<number>();
  let used = 0;
  for (let i = ids.length - 1; i >= 0; i--) {
    const next = used + (out.size ? gap : 0) + (heights[ids[i]] ?? 0);
    if (next > room && out.size) break;
    used = next;
    out.add(ids[i]);
  }
  return out;
}

export function mergeToasts(prev: Toast[], fresh: LogLine[], seedId: number, max: number) {
  const next = prev.map((t) => ({ ...t }));
  const bumped: number[] = [];
  const dropped: number[] = [];
  let id = seedId;
  for (const line of fresh) {
    const hit = next.find((x) => x.msg === line.msg);
    if (hit) {
      hit.count += 1;
      bumped.push(hit.id);
      continue;
    }
    id += 1;
    next.push({ id, ts: line.ts, msg: line.msg, level: line.level, process: line.process, count: 1 });
    bumped.push(id);
  }
  while (next.length > max) {
    const gone = next.shift();
    if (gone) dropped.push(gone.id);
  }
  return { next, bumped, dropped, lastId: id };
}
