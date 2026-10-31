import { streamerCommand, whenStreamerReady } from '@/lib/commandBus';
import { webrtcClient } from '@/lib/webrtcClient';
import { resolveEndpoints } from '@/lib/resolveEndpoints';
import { streamerBase } from '@/lib/endpoints';
import { useRobot } from '@/store/robot';
import { simEngine } from '@/lib/simEngine';
import { isDemo, demoDriver } from '@/lib/demoFlag';
import { t } from '@/lib/i18n';
import type { HistoryRow, Kind, QcStatus } from '@/lib/ptzQc';

const P = {
  status: '/api/ptz/qc/status', start: '/api/ptz/qc/start', stop: '/api/ptz/qc/stop', answer: '/api/ptz/qc/answer',
  history: '/api/ptz/qc/history', report: '/api/ptz/qc/report', file: '/api/ptz/qc/file', serial: '/api/ptz/qc/serial',
  media: '/api/ptz/qc/media',
} as const;

async function ready(passive = false) {
  const { ip, visionIp, conn } = useRobot.getState();
  if (!ip || conn !== 'connected' || simEngine.active) throw new Error(t('로봇에 연결되지 않았습니다'));
  if (passive) {
    if (!(await whenStreamerReady(0))) throw new Error(t('영상 서버에 아직 붙지 않았습니다'));
    return;
  }
  webrtcClient.ensureConnected(resolveEndpoints(ip, visionIp).vision);
  if (!(await whenStreamerReady(8000))) throw new Error(t('영상 서버에 연결하지 못했습니다 — 로봇의 비전이 켜져 있는지 확인해주세요'));
}

type Opts = { passive?: boolean };
async function call<T>(method: 'GET' | 'POST' | 'DELETE', path: string, body: Record<string, unknown> = {}, opts: Opts = {}): Promise<T> {
  const demo = demoDriver();
  if (demo) return (await demo.ptzQc(method, path, body)) as T;
  await ready(opts.passive);
  return (await streamerCommand(method, path, body)) as T;
}

export type StartReq = { level: 1 | 2 | 3; kind: Kind; soakMin?: number; presets?: boolean; steps?: number[]; base?: string };

async function chunked<T>(path: string, body: Record<string, unknown>, take: (r: any) => T, join: (parts: T[]) => T, lenOf: (x: T) => number): Promise<T> {
  for (let attempt = 0; attempt < 3; attempt++) {
    const parts: T[] = [];
    let off = 0, rev: number | null = null, changed = false;
    for (let guard = 0; guard < 200; guard++) {
      const r: any = await call('GET', path, { ...body, offset: off });
      if (rev == null) rev = r.rev ?? 0;
      else if ((r.rev ?? 0) !== rev) { changed = true; break; }
      const part = take(r);
      parts.push(part);
      off += lenOf(part);
      if (off >= (r.total ?? 0) || !lenOf(part)) return join(parts);
    }
    if (!changed) break;
  }
  throw new Error(t('보고서를 받는 동안 로봇에서 보고서가 계속 바뀌었습니다 — 잠시 뒤 다시 열어 주세요'));
}

export const ptzQcApi = {
  status: (o: Opts = {}) => call<QcStatus>('GET', P.status, {}, o),
  start: (r: StartReq) => call<{ queued?: boolean }>('POST', P.start, r as Record<string, unknown>),
  stop: (why?: string) => call('POST', P.stop, why ? { why } : {}),
  answer: (go: boolean) => call('POST', P.answer, { go }),
  history: (o: Opts = {}) => call<{ reports?: HistoryRow[] }>('GET', P.history, {}, o),
  report: async (stem: string): Promise<any> => {
    const text = await chunked<string>(P.report, { stem }, (r) => String(r.text ?? ''), (p) => p.join(''), (x) => x.length);
    return JSON.parse(text);
  },
  photo: async (path: string): Promise<string> => {
    const b64 = await chunked<string>(P.file, { path }, (r) => String(r.base64 ?? ''), (p) => p.join(''),
      (x) => Math.floor((x.length * 3) / 4) - (x.endsWith('==') ? 2 : x.endsWith('=') ? 1 : 0));
    return `data:image/jpeg;base64,${b64}`;
  },
  remove: (stem: string) => call<{ removed?: number }>('DELETE', P.report, { stem }),
  serial: (serial: string) => call('POST', P.serial, { serial }),
  mediaUrl: (path: string): string | null => {
    if (isDemo()) return null;
    const { ip, via } = useRobot.getState();
    if (via === 'remote') return null;
    return `${streamerBase(ip)}${P.media}?path=${encodeURIComponent(path)}`;
  },
};
