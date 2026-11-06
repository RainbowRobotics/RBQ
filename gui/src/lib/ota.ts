import { openFileStream } from './fileStream';
import { coreSystemOf, fileSidecars, type FwSidecars } from './firmwareRelease';
import { isDemo } from './demoFlag';
import { RemoteUnsupportedError, TunnelWebSocket, tunnelReady, viaRemote } from './robotTunnel';
import { t } from '@/lib/i18n';

export const OTA_PORT = 56785;
export const VISION_OTA_IP = '192.168.0.12';
export const visionOtaIp = (visionIp: string | null | undefined) => (visionIp ?? '').trim() || VISION_OTA_IP;
const AUTH_TOKEN = process.env.EXPO_PUBLIC_ROBOT_OTA_TOKEN || '';
export const otaAvailable = AUTH_TOKEN !== '';
const DEST_PATH = 'rbq_ws/patches';
const CHUNK = 64 * 1024;
const WINDOW = 4 * 1024 * 1024;
const REMOTE_WINDOW = 256 * 1024;

export const AUTO_DEPLOY_RE = /^(RBQ-.+(\.tar\.gz|\.zip)|core-.+\.rbq)$/;

export interface OtaProgress {
  phase: 'connecting' | 'handshake' | 'transfer' | 'reconnecting' | 'applying' | 'complete';
  attempt?: number;
  acked: number;
  total: number;
  percentage: number;
}

export interface OtaFile { uri: string; name: string; size: number }

export interface OtaHandle {
  cancel: () => void;
  done: Promise<void>;
}

const isPackage = (name: string) => name.endsWith('.zip');

export function packageMismatch(name: string, system: string | undefined): string | null {
  const pkg = coreSystemOf(name);
  if (!system) return pkg ? t('이 로봇은 시스템별 패키지(core-*.rbq)를 모르는 구형입니다 — 시스템을 알리지 않는 로봇에는 RBQ-* 패키지를 보내세요') : null;
  if (/^RBQ-/.test(name)) return t('이 로봇은 RBQ-* 패키지를 받지 않습니다 — 로봇을 연결한 채 다시 받으세요');
  if (pkg && pkg !== system) return `${t('이 패키지의 대상은')} ${pkg}, ${t('로봇은')} ${system}${t('입니다 — 로봇을 연결한 채 다시 받으세요')}`;
  return null;
}

function otaSocket(ip: string, path = '/'): WebSocket {
  if (viaRemote()) {
    if (!tunnelReady()) throw new RemoteUnsupportedError(`:${OTA_PORT}`);
    return new TunnelWebSocket(OTA_PORT, path) as unknown as WebSocket;
  }
  return new WebSocket(`ws://${ip}:${OTA_PORT}${path}`);
}
const authPath = () => `/?auth=${encodeURIComponent(AUTH_TOKEN)}`;

export function otaProbe(ip: string, timeoutMs = viaRemote() ? 10000 : 3000): Promise<{ protocol: number; system?: string } | null> {
  if (isDemo() || !ip) return Promise.resolve(null);
  return new Promise((resolve) => {
    let ws: WebSocket | null = null;
    const end = (v: { protocol: number; system?: string } | null) => {
      clearTimeout(timer);
      try { ws?.close(); } catch { }
      resolve(v);
    };
    const timer = setTimeout(() => end(null), timeoutMs);
    try { ws = otaSocket(ip); } catch { end(null); return; }
    ws.onmessage = (ev) => {
      try {
        const m = JSON.parse(String(ev.data));
        if (m?.type === 'connected') {
          end({ protocol: Number(m.protocol ?? 1), system: typeof m.system === 'string' ? m.system : undefined });
        }
      } catch { }
    };
    ws.onerror = () => end(null);
    ws.onclose = () => end(null);
  });
}

const HANDSHAKE_MS = 20_000;
const ACK_MS = 30_000;
const RETRY_MAX = 6;
const RETRY_GAP_MS = 3000;
const APPLY_POLL_MS = 5000;
const APPLY_MAX_MS = 35 * 60_000;

type Outcome = { ok: true } | { error: Error } | { dropped: 'before' | 'transfer' | 'applying'; connected: boolean };
type Busy = { stage?: string; fileName?: string };
type Last = { ok?: boolean; fileName?: string; message?: string; at?: number };

export function otaTransfer(
  ip: string,
  file: OtaFile,
  onProgress: (p: OtaProgress) => void,
  alts: OtaFile[] = [],
): OtaHandle {
  let ws: WebSocket | null = null;
  let cancelled = false;
  let settled = false;
  const waiters = new Set<() => void>();
  const wakeAll = () => { for (const w of [...waiters]) w(); waiters.clear(); };
  let settle: (err?: Error) => void = () => {};
  const done = new Promise<void>((resolve, reject) => {
    settle = (err) => {
      if (settled) return;
      settled = true;
      try { ws?.close(); } catch { }
      wakeAll();
      err ? reject(err) : resolve();
    };
  });
  const sleep = (ms: number) => new Promise<void>((r) => {
    const w = () => { clearTimeout(tm); waiters.delete(w); r(); };
    const tm = setTimeout(w, ms);
    waiters.add(w);
  });

  let active = file;
  let chosen = false;
  let protocol = 1;
  let acked = 0;
  let applying = false;
  const startedAt = Date.now();
  const cands = [file, ...alts];
  let extras: FwSidecars[] = [];
  let extra: FwSidecars = {};
  const emit = (p: Partial<OtaProgress> & { phase: OtaProgress['phase'] }) =>
    onProgress({ acked, total: active.size, percentage: active.size ? Math.floor((acked * 100) / active.size) : 0, ...p });

  const once = () => new Promise<Outcome>((resolveOnce) => {
    let ended = false;
    let connected = false;
    let stage: 'before' | 'transfer' | 'applying' = applying ? 'applying' : 'before';
    let timer: ReturnType<typeof setTimeout> | null = null;
    const arm = (ms: number) => { if (timer) clearTimeout(timer); timer = setTimeout(() => end({ dropped: stage, connected }), ms); };
    const end = (o: Outcome) => {
      if (ended) return;
      ended = true;
      if (timer) clearTimeout(timer);
      try { ws?.close(); } catch { }
      wakeAll();
      resolveOnce(o);
    };
    try { ws = otaSocket(ip, authPath()); } catch (e) {
      if (e instanceof RemoteUnsupportedError && !tunnelReady() && (stage !== 'before' || acked > 0)) end({ dropped: stage, connected: false });
      else end({ error: e instanceof Error ? e : new Error(String(e)) });
      return;
    }
    const sock = ws;
    const window = viaRemote() ? REMOTE_WINDOW : WINDOW;
    (sock as unknown as { binaryType: string }).binaryType = 'arraybuffer';
    arm(HANDSHAKE_MS);

    let sent = 0;
    let lastEmit = 0;

    const pump = async (resumeFrom: number) => {
      const reader = openFileStream(active.uri);
      let skip = resumeFrom;
      sent = resumeFrom;
      acked = resumeFrom;
      try {
        for (;;) {
          const { done: eof, value: chunk } = await reader.read();
          if (eof || ended) break;
          if (skip >= chunk.byteLength) { skip -= chunk.byteLength; continue; }
          const value = skip > 0 ? chunk.subarray(skip) : chunk;
          skip = 0;
          for (let off = 0; off < value.byteLength; off += CHUNK) {
            if (ended || cancelled) return;
            while (sent - acked >= window && !ended && !cancelled) {
              await new Promise<void>((r) => { waiters.add(r); });
            }
            if (ended || cancelled) return;
            const part = value.subarray(off, Math.min(off + CHUNK, value.byteLength));
            sock.send(part.slice().buffer);
            sent += part.byteLength;
            if (sent % (1024 * 1024) === 0) await new Promise<void>((r) => setTimeout(r, 0));
          }
        }
      } catch (e) {
        if (!ended) end({ error: e instanceof Error ? e : new Error(String(e)) });
      } finally {
        reader.cancel().catch(() => {});
      }
    };

    sock.onmessage = (ev) => {
      if (ended || cancelled) return;
      let msg: { type?: string; message?: string; received?: number; percentage?: number; step?: string };
      try { msg = JSON.parse(String(ev.data)); } catch { return; }
      switch (msg.type) {
        case 'connected': {
          connected = true;
          const m = msg as { protocol?: number; system?: unknown; busy?: Busy; last?: Last };
          protocol = Number(m.protocol ?? 1);
          const busy = m.busy;
          const last = m.last;
          const finished = chosen && !busy && last && last.fileName === active.name && (last.at ?? 0) * 1000 >= startedAt - 60_000;
          if (finished) {
            end(last.ok ? { ok: true } : { error: new Error(last.message || t('로봇이 설치에 실패했습니다')) });
            break;
          }
          if (applying) {
            if (busy && busy.fileName === active.name && busy.stage !== 'receiving') { arm(APPLY_MAX_MS); emit({ phase: 'applying' }); break; }
            end({ error: new Error(t('로봇이 설치를 끝냈는지 확인하지 못했습니다 — 로봇 버전을 확인하세요')) });
            break;
          }
          if (!chosen) {
            chosen = true;
            const sys = typeof m.system === 'string' ? m.system : undefined;
            const fits = (f: OtaFile) => !packageMismatch(f.name, sys) && !(isPackage(f.name) && protocol < 2);
            const pick = cands.find(fits);
            if (!pick) {
              const why = cands.map((f) => packageMismatch(f.name, sys)).find((x) => x)
                ?? '이 로봇은 구형이라 비번 걸린 패키지를 설치하지 못합니다 — 전체 아카이브를 받아 다시 시도하세요';
              end({ error: new Error(why) });
              break;
            }
            active = pick;
            extra = extras[cands.indexOf(pick)] ?? {};
          }
          if (busy && busy.fileName === active.name && busy.stage && busy.stage !== 'receiving') {
            applying = true; stage = 'applying'; arm(APPLY_MAX_MS); emit({ phase: 'applying' });
            break;
          }
          emit({ phase: 'handshake' });
          const start: Record<string, unknown> = {
            type: 'start_transfer', auth: AUTH_TOKEN, protocol: 2,
            fileName: active.name, fileSize: active.size, destinationPath: DEST_PATH, resume: true,
            ...(extra.sha256 ? { sha256: extra.sha256 } : {}),
            ...(extra.signature ? { signature: extra.signature } : {}),
          };
          sock.send(JSON.stringify(start));
          arm(ACK_MS);
          break;
        }
        case 'ready': {
          stage = 'transfer';
          arm(ACK_MS);
          const from = Number((msg as { resumeFrom?: number }).resumeFrom ?? 0);
          emit({ phase: 'transfer', acked: from });
          void pump(from);
          break;
        }
        case 'progress':
          acked = msg.received ?? acked;
          arm(ACK_MS);
          if (Date.now() - lastEmit > 150) {
            lastEmit = Date.now();
            emit({ phase: 'transfer', percentage: msg.percentage ?? 0 });
          }
          if (sent - acked < window) wakeAll();
          break;
        case 'complete':
          acked = active.size;
          if (protocol >= 2) { applying = true; stage = 'applying'; arm(APPLY_MAX_MS); emit({ phase: 'applying' }); break; }
          end({ ok: true });
          break;
        case 'applying':
          applying = true; stage = 'applying';
          arm(APPLY_MAX_MS);
          emit({ phase: 'applying' });
          break;
        case 'deployed':
          end({ ok: true });
          break;
        case 'deploy_failed':
          end({ error: new Error(msg.message ?? '로봇이 설치에 실패했습니다') });
          break;
        case 'error':
          if (/^Transfer timed out/.test(msg.message ?? '')) { end({ dropped: 'transfer', connected }); break; }
          end({ error: new Error(msg.message ?? '로봇이 전송을 거부했습니다') });
          break;
      }
    };
    sock.onerror = () => end({ dropped: stage, connected });
    sock.onclose = () => end({ dropped: stage, connected });
  });

  const run = async () => {
    if (isDemo()) throw new Error('데모 모드 — 펌웨어 전송 없음');
    emit({ phase: 'connecting' });
    extras = await Promise.all(cands.map((f) => fileSidecars(f).catch((): FwSidecars => ({}))));
    let tries = 0;
    let everConnected = false;
    let applyingSince = 0;
    for (;;) {
      if (cancelled || settled) return;
      const before = acked;
      const o = await once();
      if (cancelled || settled) return;
      if ('ok' in o) { emit({ phase: 'complete', acked: active.size, percentage: 100 }); settle(); return; }
      if ('error' in o) { settle(o.error); return; }
      everConnected ||= o.connected;
      if (applying) {
        applyingSince ||= Date.now();
        if (Date.now() - applyingSince > APPLY_MAX_MS) {
          settle(new Error(t('로봇이 설치를 끝냈는지 확인하지 못했습니다 — 로봇 버전을 확인하세요')));
          return;
        }
        emit({ phase: 'applying' });
        await sleep(APPLY_POLL_MS);
        continue;
      }
      if (!everConnected && tries >= 1) {
        settle(new Error(`${ip}:${OTA_PORT} ${t('연결 실패 — 그 주소에 OTA 데몬이 없거나 PC 가 꺼져 있습니다')}`));
        return;
      }
      if (acked > before) tries = 0;
      if (++tries > RETRY_MAX) {
        settle(new Error(t('연결이 계속 끊깁니다 — 연결을 확인하고 다시 보내면 받은 데까지 이어서 보냅니다')));
        return;
      }
      emit({ phase: 'reconnecting', attempt: tries });
      await sleep(RETRY_GAP_MS * tries);
    }
  };

  run().catch((e) => settle(e instanceof Error ? e : new Error(String(e))));

  return {
    cancel: () => { cancelled = true; settle(new Error('cancelled')); },
    done,
  };
}
