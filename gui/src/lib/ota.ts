import { openFileStream } from './fileStream';
import { isDemo } from './demoFlag';
import { t } from '@/lib/i18n';

export const OTA_PORT = 56785;
const AUTH_TOKEN = process.env.EXPO_PUBLIC_ROBOT_OTA_TOKEN || '';
export const otaAvailable = AUTH_TOKEN !== '';
const DEST_PATH = 'rbq_ws/patches';
const CHUNK = 64 * 1024;
const WINDOW = 4 * 1024 * 1024;

export const AUTO_DEPLOY_RE = /^RBQ-.+(\.tar\.gz|\.zip)$/;

export interface OtaProgress {
  phase: 'connecting' | 'handshake' | 'transfer' | 'complete';
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

export function otaTransfer(
  ip: string,
  file: OtaFile,
  onProgress: (p: OtaProgress) => void,
  fallback?: OtaFile,
): OtaHandle {
  let ws: WebSocket | null = null;
  let cancelled = false;
  let finish: (err?: Error) => void = () => {};

  const done = new Promise<void>((resolve, reject) => {
    finish = (err) => {
      finish = () => {};
      try { ws?.close(); } catch { }
      err ? reject(err) : resolve();
    };
  });

  const run = async () => {
    if (isDemo()) throw new Error('데모 모드 — 펌웨어 전송 없음');
    onProgress({ phase: 'connecting', acked: 0, total: file.size, percentage: 0 });
    let active = file;
    ws = new WebSocket(`ws://${ip}:${OTA_PORT}`);
    (ws as unknown as { binaryType: string }).binaryType = 'arraybuffer';

    let acked = 0;
    let sent = 0;
    let resumeFrom = 0;
    let protocol = 1;
    let lastEmit = 0;
    let ready: () => void = () => {};
    const readyP = new Promise<void>((r) => { ready = r; });
    let windowOpen: (() => void) | null = null;

    ws.onmessage = (ev) => {
      if (cancelled) return;
      let msg: { type?: string; message?: string; received?: number; percentage?: number };
      try { msg = JSON.parse(String(ev.data)); } catch { return; }
      switch (msg.type) {
        case 'connected': {
          protocol = Number((msg as { protocol?: number }).protocol ?? 1);
          if (isPackage(active.name) && protocol < 2) {
            if (!fallback) {
              finish(new Error('이 로봇은 구형이라 비번 걸린 패키지를 설치하지 못합니다 — 전체 아카이브를 받아 다시 시도하세요'));
              break;
            }
            active = fallback;
          }
          onProgress({ phase: 'handshake', acked: 0, total: active.size, percentage: 0 });
          const start: Record<string, unknown> = {
            type: 'start_transfer', auth: AUTH_TOKEN, protocol: 2,
            fileName: active.name, fileSize: active.size, destinationPath: DEST_PATH, resume: true,
          };
          ws!.send(JSON.stringify(start));
          break;
        }
        case 'ready':
          resumeFrom = Number((msg as { resumeFrom?: number }).resumeFrom ?? 0);
          ready();
          break;
        case 'progress':
          acked = msg.received ?? acked;
          if (Date.now() - lastEmit > 150) {
            lastEmit = Date.now();
            onProgress({ phase: 'transfer', acked, total: active.size, percentage: msg.percentage ?? 0 });
          }
          if (windowOpen && sent - acked < WINDOW) { windowOpen(); windowOpen = null; }
          break;
        case 'complete':
          if (protocol >= 2) break;
          onProgress({ phase: 'complete', acked: active.size, total: active.size, percentage: 100 });
          finish();
          break;
        case 'applying':
          break;
        case 'deployed':
          onProgress({ phase: 'complete', acked: active.size, total: active.size, percentage: 100 });
          finish();
          break;
        case 'deploy_failed':
          finish(new Error(msg.message ?? '로봇이 설치에 실패했습니다'));
          break;
        case 'error':
          finish(new Error(msg.message ?? '로봇이 전송을 거부했습니다'));
          break;
      }
    };
    ws.onerror = () => finish(new Error(
      `${ip}:${OTA_PORT} ${t('연결 실패 — 그 주소에 OTA 데몬이 없거나 PC 가 꺼져 있습니다')}`));
    ws.onclose = () => finish(new Error(cancelled ? 'cancelled' : '연결이 끊어졌습니다'));

    await readyP;

    const reader = openFileStream(active.uri);
    let skip = resumeFrom;
    sent = resumeFrom;
    acked = resumeFrom;
    try {
      for (;;) {
        const { done: eof, value: chunk } = await reader.read();
        if (eof) break;
        if (skip >= chunk.byteLength) { skip -= chunk.byteLength; continue; }
        const value = skip > 0 ? chunk.subarray(skip) : chunk;
        skip = 0;
        for (let off = 0; off < value.byteLength; off += CHUNK) {
          if (cancelled) return;
          if (sent - acked >= WINDOW) {
            await new Promise<void>((r) => { windowOpen = r; });
            if (cancelled) return;
          }
          const part = value.subarray(off, Math.min(off + CHUNK, value.byteLength));
          ws!.send(part.slice().buffer);
          sent += part.byteLength;
          if (sent % (1024 * 1024) === 0) await new Promise<void>((r) => setTimeout(r, 0));
        }
      }
    } finally {
      reader.cancel().catch(() => {});
    }
  };

  run().catch((e) => finish(e instanceof Error ? e : new Error(String(e))));

  return {
    cancel: () => { cancelled = true; finish(new Error('cancelled')); },
    done,
  };
}
