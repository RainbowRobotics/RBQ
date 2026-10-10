
export function isDesktop(): boolean {
  return typeof window !== 'undefined' && '__TAURI_INTERNALS__' in window;
}

let restartProxySeq = 0;

export type RendezvousArgs = { rendezvousUrl?: string; robotId?: string; webrtcToken?: string };

export async function restartProxy(ip: string, visionIp?: string, rv?: RendezvousArgs): Promise<void> {
  if (!isDesktop()) return;
  const seq = ++restartProxySeq;
  const { emit, listen } = await import('@tauri-apps/api/event');
  if (seq !== restartProxySeq) return;
  let resolveReady: () => void;
  const ready = new Promise<void>((r) => {
    resolveReady = r;
  });
  const unlisten = await listen<{ running?: boolean }>('proxy-status', (e) => {
    if (e.payload?.running) resolveReady();
  });
  try {
    if (seq !== restartProxySeq) return;
    await emit('restart-proxy', {
      ip, visionIp: (visionIp ?? '').trim(),
      rendezvousUrl: (rv?.rendezvousUrl ?? '').trim(),
      robotId: (rv?.robotId ?? '').trim(),
      webrtcToken: (rv?.webrtcToken ?? '').trim(),
    });
    await Promise.race([ready, new Promise<void>((r) => setTimeout(r, 30000))]);
  } finally {
    unlisten();
  }
}


export type WifiNetwork = { ssid: string; signal: number; secured: boolean; saved?: boolean; active: boolean; ip?: string };

type EventTransport = {
  listen: <T>(event: string, cb: (payload: T) => void) => Promise<() => void>;
  emit: (event: string, payload: unknown) => Promise<void>;
};

type WifiResult<T> = { reqId: string; ok: boolean; data?: T; error?: string };

const SESSION_TAG = Date.now().toString(36);
let reqSeq = 0;
function nextReqId(): string {
  return `wifi-${SESSION_TAG}-${++reqSeq}`;
}

export async function requestOnce<T>(
  tx: EventTransport,
  reqEvent: string,
  resultEvent: string,
  payload: Record<string, unknown>,
  timeoutMs: number,
): Promise<T> {
  const reqId = nextReqId();
  let settle!: { resolve: (v: T) => void; reject: (e: Error) => void };
  const result = new Promise<T>((resolve, reject) => {
    settle = { resolve, reject };
  });
  const unlisten = await tx.listen<WifiResult<T>>(resultEvent, (p) => {
    if (!p || p.reqId !== reqId) return;
    if (p.ok) settle.resolve(p.data as T);
    else settle.reject(new Error(p.error || 'WiFi 명령 실패'));
  });
  const timer = setTimeout(() => settle.reject(new Error('WiFi 응답 없음 — 다시 시도하세요')), timeoutMs);
  try {
    await tx.emit(reqEvent, { ...payload, reqId });
    return await result;
  } finally {
    clearTimeout(timer);
    unlisten();
  }
}

export function sortAndDedup(nets: WifiNetwork[]): WifiNetwork[] {
  const best = new Map<string, WifiNetwork>();
  for (const n of nets) {
    const prev = best.get(n.ssid);
    const active = n.active || (prev?.active ?? false);
    if (!prev || n.signal > prev.signal) best.set(n.ssid, { ...n, active });
    else if (active !== prev.active) best.set(n.ssid, { ...prev, active });
  }
  return [...best.values()].sort((a, b) => b.signal - a.signal);
}

async function eventTransport(): Promise<EventTransport> {
  const { emit, listen } = await import('@tauri-apps/api/event');
  return {
    listen: (event, cb) => listen(event, (e: { payload: unknown }) => cb(e.payload as never)),
    emit: (event, payload) => emit(event, payload),
  };
}

export async function setUiZoom(scale: number | null): Promise<void> {
  if (!isDesktop()) return;
  const tx = await eventTransport();
  await tx.emit('ui-zoom', { scale });
}

export async function quitApp(): Promise<void> {
  if (!isDesktop()) return;
  const tx = await eventTransport();
  await tx.emit('app-quit', {});
}

export async function wifiScan(): Promise<WifiNetwork[]> {
  if (!isDesktop()) return [];
  const tx = await eventTransport();
  const raw = await requestOnce<WifiNetwork[]>(tx, 'wifi-scan', 'wifi-scan-result', {}, 25000);
  return sortAndDedup(raw);
}

export async function wifiConnect(ssid: string, password?: string): Promise<void> {
  if (!isDesktop()) throw new Error('데스크탑에서만 WiFi 연결을 지원합니다');
  const tx = await eventTransport();
  await requestOnce<null>(tx, 'wifi-connect', 'wifi-connect-result', { ssid, password: password ?? null }, 40000);
}

export async function wifiCurrent(): Promise<WifiNetwork | null> {
  if (!isDesktop()) return null;
  const tx = await eventTransport();
  return await requestOnce<WifiNetwork | null>(tx, 'wifi-current', 'wifi-current-result', {}, 5000);
}

export async function toggleFullscreen(): Promise<void> {
  if (!isDesktop()) return;
  const tx = await eventTransport();
  await tx.emit('toggle-fullscreen', {});
}

export function installFullscreenHotkey(): () => void {
  if (!isDesktop() || typeof window === 'undefined') return () => {};
  const onKey = (e: KeyboardEvent) => {
    if (e.key !== 'F11') return;
    e.preventDefault();
    void toggleFullscreen();
  };
  window.addEventListener('keydown', onKey);
  return () => window.removeEventListener('keydown', onKey);
}
