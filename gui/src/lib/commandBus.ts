export type CommandSender = (method: string, path: string, body?: object) => Promise<any>;

let sender: CommandSender | null = null;

export function setCommandSender(s: CommandSender) { sender = s; }

export function dcCommand(method: string, path: string, body?: object): Promise<any> {
  if (!sender) return Promise.reject(new Error(`${path} — 연결 전(command 채널 미등록)`));
  return sender(method, path, body);
}

let streamerSender: CommandSender | null = null;
let streamerWaiters: (() => void)[] = [];
const streamerReadyListeners = new Set<() => void>();
export function setStreamerCommandSender(s: CommandSender | null) {
  const was = streamerSender;
  streamerSender = s;
  if (s) { const w = streamerWaiters; streamerWaiters = []; w.forEach((f) => f()); }
  if (s && s !== was) streamerReadyListeners.forEach((fn) => { try { fn(); } catch { } });
}
export function onStreamerCommandReady(fn: () => void): () => void {
  streamerReadyListeners.add(fn);
  return () => { streamerReadyListeners.delete(fn); };
}
export function whenStreamerReady(timeoutMs: number): Promise<boolean> {
  if (streamerSender) return Promise.resolve(true);
  return new Promise((resolve) => {
    const done = () => { clearTimeout(timer); resolve(true); };
    const timer = setTimeout(() => {
      streamerWaiters = streamerWaiters.filter((f) => f !== done);
      resolve(false);
    }, timeoutMs);
    streamerWaiters.push(done);
  });
}
export function streamerCommand(method: string, path: string, body?: object): Promise<any> {
  if (!streamerSender) return Promise.reject(new Error(`${path} — 스트리머 연결 전(command 채널 미등록)`));
  return streamerSender(method, path, body);
}

let visionRequestSender: ((bytes: Uint8Array) => void) | null = null;
export function setVisionRequestSender(s: ((bytes: Uint8Array) => void) | null) { visionRequestSender = s; }
export const visionDcActive = () => visionRequestSender != null;
export function visionRequestViaDc(bytes: Uint8Array): boolean {
  if (!visionRequestSender) return false;
  try { visionRequestSender(bytes); return true; } catch { return false; }
}
