
export class RemoteUnsupportedError extends Error {
  constructor(path: string) {
    super(`${path} — 원격 연결에서는 사용할 수 없습니다`);
    this.name = 'RemoteUnsupportedError';
  }
}

export function viaRemote(): boolean {
  try { return require('@/store/robot').useRobot.getState().via === 'remote'; }
  catch { return false; }
}

type Dc = {
  readyState: string;
  binaryType?: string;
  send: (d: string | ArrayBuffer) => void;
  onmessage: ((e: { data: any }) => void) | null;
  onclose?: (() => void) | null;
};

type Stream = {
  opened: boolean;
  onData: (b: Uint8Array) => void;
  onClose: (err?: string) => void;
};

const OPEN_TIMEOUT_MS = 10_000;
const CHUNK = 16 * 1024;

class TunnelLink {
  dc: Dc | null = null;
  hello = false;
  nextId = 1;
  streams = new Map<number, Stream>();

  attach(ch: Dc | null, onRx?: () => void) {
    for (const p of this.streams.values()) p.onClose('연결이 끊겼습니다');
    this.streams.clear();
    this.hello = false;
    this.dc = ch;
    if (!ch) return;
    try { ch.binaryType = 'arraybuffer'; } catch {}
    ch.onmessage = (e) => { onRx?.(); this.onMessage(e.data); };
  }

  ready() { return this.hello && this.dc?.readyState === 'open'; }

  private onMessage(data: any) {
    if (typeof data === 'string') {
      let m: any;
      try { m = JSON.parse(data); } catch { return; }
      if (m.t === 'hello') { this.hello = true; return; }
      const p = this.streams.get(m.id);
      if (!p) return;
      if (m.t === 'opened') p.opened = true;
      else if (m.t === 'close') { this.streams.delete(m.id); p.onClose(m.err); }
      return;
    }
    const buf = new Uint8Array(data as ArrayBuffer);
    if (buf.length < 4) return;
    const id = ((buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3]) >>> 0;
    this.streams.get(id)?.onData(buf.slice(4));
  }

  private sendBytes(id: number, bytes: Uint8Array) {
    for (let off = 0; off < bytes.length || off === 0; off += CHUNK) {
      const part = bytes.subarray(off, off + CHUNK);
      const out = new Uint8Array(4 + part.length);
      out[0] = id >>> 24; out[1] = (id >>> 16) & 255; out[2] = (id >>> 8) & 255; out[3] = id & 255;
      out.set(part, 4);
      this.dc!.send(out.buffer);
      if (bytes.length === 0) break;
    }
  }

  open(port: number, onData: (b: Uint8Array) => void, onClose: (err?: string) => void) {
    if (!this.ready()) throw new Error('tunnel not ready');
    const id = this.nextId++ >>> 0;
    let closed = false;
    const st: Stream = {
      opened: false,
      onData,
      onClose: (err) => { if (closed) return; closed = true; clearTimeout(timer); onClose(err); },
    };
    const timer = setTimeout(() => { if (!st.opened) { this.streams.delete(id); st.onClose('로봇 포트에 연결하지 못했습니다'); } }, OPEN_TIMEOUT_MS);
    this.streams.set(id, st);
    this.dc!.send(JSON.stringify({ t: 'open', id, port }));
    return {
      write: (bytes: Uint8Array) => { if (!closed && this.dc?.readyState === 'open') this.sendBytes(id, bytes); },
      close: () => {
        if (closed) return;
        try { this.dc?.send(JSON.stringify({ t: 'close', id })); } catch {}
        this.streams.delete(id);
        st.onClose('closed');
      },
    };
  }
}

const motion = new TunnelLink();
const byPort = new Map<number, TunnelLink>();

export function attachTunnel(ch: Dc | null, onRx?: () => void) { motion.attach(ch, onRx); }

export function attachPortTunnel(port: number, ch: Dc | null) {
  let l = byPort.get(port);
  if (!l) { l = new TunnelLink(); byPort.set(port, l); }
  l.attach(ch);
}

export const tunnelReady = () => motion.ready();

function pick(port: number): TunnelLink {
  const l = byPort.get(port);
  return l?.ready() ? l : motion;
}

export function openStream(port: number, onData: (b: Uint8Array) => void, onClose: (err?: string) => void) {
  return pick(port).open(port, onData, onClose);
}

export const STREAM_IDLE_MS = 60_000;
const STALLED = '로봇이 응답하지 않습니다 — 연결을 확인하고 다시 시도하세요';

function idleGuard(onStall: () => void) {
  let t = setTimeout(onStall, STREAM_IDLE_MS);
  return { kick: () => { clearTimeout(t); t = setTimeout(onStall, STREAM_IDLE_MS); }, stop: () => clearTimeout(t) };
}

function roundTrip(port: number, request: Uint8Array, signal?: AbortSignal | null): Promise<Uint8Array> {
  return new Promise((resolve, reject) => {
    const chunks: Uint8Array[] = [];
    let stalled = false;
    const onAbort = () => s.close();
    const idle = idleGuard(() => { stalled = true; s.close(); });
    const s = openStream(port, (b) => { idle.kick(); chunks.push(b); }, (err) => {
      idle.stop();
      signal?.removeEventListener?.('abort', onAbort);
      if (stalled) { reject(new Error(STALLED)); return; }
      if (err) { reject(err === 'closed' ? new DOMException('Aborted', 'AbortError') : new Error(err)); return; }
      resolve(concat(chunks));
    });
    signal?.addEventListener?.('abort', onAbort);
    s.write(request);
  });
}

function concat(chunks: Uint8Array[]): Uint8Array {
  let n = 0;
  for (const c of chunks) n += c.length;
  const all = new Uint8Array(n);
  let o = 0;
  for (const c of chunks) { all.set(c, o); o += c.length; }
  return all;
}

const enc = new TextEncoder();

async function bodyBytes(body: any): Promise<Uint8Array> {
  if (body == null) return new Uint8Array(0);
  if (typeof body === 'string') return enc.encode(body);
  if (body instanceof Uint8Array) return body;
  if (body instanceof ArrayBuffer) return new Uint8Array(body);
  if (ArrayBuffer.isView(body)) return new Uint8Array(body.buffer, body.byteOffset, body.byteLength);
  throw new Error('원격 요청 본문 형식을 지원하지 않습니다');
}

export function parseHttp(raw: Uint8Array): Response {
  let i = 0;
  while (i + 3 < raw.length && !(raw[i] === 13 && raw[i + 1] === 10 && raw[i + 2] === 13 && raw[i + 3] === 10)) i++;
  if (i + 3 >= raw.length) throw new Error('로봇 응답이 잘렸습니다');
  const head = new TextDecoder().decode(raw.subarray(0, i)).split('\r\n');
  const status = Number(head[0].split(' ')[1]);
  const headers = new Headers();
  for (const line of head.slice(1)) {
    const c = line.indexOf(':');
    if (c > 0) headers.append(line.slice(0, c).trim(), line.slice(c + 1).trim());
  }
  const body = raw.slice(i + 4);
  const declared = Number(headers.get('content-length') ?? NaN);
  if (Number.isFinite(declared) && body.length < declared) throw new Error('로봇 응답이 잘렸습니다 — 다시 시도하세요');
  return new Response(status === 204 || status === 304 ? null : body, { status, headers });
}

async function buildRequest(url: string, init: RequestInit) {
  const { port, path } = targetOf(url);
  const body = await bodyBytes(init.body);
  const h = new Headers(init.headers as any);
  let head = `${(init.method ?? 'GET').toUpperCase()} ${path} HTTP/1.1\r\nHost: 127.0.0.1:${port}\r\nConnection: close\r\n`;
  h.forEach((v, k) => { if (k !== 'host' && k !== 'connection' && k !== 'content-length') head += `${k}: ${v}\r\n`; });
  if (body.length || init.body != null) head += `Content-Length: ${body.length}\r\n`;
  const headBytes = enc.encode(head + '\r\n');
  const req = new Uint8Array(headBytes.length + body.length);
  req.set(headBytes);
  req.set(body, headBytes.length);
  return { port, req };
}

export async function tunnelFetch(url: string, init: RequestInit = {}): Promise<Response> {
  const { port, req } = await buildRequest(url, init);
  return parseHttp(await roundTrip(port, req, init.signal));
}

export async function tunnelDownload(url: string, init: RequestInit, write: (b: Uint8Array) => void): Promise<{ status: number; headers: Headers }> {
  const { port, req } = await buildRequest(url, init);
  const signal = init.signal;
  return new Promise((resolve, reject) => {
    let head: Uint8Array | null = new Uint8Array(0);
    let meta: { status: number; headers: Headers } | null = null;
    let failed: unknown = null;
    let got = 0;
    const onAbort = () => { failed = new DOMException('Aborted', 'AbortError'); s.close(); };
    const idle = idleGuard(() => { failed = new Error(STALLED); s.close(); });
    const body = (b: Uint8Array) => { got += b.length; write(b); };
    const s = openStream(port, (b) => {
      if (failed) return;
      idle.kick();
      try {
        if (meta) { body(b); return; }
        head = concat([head!, b]);
        const i = indexOfCrlf2(head);
        if (i < 0) return;
        const h = headOnly(head.slice(0, i + 4));
        meta = { status: h.status, headers: h.headers };
        const rest = head.subarray(i + 4);
        head = null;
        if (rest.length) body(rest);
      } catch (e) { failed = e; s.close(); }
    }, (err) => {
      idle.stop();
      signal?.removeEventListener?.('abort', onAbort);
      if (failed) { reject(failed); return; }
      if (err) { reject(new Error(err)); return; }
      if (!meta) { reject(new Error('로봇 응답이 잘렸습니다')); return; }
      const declared = Number(meta.headers.get('content-length') ?? NaN);
      if (Number.isFinite(declared) && got !== declared) {
        reject(new Error(`로봇 응답이 잘렸습니다 (${got} / ${declared} B) — 다시 받으세요`));
        return;
      }
      resolve(meta);
    });
    signal?.addEventListener?.('abort', onAbort);
    s.write(req);
  });
}

function headOnly(block: Uint8Array): Response {
  const lines = new TextDecoder().decode(block).split('\r\n');
  const status = Number(lines[0].split(' ')[1]);
  const headers = new Headers();
  for (const line of lines.slice(1)) {
    const c = line.indexOf(':');
    if (c > 0) headers.append(line.slice(0, c).trim(), line.slice(c + 1).trim());
  }
  return new Response(null, { status: status >= 200 && status <= 599 ? status : 502, headers });
}

function indexOfCrlf2(b: Uint8Array): number {
  for (let i = 0; i + 3 < b.length; i++) if (b[i] === 13 && b[i + 1] === 10 && b[i + 2] === 13 && b[i + 3] === 10) return i;
  return -1;
}

export function targetOf(url: string): { port: number; path: string } {
  const m = /^https?:\/\/[^/]+:(\d+)(\/.*)?$/.exec(url);
  if (m) return { port: Number(m[1]), path: m[2] || '/' };
  if (url.startsWith('/streamer/')) return { port: 8081, path: url.slice('/streamer'.length) };
  if (url.startsWith('/slamnav/')) return { port: 8082, path: url.slice('/slamnav'.length) };
  return { port: 8080, path: url.startsWith('/') ? url : '/' + url };
}

export class TunnelWebSocket {
  binaryType = 'arraybuffer';
  readyState = 0;
  onopen: (() => void) | null = null;
  onmessage: ((e: { data: string | ArrayBuffer }) => void) | null = null;
  onerror: ((e: unknown) => void) | null = null;
  onclose: (() => void) | null = null;
  private s: ReturnType<typeof openStream> | null = null;
  private buf: Uint8Array = new Uint8Array(0);
  private upgraded = false;
  private frag: Uint8Array[] = [];
  private fragOp = 0;

  constructor(port: number, path = '/') {
    try {
      this.s = openStream(port, (b) => this.onBytes(b), (err) => this.end(err));
    } catch (e) { setTimeout(() => this.end(String(e)), 0); return; }
    const key = btoa(String.fromCharCode(...Array.from({ length: 16 }, () => (Math.random() * 256) | 0)));
    this.s.write(enc.encode(`GET ${path} HTTP/1.1\r\nHost: 127.0.0.1:${port}\r\nUpgrade: websocket\r\nConnection: Upgrade\r\nSec-WebSocket-Key: ${key}\r\nSec-WebSocket-Version: 13\r\n\r\n`));
  }

  send(data: string | ArrayBuffer | Uint8Array) {
    if (this.readyState !== 1) throw new Error('WebSocket is not open');
    const text = typeof data === 'string';
    const payload = text ? enc.encode(data as string) : data instanceof Uint8Array ? data : new Uint8Array(data as ArrayBuffer);
    this.frame(text ? 1 : 2, payload);
  }

  close() {
    if (this.readyState >= 2) return;
    if (this.readyState === 1) { try { this.frame(8, new Uint8Array(0)); } catch {} }
    this.readyState = 2;
    this.s?.close();
  }

  private frame(op: number, payload: Uint8Array) {
    const n = payload.length;
    const hl = n < 126 ? 2 : n < 65536 ? 4 : 10;
    const pad = (4 - ((hl + 4) & 3)) & 3;
    const f = new Uint8Array(new ArrayBuffer(pad + hl + 4 + n), pad, hl + 4 + n);
    f[0] = 0x80 | op;
    if (n < 126) f[1] = 0x80 | n;
    else if (n < 65536) { f[1] = 0x80 | 126; f[2] = n >> 8; f[3] = n & 255; }
    else { f[1] = 0x80 | 127; for (let i = 0; i < 8; i++) f[2 + i] = i < 4 ? 0 : (n / 2 ** (8 * (7 - i))) & 255; }
    const mask = Array.from({ length: 4 }, () => (Math.random() * 256) | 0);
    f.set(mask, hl);
    const body = f.subarray(hl + 4);
    body.set(payload);
    const words = n >> 2;
    if (words) {
      const m32 = new Uint32Array(new Uint8Array(mask).buffer)[0];
      const w = new Uint32Array(body.buffer, body.byteOffset, words);
      for (let i = 0; i < words; i++) w[i] ^= m32;
    }
    for (let i = words << 2; i < n; i++) body[i] ^= mask[i & 3];
    this.s!.write(f);
  }

  private onBytes(b: Uint8Array) {
    this.buf = this.buf.length ? concat([this.buf, b]) : b;
    if (!this.upgraded) {
      let i = 0;
      while (i + 3 < this.buf.length && !(this.buf[i] === 13 && this.buf[i + 1] === 10 && this.buf[i + 2] === 13 && this.buf[i + 3] === 10)) i++;
      if (i + 3 >= this.buf.length) return;
      const status = new TextDecoder().decode(this.buf.subarray(0, i)).split(' ')[1];
      this.buf = this.buf.slice(i + 4);
      if (status !== '101') { this.onerror?.(new Error(`HTTP ${status}`)); this.close(); return; }
      this.upgraded = true;
      this.readyState = 1;
      this.onopen?.();
    }
    for (;;) {
      const B = this.buf;
      if (B.length < 2) return;
      let n = B[1] & 0x7f;
      let o = 2;
      if (n === 126) { if (B.length < 4) return; n = (B[2] << 8) | B[3]; o = 4; }
      else if (n === 127) { if (B.length < 10) return; n = 0; for (let i = 2; i < 10; i++) n = n * 256 + B[i]; o = 10; }
      if (B[1] & 0x80) o += 4;
      if (B.length < o + n) return;
      const op = B[0] & 0x0f;
      const fin = (B[0] & 0x80) !== 0;
      const payload = B.slice(o, o + n);
      this.buf = B.slice(o + n);
      if (op === 8) { this.close(); return; }
      if (op === 9) { this.frame(10, payload); continue; }
      if (op === 10) continue;
      if (op !== 0) this.fragOp = op;
      this.frag.push(payload);
      if (!fin) continue;
      const msg = concat(this.frag);
      this.frag = [];
      this.onmessage?.({ data: this.fragOp === 1 ? new TextDecoder().decode(msg) : (msg.buffer as ArrayBuffer) });
    }
  }

  private end(err?: string) {
    const was = this.readyState;
    this.readyState = 3;
    if (err && err !== 'closed' && was < 2) this.onerror?.(new Error(err));
    this.onclose?.();
  }
}
