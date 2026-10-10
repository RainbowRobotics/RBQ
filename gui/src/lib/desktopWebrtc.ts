
const CH = ['log', 'motion-state', 'command', 'estop'] as const;
const TEXT_CH = new Set<string>(['log', 'command']);
type ChLabel = string;

type PcState = 'new' | 'connecting' | 'connected' | 'disconnected' | 'failed' | 'closed';

class BridgeChannel {
  readyState: 'connecting' | 'open' | 'closing' | 'closed' = 'connecting';
  binaryType = 'arraybuffer';
  onopen: (() => void) | null = null;
  onmessage: ((e: { data: unknown }) => void) | null = null;
  onclose: (() => void) | null = null;
  constructor(
    private readonly bridge: DesktopWebrtcBridge,
    private readonly id: number,
    private readonly label: ChLabel,
  ) {}
  send(data: string | ArrayBuffer | Uint8Array) {
    this.bridge._send(this.id, data);
  }
}

export class DesktopWebrtcBridge {
  connectionState: PcState = 'new';
  onconnectionstatechange: (() => void) | null = null;
  private ws: WebSocket | null = null;
  private readonly channels = new Map<ChLabel, BridgeChannel>();
  private readonly labels: string[];
  private readonly ext: string[];

  constructor(ext: string[] = []) {
    this.ext = ext;
    this.labels = [...CH, ...ext];
    this.labels.forEach((label, id) => this.channels.set(label, new BridgeChannel(this, id, label)));
  }

  createDataChannel(label: string): BridgeChannel {
    const ch = this.channels.get(label as ChLabel);
    if (!ch) throw new Error('알 수 없는 채널: ' + label);
    return ch;
  }

  connect() {
    this.setState('connecting');
    const proto = typeof location !== 'undefined' && location.protocol === 'https:' ? 'wss://' : 'ws://';
    const host = typeof location !== 'undefined' ? location.host : '127.0.0.1:8090';
    const ws = new WebSocket(`${proto}${host}/webrtc${this.ext.length ? `?ext=${encodeURIComponent(this.ext.join(','))}` : ''}`);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;
    ws.onmessage = (e) => this.onFrame(e.data);
    ws.onclose = () => this.setState('closed');
    ws.onerror = () => this.setState('failed');
  }

  close() {
    const ws = this.ws;
    this.ws = null;
    if (ws) { ws.onclose = null; ws.onerror = null; ws.onmessage = null; try { ws.close(); } catch { } }
    for (const ch of this.channels.values()) ch.readyState = 'closed';
    this.connectionState = 'closed';
  }

  _send(id: number, data: string | ArrayBuffer | Uint8Array) {
    if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
    const payload =
      typeof data === 'string' ? new TextEncoder().encode(data)
      : data instanceof Uint8Array ? data
      : new Uint8Array(data);
    const frame = new Uint8Array(1 + payload.length);
    frame[0] = id;
    frame.set(payload, 1);
    this.ws.send(frame);
  }

  private onFrame(data: unknown) {
    const buf = new Uint8Array(data instanceof ArrayBuffer ? data : (data as ArrayBufferView).buffer ?? new ArrayBuffer(0));
    if (buf.length < 1) return;
    const id = buf[0];
    if (id === 0xff) {
      let ctrl: { t: string; ch?: string; state?: string };
      try { ctrl = JSON.parse(new TextDecoder().decode(buf.subarray(1))); } catch { return; }
      if (ctrl.t === 'open') { const c = this.channels.get(ctrl.ch as ChLabel); if (c) { c.readyState = 'open'; c.onopen?.(); } }
      else if (ctrl.t === 'close') { const c = this.channels.get(ctrl.ch as ChLabel); if (c) { c.readyState = 'closed'; c.onclose?.(); } }
      else if (ctrl.t === 'pc') { this.setState((ctrl.state as PcState) ?? this.connectionState); }
      else if (ctrl.t === 'error') { this.setState('failed'); }
      return;
    }
    const label = this.labels[id];
    const ch = label && this.channels.get(label);
    if (!ch || !ch.onmessage) return;
    const payload = buf.subarray(1);
    const asText = TEXT_CH.has(label) || this.ext.includes(label);
    ch.onmessage({ data: asText ? new TextDecoder().decode(payload) : payload.slice().buffer });
  }

  private setState(s: PcState) {
    if (this.connectionState === s) return;
    this.connectionState = s;
    this.onconnectionstatechange?.();
  }
}
