import { useEffect, useMemo } from 'react';
import { create } from 'zustand';
import { RTCPeerConnection, RTCSessionDescription } from './rtcPeer';
import { robotAuth } from './auth';
import { slamnavBase } from './endpoints';
import { isDemo } from './demoFlag';
import { resolveIceServers } from './remoteIce';
import { installId, installIdReady } from '@/lib/installId';
import { attachPortTunnel } from './robotTunnel';
import { robotFetch } from './rest';
import { simEngine } from './simEngine';
import { useRobot } from '@/store/robot';
import { useSettings } from '@/store/settings';
import {
  CloudAssembler, deriveSlamUi, mapStride, parseCloudFrame, parseMapCloud, parseSlamnavState, utf8,
  type CloudSet, type MapCloud, type SlamnavResults, type SlamnavState, type SlamUiState,
} from './slamnavWire';
import { demoMode, remote, remoteActive } from '@/modules/registry';

const clientId = () => 'rbq-app-slam-' + installId();
const STATE_STALE_MS = 5000;
const FIRST_STATE_MS = 10000;
const LINGER_MS = 2000;

export type SlamLink = 'off' | 'connecting' | 'connected' | 'retrying' | 'unsupported';
export type TopoNode = { id: string; name: string; type: string; info?: string; linked: string[]; pos: [number, number, number]; yaw: number };
export type GlobalPath = { version: number; nodes: string[]; points: [number, number, number][] };
export type SlamMap = MapCloud & { stride: number };

type SlamnavStore = {
  link: SlamLink;
  state?: SlamnavState;
  results: SlamnavResults;
  scan?: CloudSet; obs?: CloudSet; live?: CloudSet;
  map?: SlamMap;
  topo?: TopoNode[];
  path?: GlobalPath;
  lastLog?: { time: string; text: string; color: string };
  lastError?: { path: string; error: string; at: number };
};

const EMPTY = { state: undefined, scan: undefined, obs: undefined, live: undefined, map: undefined,
  topo: undefined, path: undefined, lastLog: undefined, lastError: undefined };

export const useSlamnav = create<SlamnavStore>(() => ({ link: 'off', results: {} }));

export function setSlamResult(r: Partial<SlamnavResults>) {
  useSlamnav.setState((s) => ({ results: { ...s.results, ...r } }));
}

export function useSlamUi(): SlamUiState | undefined {
  const state = useSlamnav((s) => s.state);
  const results = useSlamnav((s) => s.results);
  return useMemo(() => (state ? deriveSlamUi(state, results) : undefined), [state, results]);
}


let users = 0;
let mapUsers = 0;

export function useSlamnavSession(active: boolean, map = false) {
  useEffect(() => {
    if (!active) return;
    users++; if (map) mapUsers++;
    reconcile();
    if (map) client.refreshReads();
    return () => { users--; if (map) mapUsers--; reconcile(); };
  }, [active, map]);
}

let lingerTimer: ReturnType<typeof setTimeout> | null = null;

function wantedTarget(): string | null {
  if (users <= 0) return null;
  if (isDemo()) return 'demo';
  const r = useRobot.getState();
  if (r.conn !== 'connected' || !r.ip) return null;
  return `${r.ip}|${r.visionIp}`;
}

function reconcile() {
  const want = wantedTarget();
  if (want) {
    if (lingerTimer) { clearTimeout(lingerTimer); lingerTimer = null; }
    if (client.target !== want) client.connect(want);
    return;
  }
  if (!client.target || lingerTimer) return;
  const delay = users <= 0 ? LINGER_MS : 0;
  lingerTimer = setTimeout(() => { lingerTimer = null; if (!wantedTarget()) client.disconnect(); }, delay);
}

useRobot.subscribe((s, p) => { if (s.conn !== p.conn || s.ip !== p.ip || s.visionIp !== p.visionIp) reconcile(); });


function waitIce(pc: any): Promise<void> {
  return new Promise((res) => {
    if (pc.iceGatheringState === 'complete') return res();
    const f = () => { if (pc.iceGatheringState === 'complete') { pc.removeEventListener?.('icegatheringstatechange', f); res(); } };
    pc.addEventListener?.('icegatheringstatechange', f);
    setTimeout(res, 2000);
  });
}

const toBuf = (d: unknown): ArrayBuffer | null =>
  d instanceof ArrayBuffer ? d : ArrayBuffer.isView(d) ? (d.buffer.slice(d.byteOffset, d.byteOffset + d.byteLength) as ArrayBuffer) : null;

type Pending = { resolve: (v: any) => void; reject: (e: Error) => void; timer: ReturnType<typeof setTimeout> };

class SlamnavClient {
  target = '';
  private ip = '';
  private pc: any = null;
  private gen = 0;
  private cmdDc: any = null;
  private pending = new Map<string, Pending>();
  private seq = 0;
  private retryMs = 1000;
  private retryTimer: ReturnType<typeof setTimeout> | null = null;
  private wdTimer: ReturnType<typeof setInterval> | null = null;
  private stateAt = 0;
  private stateOpenAt = 0;
  private asm = new CloudAssembler();
  private got = { map: -1, topo: -1, path: -1 };
  private notRegistered = 0;
  private busy = { map: false, topo: false, path: false };

  connect(target: string) {
    this.disconnect();
    this.target = target;
    if (target === 'demo') {
      useSlamnav.setState({ link: 'connected' });
      demoMode.driver?.slam.start({ state: (s) => this.onState(s), cloud: (c) => this.onCloud(c), log: () => {} });
      return;
    }
    this.ip = target.split('|')[0];
    this.open();
  }

  private open() {
    const gen = ++this.gen;
    useSlamnav.setState({ link: this.retryMs > 1000 ? 'retrying' : 'connecting' });
    const net = remoteActive() ? remote.net! : null;
    if (!net) { this.openPc(gen, [], ''); return; }
    (async () => {
      await installIdReady;
      const ticket = await net.pass(clientId()).catch(() => '');
      if (gen !== this.gen) return;
      if (!ticket) { this.retry(gen); return; }
      const ice = await resolveIceServers().catch(() => []);
      if (gen === this.gen) this.openPc(gen, ice, ticket);
    })();
  }

  private openPc(gen: number, iceServers: any[], ticket: string) {
    const ip = this.ip;
    let pc: any;
    try {
      pc = new RTCPeerConnection(iceServers.length ? { iceServers } : undefined);
    } catch { this.retry(gen); return; }
    this.pc = pc;
    const cmd = pc.createDataChannel('command');
    cmd.onmessage = (e: any) => this.onReply(e.data);
    cmd.onopen = () => { if (this.pc === pc) { this.cmdDc = cmd; this.notRegistered = 0; useSlamnav.setState({ link: 'connected' }); this.retryMs = 1000; } };
    cmd.onclose = () => { if (this.cmdDc === cmd) { this.cmdDc = null; this.failPending('SLAMNAV command 채널 닫힘'); } };
    const st = pc.createDataChannel('slam-state', { ordered: false, maxRetransmits: 0 });
    try { st.binaryType = 'arraybuffer'; } catch {}
    st.onopen = () => this.startWatchdog(gen);
    st.onmessage = (e: any) => {
      if (this.pc !== pc) return;
      const s = parseSlamnavState(typeof e.data === 'string' ? e.data : (toBuf(e.data) ?? ''));
      if (s) { this.stateAt = Date.now(); this.onState(s); }
    };
    const cl = pc.createDataChannel('slam-cloud', { ordered: false, maxRetransmits: 0 });
    try { cl.binaryType = 'arraybuffer'; } catch {}
    cl.onmessage = (e: any) => {
      if (this.pc !== pc) return;
      const b = toBuf(e.data);
      const part = b && parseCloudFrame(b);
      const set = part && this.asm.push(part);
      if (set) this.onCloud(set);
    };
    attachPortTunnel(8082, pc.createDataChannel('tunnel'));
    const log = pc.createDataChannel('log');
    log.onmessage = (e: any) => {
      try {
        const m = JSON.parse(typeof e.data === 'string' ? e.data : utf8(new Uint8Array(toBuf(e.data) ?? new ArrayBuffer(0))));
        if (m?.t === 'log') useSlamnav.setState({ lastLog: { time: String(m.time ?? ''), text: String(m.text ?? ''), color: String(m.color ?? '') } });
      } catch {}
    };
    pc.onconnectionstatechange = () => {
      if (this.pc !== pc) return;
      if (pc.connectionState === 'failed' || pc.connectionState === 'closed') this.retry(gen);
    };
    (async () => {
      try {
        await pc.setLocalDescription(await pc.createOffer());
        await waitIce(pc);
        if (gen !== this.gen) return;
        const token = useSettings.getState().webrtcToken ?? '';
        let ans: any;
        if (ticket) {
          try {
            ans = await remote.net!.exchange('slamnav', { clientId: clientId(), sdp: pc.localDescription.sdp, token, pass: ticket });
          } catch (e: any) {
            if (/not registered/.test(String(e?.message ?? '')) && ++this.notRegistered >= 4) {
              if (gen === this.gen) { this.closePc(); useSlamnav.setState({ link: 'unsupported' }); }
              return;
            }
            throw e;
          }
        } else {
          const res = await fetch(`${slamnavBase(ip)}/api/webrtc/offer`, {
            method: 'POST',
            headers: { ...robotAuth(), 'Content-Type': 'application/json' },
            body: JSON.stringify({ sdp: pc.localDescription.sdp, clientId: clientId(), token }),
          });
          ans = await res.json().catch(() => ({}));
          if (!ans?.sdp) throw new Error('offer ' + res.status);
        }
        if (gen !== this.gen || this.pc !== pc || pc.signalingState === 'closed') return;
        await pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: ans.sdp }));
      } catch (e: any) {
        console.log('[slamnav] 연결 실패:', e?.message ?? String(e));
        this.retry(gen);
      }
    })();
  }

  private startWatchdog(gen: number) {
    this.stateAt = 0;
    this.stateOpenAt = Date.now();
    if (this.wdTimer) clearInterval(this.wdTimer);
    this.wdTimer = setInterval(() => {
      if (gen !== this.gen) return;
      const stale = this.stateAt ? Date.now() - this.stateAt > STATE_STALE_MS : Date.now() - this.stateOpenAt > FIRST_STATE_MS;
      if (stale) this.retry(gen);
    }, 1000);
  }

  private retry(gen: number) {
    if (gen !== this.gen) return;
    this.closePc();
    const next = this.gen;
    const wait = this.retryMs;
    this.retryMs = Math.min(this.retryMs * 2, 8000);
    useSlamnav.setState({ link: 'retrying', state: undefined });
    this.retryTimer = setTimeout(() => { this.retryTimer = null; if (next === this.gen) this.open(); }, wait);
  }

  private closePc() {
    this.gen++;
    if (this.wdTimer) { clearInterval(this.wdTimer); this.wdTimer = null; }
    if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
    const old = this.pc;
    this.pc = null;
    this.cmdDc = null;
    this.failPending('SLAMNAV 연결 해제');
    this.asm.reset();
    if (old) {
      try { old.onconnectionstatechange = null; old.close(); } catch {}
      attachPortTunnel(8082, null);
      if (this.ip && !remoteActive()) {
        fetch(`${slamnavBase(this.ip)}/api/webrtc/hangup`, {
          method: 'POST', headers: { ...robotAuth(), 'Content-Type': 'application/json' }, body: JSON.stringify({ clientId: clientId() }),
        }).catch(() => {});
      }
    }
  }

  disconnect() {
    demoMode.driver?.slam.stop();
    this.closePc();
    this.target = '';
    this.retryMs = 1000;
    this.notRegistered = 0;
    this.got = { map: -1, topo: -1, path: -1 };
    useSlamnav.setState({ link: 'off', results: {}, ...EMPTY });
  }


  private onState(s: SlamnavState) {
    useSlamnav.setState({ state: s });
    this.refreshReads();
  }

  private onCloud(c: CloudSet) {
    if (c.kind === 1) useSlamnav.setState({ scan: c });
    else if (c.kind === 2) useSlamnav.setState({ obs: c });
    else useSlamnav.setState({ live: c });
  }

  refreshReads() {
    const s = useSlamnav.getState().state;
    if (!s || !this.target) return;
    if (!s.flags.mapLoaded) {
      if (useSlamnav.getState().map) useSlamnav.setState({ map: undefined });
      this.got.map = -1;
    } else if (mapUsers > 0 && this.got.map !== s.map.version && !this.busy.map) {
      this.fetchMap(s.map.version, s.map.points);
    }
    if (this.got.topo !== s.map.topoVersion && !this.busy.topo) this.fetchTopo(s.map.topoVersion);
    if (this.got.path !== s.map.pathVersion && !this.busy.path) this.fetchPath(s.map.pathVersion);
  }

  private async getJson(path: string): Promise<any> {
    const r = await robotFetch(`${slamnavBase(this.ip)}${path}`, { headers: robotAuth() });
    if (!r.ok) throw new Error(`${path} ${r.status}`);
    return r.json();
  }

  private async fetchMap(version: number, points: number) {
    const target = this.target;
    const stride = mapStride(points);
    this.busy.map = true;
    try {
      let m: MapCloud | null;
      if (target === 'demo') m = demoMode.driver?.slam.mapCloud(stride) ?? null;
      else {
        const r = await robotFetch(`${slamnavBase(this.ip)}/api/map/cloud?stride=${stride}`, { headers: robotAuth() });
        m = r.ok ? parseMapCloud(await r.arrayBuffer()) : null;
      }
      if (this.target !== target) return;
      this.got.map = version;
      if (m) useSlamnav.setState({ map: { ...m, stride } });
    } catch (e: any) {
      console.log('[slamnav] map/cloud 실패:', e?.message ?? String(e));
    } finally { this.busy.map = false; }
  }

  private async fetchTopo(version: number) {
    const target = this.target;
    this.busy.topo = true;
    try {
      const j = target === 'demo' ? demoMode.driver?.slam.topoJson() : await this.getJson('/api/map/topo');
      if (this.target !== target) return;
      this.got.topo = version;
      const nodes: TopoNode[] = (Array.isArray(j?.nodes) ? j.nodes : []).map((n: any) => ({
        id: String(n.id), name: String(n.name ?? ''), type: String(n.type ?? 'ROUTE'), info: n.info,
        linked: Array.isArray(n.linked) ? n.linked.map(String) : [],
        pos: Array.isArray(n.pos) ? [Number(n.pos[0]) || 0, Number(n.pos[1]) || 0, Number(n.pos[2]) || 0] : [0, 0, 0],
        yaw: Number(n.yaw) || 0,
      }));
      useSlamnav.setState({ topo: nodes });
    } catch (e: any) {
      console.log('[slamnav] map/topo 실패:', e?.message ?? String(e));
    } finally { this.busy.topo = false; }
  }

  private async fetchPath(version: number) {
    const target = this.target;
    this.busy.path = true;
    try {
      const j = target === 'demo' ? demoMode.driver?.slam.pathJson() : await this.getJson('/api/path/global');
      if (this.target !== target) return;
      this.got.path = version;
      const points = (Array.isArray(j?.points) ? j.points : [])
        .filter((p: unknown) => Array.isArray(p))
        .map((p: number[]) => [Number(p[0]) || 0, Number(p[1]) || 0, Number(p[2]) || 0] as [number, number, number]);
      useSlamnav.setState({ path: { version, nodes: Array.isArray(j?.nodes) ? j.nodes.map(String) : [], points } });
    } catch (e: any) {
      console.log('[slamnav] path/global 실패:', e?.message ?? String(e));
    } finally { this.busy.path = false; }
  }


  command(method: 'GET' | 'POST', path: string, body: Record<string, unknown> = {}): Promise<any> {
    const demo = this.target === 'demo' ? demoMode.driver : undefined;
    if (demo) {
      const r = demo.slam.command(path, body);
      return r.status < 300 ? Promise.resolve(r.body) : Promise.reject(new Error(String(r.body.error ?? r.status)));
    }
    if (simEngine.active) return Promise.reject(new Error('물리 시뮬 중에는 로봇으로 보내지 않는다'));
    const dc = this.cmdDc;
    if (!dc || dc.readyState !== 'open') return Promise.reject(new Error('SLAMNAV 미연결'));
    return new Promise((resolve, reject) => {
      const id = 'sl-' + (++this.seq) + '-' + Math.random().toString(16).slice(2, 6);
      const timer = setTimeout(() => { if (this.pending.delete(id)) reject(new Error('SLAMNAV 응답 없음')); }, 8000);
      this.pending.set(id, { resolve, reject, timer });
      try { dc.send(JSON.stringify({ id, method, path, body: method === 'POST' ? { ...body, msg_id: id } : body })); }
      catch (e: any) { clearTimeout(timer); this.pending.delete(id); reject(e instanceof Error ? e : new Error(String(e))); }
    });
  }

  private onReply(data: unknown) {
    let env: any;
    try { env = JSON.parse(typeof data === 'string' ? data : utf8(new Uint8Array(toBuf(data) ?? new ArrayBuffer(0)))); } catch { return; }
    const p = env && this.pending.get(env.id);
    if (!p) return;
    clearTimeout(p.timer); this.pending.delete(env.id);
    const status = Number(env.status ?? 0);
    if (status >= 200 && status < 300) p.resolve(env.body ?? {});
    else p.reject(new Error(String(env.body?.error ?? `status ${status}`)));
  }

  private failPending(reason: string) {
    for (const [, p] of this.pending) { clearTimeout(p.timer); p.reject(new Error(reason)); }
    this.pending.clear();
  }
}

const client = new SlamnavClient();

export async function slamCmd(path: string, body: Record<string, unknown> = {}, onOk?: () => void): Promise<boolean> {
  try {
    await client.command('POST', path, body);
    onOk?.();
    if (useSlamnav.getState().lastError) useSlamnav.setState({ lastError: undefined });
    return true;
  } catch (e: any) {
    useSlamnav.setState({ lastError: { path, error: String(e?.message ?? e), at: Date.now() } });
    return false;
  }
}
