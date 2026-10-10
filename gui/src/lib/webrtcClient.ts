import { PermissionsAndroid, Platform } from 'react-native';
import { create } from 'zustand';
import { isDemo } from '@/lib/demoFlag';
// @ts-ignore
import { RTCPeerConnection, RTCSessionDescription, mediaDevices } from 'react-native-webrtc';
import * as Device from 'expo-device';
import { robotAuth } from '@/lib/auth';
import { streamerBase } from '@/lib/endpoints';
import { useSettings } from '@/store/settings';
import { useRobot } from '@/store/robot';
import { waitRoleKnown } from '@/lib/spectating';
import { attachPointcloudChannel, resetAllClouds } from '@/lib/pointcloud';
import { attachElevationChannel } from '@/lib/elevationMap';
import { attachWallmapChannel } from '@/lib/wallMap';
import { setStreamerCommandSender, setVisionRequestSender } from '@/lib/commandBus';
import { handleVisionStateFrame } from '@/lib/visionStateDc';
import { noteVisionLink } from '@/store/telemetry';
import { modeField, rendezvousEnabled, rendezvousUrl, robotId, resolveIceServers, routeKey } from '@/lib/remoteIce';
import { rendezvousExchange } from '@/lib/rendezvousClient';
import { currentTicket } from '@/lib/connectTicket';
import { rest } from './rest';
import { inboundVideoFromStats, routeDetail, readStats } from '@/lib/connectionRoute';
import { setKeyframeProbe } from '@/lib/videoKeyframe';
import { iosAudioSession } from '@/lib/iosAudioSession';
import { simEngine } from './simEngine';
import { armVideoStaleClear, cancelVideoStaleClear } from './videoStale';

const CLIENT_ID = 'rbq-app-' + Math.random().toString(16).slice(2, 10);
const VISION_STALE_MS = 3000;
const VISION_WATCHDOG_TICK_MS = 1000;
const ROBOT_CLOSED_RETRY_MS = 250;

const IS_IOS_SIM = Platform.OS === 'ios' && !Device.isDevice;

type WebrtcState = { url: string | null; stream?: unknown; imgMode?: boolean; status: string; retryS?: number };
export const useWebrtcStore = create<WebrtcState>(() => ({ url: null, imgMode: false, status: '미연결' }));
export const useWebrtcStream = () => useWebrtcStore();

function stripTransportCc(sdp: string): string {
  return sdp
    .split('\r\n')
    .filter((l) => !/^a=rtcp-fb:\d+ transport-cc/.test(l)
      && !(l.startsWith('a=extmap:') && l.includes('transport-wide-cc')))
    .join('\r\n');
}

function waitIce(pc: any): Promise<void> {
  return new Promise((res) => {
    if (pc.iceGatheringState === 'complete') return res();
    const f = () => {
      if (pc.iceGatheringState === 'complete') {
        pc.removeEventListener?.('icegatheringstatechange', f);
        res();
      }
    };
    pc.addEventListener?.('icegatheringstatechange', f);
    setTimeout(res, 3000);
  });
}

class WebrtcClient {
  private diagTimer: any = null;
  private lastBytes = 0;

  private startVideoDiag(pc: any) {
    this.stopVideoDiag();
    if (typeof pc?.getStats !== 'function') return;
    this.diagTimer = setInterval(() => {
      if (this.pc !== pc) { this.stopVideoDiag(); return; }
      pc.getStats().then((r: any) => {
        const arr: any[] = [];
        if (typeof r?.forEach === 'function') r.forEach((v: any) => arr.push(v));
        const v = inboundVideoFromStats(arr);
        const d = routeDetail(arr);
        if (v) {
          const mbps = this.lastBytes > 0 ? ((v.bytesReceived - this.lastBytes) * 8 / 5 / 1e6).toFixed(2) : '?';
          this.lastBytes = v.bytesReceived;
          console.log(`[video] ${d.route}${d.proto ? '(' + d.proto + ')' : ''} ` +
            `${v.frameWidth}x${v.frameHeight} ${v.framesPerSecond}fps ${mbps}Mbps ` +
            `jbuf=${v.jitterBufferMs}ms nack=${v.nackCount} ` +
            `decoded=${v.framesDecoded} dropped=${v.framesDropped} lost=${v.packetsLost}(${v.lossPct}%)`);
        }
      }).catch(() => {});
    }, 5000);
  }

  private stopVideoDiag() {
    if (this.diagTimer) { clearInterval(this.diagTimer); this.diagTimer = null; }
  }

  private pc: any = null;
  private connGen = 0;
  private ip = '';
  private retryMs = 1000;
  private retryTimer: ReturnType<typeof setTimeout> | null = null;
  private route = '';
  private audioTx: any = null;
  private micTrack: any = null;
  private autoTrack: any = null;
  private remoteAudio: any = null;
  private listenGate = false;
  private micHeld = false;
  private appVolume = 1.0;
  private cmdDc: any = null;
  private visionDc: any = null;
  private lastSourceId: number | null = null;
  private cmdSeq = 0;
  private cmdPending = new Map<string, { resolve: (v: any) => void; reject: (e: any) => void; timer: ReturnType<typeof setTimeout> }>();
  streamerCommand(method: string, path: string, body: object = {}): Promise<any> {
    if (simEngine.active) return Promise.reject(new Error(`${path} — 물리 시뮬 중에는 로봇으로 보내지 않는다`));
    return new Promise((resolve, reject) => {
      const dc = this.cmdDc;
      if (!dc || dc.readyState !== 'open') { reject(new Error(`${path} — 스트리머 command 채널 미연결`)); return; }
      const id = 's-' + (++this.cmdSeq) + '-' + Math.random().toString(16).slice(2, 6);
      const timer = setTimeout(() => { if (this.cmdPending.delete(id)) reject(new Error(`${path} — 스트리머 command 응답 없음(8s)`)); }, 8000);
      this.cmdPending.set(id, { resolve, reject, timer });
      try { dc.send(JSON.stringify({ id, method, path, body: { ...body, msg_id: id } })); }
      catch (e) { clearTimeout(timer); this.cmdPending.delete(id); reject(e); }
    });
  }
  private onCmdResponse(data: any) {
    let env: any; try { env = JSON.parse(String(data)); } catch { return; }
    const p = env && this.cmdPending.get(env.id);
    if (!p) return;
    clearTimeout(p.timer); this.cmdPending.delete(env.id);
    const status = Number(env.status ?? 0);
    if (status >= 200 && status < 400) p.resolve(env.body ?? {});
    else p.reject(new Error(`${env.path ?? ''} → command status ${status}`));
  }
  private clearCmdPending(reason: string) {
    for (const [, p] of this.cmdPending) { clearTimeout(p.timer); p.reject(new Error(reason)); }
    this.cmdPending.clear();
  }
  private syncAudioUnit() {
    iosAudioSession.setEnabled(this.listenGate || this.micHeld);
  }

  private headers(): Record<string, string> {
    return { ...robotAuth(), 'Content-Type': 'application/json' };
  }
  private base() {
    return streamerBase(this.ip);
  }

  private connecting: Promise<void> | null = null;

  private offeredSpectate: boolean | null = null;
  private roleTimer: ReturnType<typeof setTimeout> | null = null;
  reofferForRole() {
    if (this.roleTimer) clearTimeout(this.roleTimer);
    this.roleTimer = setTimeout(() => {
      this.roleTimer = null;
      if (!this.ip || this.offeredSpectate === null || this.offeredSpectate === !useRobot.getState().isMine) return;
      const ip = this.ip;
      this.disconnect();
      this.ip = '';
      this.retryMs = 1000;
      this.connect(ip);
    }, 800);
  }

  private visionRxAt = 0;
  private wdTimer: ReturnType<typeof setInterval> | null = null;
  private recheckOwner = false;
  private armVisionPeerListener(pc: any, ip: string) {
    pc.addEventListener?.('connectionstatechange', () => {
      if (this.pc !== pc) return;
      const st = pc.connectionState;
      if (st === 'failed' || st === 'closed') this.failAndRetry(pc, ip, '영상 연결 끊김');
    });
  }

  private startVisionStaleWatch(pc: any, ip: string) {
    this.visionRxAt = Date.now();
    if (this.wdTimer) { clearInterval(this.wdTimer); this.wdTimer = null; }
    this.wdTimer = setInterval(() => {
      if (this.pc !== pc) { if (this.wdTimer) { clearInterval(this.wdTimer); this.wdTimer = null; } return; }
      if (Date.now() - this.visionRxAt > VISION_STALE_MS) this.failAndRetry(pc, ip, '영상 수신 없음');
    }, VISION_WATCHDOG_TICK_MS);
  }

  private failAndRetry(pc: any, ip: string, reason: string) {
    this.stopVideoDiag();
    if (this.pc !== pc) return;
    this.pc = null;
    try { pc.close(); } catch {}
    this.audioTx = null;
    const wait = this.retryMs;
    this.retryMs = Math.min(this.retryMs * 2, 8000);
    useWebrtcStore.setState({ status: reason, retryS: Math.round(wait / 1000) });
    armVideoStaleClear(() => { if (!this.pc) useWebrtcStore.setState({ url: null }); });
    this.retryTimer = setTimeout(() => { this.retryTimer = null; if (this.ip === ip) this.connect(ip); }, wait);
  }

  retryNowIfWaiting() {
    if (!this.retryTimer || !this.ip || this.pc) return;
    clearTimeout(this.retryTimer); this.retryTimer = null;
    this.retryMs = 1000;
    const ip = this.ip; this.ip = '';
    this.connect(ip);
  }

  connect(ip: string) {
    if (isDemo()) return;
    const route = routeKey();
    if (this.pc && this.ip === ip && this.route === route) return;
    this.route = route;
    this.teardownForReconnect();
    if (ip !== this.ip) this.retryMs = 1000;
    this.ip = ip;
    useWebrtcStore.setState({ status: '연결 중…', retryS: undefined });
    noteVisionLink('connecting');
    iosAudioSession.arm();
    const gen = ++this.connGen;
    this.connecting = (async () => {
      let pc: any = null;
      try {
        await waitRoleKnown();
        if (gen !== this.connGen) return;
        let ticket = '';
        if (rendezvousEnabled()) {
          if (!useRobot.getState().isMine) {
            useWebrtcStore.setState({ status: '원격 관전 중 — 영상은 조종하는 기기만 받습니다', retryS: undefined });
            noteVisionLink('disconnected');
            this.offeredSpectate = true;
            return;
          }
          ticket = await currentTicket(CLIENT_ID);
          if (!ticket) {
            this.failAndRetry(pc, ip, '접근 코드 대기 중');
            return;
          }
          if (gen !== this.connGen) return;
        }
        const iceServers = await resolveIceServers();
        if (gen !== this.connGen) return;
        pc = new RTCPeerConnection({ iceServers,
          audioJitterBufferMaxPackets: 50, audioJitterBufferFastAccelerate: true } as any);
        this.pc = pc;
        this.armVisionPeerListener(pc, ip);
        this.startVideoDiag(pc);
        let dropTimer: ReturnType<typeof setTimeout> | null = null;
        pc.addEventListener('connectionstatechange', () => {
          if (this.pc !== pc) { if (dropTimer) clearTimeout(dropTimer); return; }
          const st = pc.connectionState;
          if (st === 'connected') { if (dropTimer) { clearTimeout(dropTimer); dropTimer = null; } return; }
          if (st === 'failed' || st === 'closed') { console.warn('[webrtc] 연결 끊김:', st); this.failAndRetry(pc, ip, '영상 연결 끊김'); return; }
          if (st === 'disconnected' && !dropTimer) {
            dropTimer = setTimeout(() => {
              dropTimer = null;
              if (this.pc === pc && pc.connectionState !== 'connected') { console.warn('[webrtc] 연결 끊김: disconnected 4s'); this.failAndRetry(pc, ip, '영상 연결 끊김'); }
            }, 4000);
          }
        });
        pc.addEventListener('track', (e: any) => {
          if (e.track?.kind === 'audio') {
            this.remoteAudio = e.track;
            e.track.enabled = this.listenGate;
            try { e.track._setVolume?.(this.appVolume); } catch {}
            return;
          }
          if (e.streams && e.streams[0]) {
            const url = e.streams[0].toURL();
            if (useWebrtcStore.getState().url !== url) useWebrtcStore.setState({ url, status: '', retryS: undefined });
            else useWebrtcStore.setState({ status: '', retryS: undefined });
          }
        });
        pc.addTransceiver('video', { direction: 'recvonly' });
        attachPointcloudChannel(pc);
        attachElevationChannel(pc);
        attachWallmapChannel(pc);
        this.cmdDc = pc.createDataChannel('command');
        this.cmdDc.onmessage = (e: any) => this.onCmdResponse(e.data);
        this.cmdDc.onopen = () => {
          setStreamerCommandSender((m, p, b) => this.streamerCommand(m, p, b));
          this.streamerCommand('PUT', '/api/vision/stream/live', { id: this.lastSourceId ?? 0 })
            .then((r: any) => console.log('[video] source(open)', this.lastSourceId ?? 0, JSON.stringify(r))).catch(() => {});
          if (this.listenGate)
            this.streamerCommand('PUT', '/api/audio/ptt_listen', { state: 'start' }).catch(() => {});
          const a = useSettings.getState().audio;
          this.streamerCommand('PUT', '/api/audio/speaker_volume', { percent: Math.round(a.robotSpk) }).catch(() => {});
          this.streamerCommand('PUT', '/api/audio/mic_volume', { percent: Math.round(a.robotMic) }).catch(() => {});
        };
        this.cmdDc.onclose = () => {
          setStreamerCommandSender(null); this.clearCmdPending('스트리머 command 채널 닫힘');
          if (this.pc === pc && pc.connectionState !== 'closed') {
            setTimeout(() => { if (this.pc === pc) { console.warn('[webrtc] command 채널 닫힘 — 재연결'); this.failAndRetry(pc, ip, '영상 연결 끊김'); } }, 1500);
          }
        };
        this.visionDc = pc.createDataChannel('vision-state');
        try { this.visionDc.binaryType = 'arraybuffer'; } catch {}
        this.visionDc.onopen = () => {
          const dc = this.visionDc;
          setVisionRequestSender((bytes) => { if (dc?.readyState === 'open') dc.send(bytes); });
          noteVisionLink('connected');
          this.startVisionStaleWatch(pc, ip);
        };
        this.visionDc.onclose = () => {
          setVisionRequestSender(null);
          noteVisionLink('disconnected');
          if (this.pc === pc) { this.recheckOwner = true; this.retryMs = ROBOT_CLOSED_RETRY_MS; this.failAndRetry(pc, ip, '영상 연결 끊김'); }
        };
        this.visionDc.onmessage = (e: any) => { this.visionRxAt = Date.now(); if (e?.data instanceof ArrayBuffer) handleVisionStateFrame(e.data); };
        if (!IS_IOS_SIM) {
          this.audioTx = pc.addTransceiver('audio', { direction: 'sendrecv' });
          const autoTrack = this.audioTx?.sender?.track;
          if (autoTrack) {
            autoTrack.enabled = false;
            this.autoTrack = autoTrack;
          }
        }
        const offer = await pc.createOffer({});
        await pc.setLocalDescription({ type: 'offer', sdp: stripTransportCc(offer.sdp) });
        await waitIce(pc);
        let ans: { sdp?: string };
        if (rendezvousEnabled() && ticket) {
          try {
            ans = await rendezvousExchange({
              url: rendezvousUrl(), robot: robotId(), service: 'vision',
              clientId: CLIENT_ID, sdp: pc.localDescription.sdp,
              token: useSettings.getState().webrtcToken ?? '',
              ticket,
            });
          } catch (e: any) {
            if (/not allowed/.test(String(e?.message ?? ''))) {
              this.stopVideoDiag();
              if (this.pc === pc) { try { pc.close(); } catch {} this.pc = null; }
              useWebrtcStore.setState({ status: '이 로봇에 접속할 권한이 없습니다' });
              return;
            }
            console.warn('[webrtc] 랑데부 실패:', e?.message ?? String(e));
            this.failAndRetry(pc, ip, '랑데부 실패');
            return;
          }
        } else {
          if (this.recheckOwner) {
            this.recheckOwner = false;
            try {
              const d: any = await rest.getOwnership(ip);
              useRobot.getState().setOwnership({ owner: d?.ownerIP || '', myIp: d?.requesterIP || '', isMine: !!d?.IsOwner });
            } catch {}
            if (gen !== this.connGen) return;
          }
          this.offeredSpectate = !useRobot.getState().isMine;
          const res = await fetch(`${this.base()}/api/webrtc/offer`, {
            method: 'POST',
            headers: this.headers(),
            body: JSON.stringify({ sdp: pc.localDescription.sdp, clientId: CLIENT_ID, takeover: !this.offeredSpectate, spectate: this.offeredSpectate, token: useSettings.getState().webrtcToken ?? '', ...modeField() }),
          });
          ans = await res.json();
          if (!ans?.sdp) {
            console.warn('[webrtc] answer 없음:', (ans as any)?.status ?? res.status);
            if ((ans as any)?.status === 'busy') { this.failAndRetry(pc, ip, '영상 자리 없음'); return; }
            this.failAndRetry(pc, ip, 'answer 없음');
            return;
          }
        }
        let sdp = String(ans.sdp);
        if (ip === '10.0.2.2') sdp = sdp.replace(/(\d{1,3}\.\d{1,3}\.\d{1,3}\.\d{1,3})/g, (m) => (m === '0.0.0.0' ? m : '10.0.2.2'));
        if (this.pc !== pc || pc.signalingState === 'closed') return;
        await pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp }));
        if (this.pc !== pc) return;
        this.retryMs = 1000;
        this.killAutoTrack();
        this.primeReleaseMic();
        this.syncAudioUnit();
        cancelVideoStaleClear();
        useWebrtcStore.setState({ status: '영상 대기…', retryS: undefined });
      } catch (e: any) {
        console.warn('[webrtc] 연결 실패:', e?.message ?? String(e));
        this.failAndRetry(pc, ip, '영상 연결 실패');
      }
    })();
  }

  private async killAutoTrack() {
    const s = this.audioTx?.sender;
    if (!s || !this.autoTrack) return;
    try { await s.replaceTrack?.(null); } catch {}
    if (!s.track) {
      try { this.autoTrack.stop?.(); } catch {}
      this.autoTrack = null;
    }
  }

  private async primeReleaseMic() {
    if (Platform.OS !== 'android') return;
    try {
      const granted = await PermissionsAndroid.check(PermissionsAndroid.PERMISSIONS.RECORD_AUDIO);
      if (!granted) return;
      const stream: any = await mediaDevices.getUserMedia({ audio: true });
      const tr = stream.getAudioTracks()[0];
      try { tr.enabled = false; } catch {}
      try { await this.audioTx?.sender?.replaceTrack?.(tr); } catch {}
      try { await this.audioTx?.sender?.replaceTrack?.(null); } catch {}
      try { tr.stop?.(); } catch {}
    } catch { }
  }

  get live() { return !!this.connecting || !!this.pc; }

  stats() { return readStats(this.pc); }

  ensureConnected(ip: string) {
    if (this.ip === ip && this.route === routeKey() && (this.connecting || this.pc)) return;
    this.connect(ip);
  }

  resumeAfterBackground() {
    const ip = this.ip;
    if (!ip) return;
    const st = this.pc?.connectionState ?? this.pc?.iceConnectionState;
    if (this.pc && st !== 'failed' && st !== 'closed' && st !== 'disconnected') return;
    this.teardownForReconnect();
    this.retryMs = 1000;
    this.ip = '';
    this.connect(ip);
  }

  setSource(streamId: number) {
    if (!this.ip) return;
    this.lastSourceId = streamId;
    if (this.cmdDc?.readyState !== 'open') { console.log('[video] source', streamId, '(DC 대기)'); return; }
    this.streamerCommand('PUT', '/api/vision/stream/live', { id: streamId })
      .then((r: any) => console.log('[video] source', streamId, JSON.stringify(r)))
      .catch((e: any) => console.log('[video] source', streamId, 'fail', String(e?.message ?? e)));
  }

  async setSpeak(on: boolean): Promise<'ok' | 'denied' | 'unavailable'> {
    if (!on) {
      if (this.micTrack) {
        this.micTrack.enabled = false;
        if (this.micTrack !== this.autoTrack) {
          try { await this.audioTx?.sender?.replaceTrack?.(null); } catch {}
          try { this.micTrack.stop?.(); } catch {}
        }
      }
      this.micTrack = null;
      this.micHeld = false;
      this.syncAudioUnit();
      return 'ok';
    }
    if (!this.pc) this.retryNowIfWaiting();
    try { await this.connecting; } catch {}
    if (!this.pc || !this.audioTx) return 'unavailable';
    this.micHeld = true;
    this.syncAudioUnit();
    const fail = (r: 'denied' | 'unavailable') => { this.micHeld = false; this.syncAudioUnit(); return r; };
    try {
      if (Platform.OS === 'android') {
        const r = await PermissionsAndroid.request(PermissionsAndroid.PERMISSIONS.RECORD_AUDIO);
        if (r !== PermissionsAndroid.RESULTS.GRANTED) return fail('denied');
      }
      const stream: any = await mediaDevices.getUserMedia({ audio: true });
      const fresh = stream.getAudioTracks()[0];
      try { await this.audioTx.sender?.replaceTrack?.(fresh); } catch {}
      if (this.audioTx.sender?.track?.id === fresh.id) {
        if (this.autoTrack && this.autoTrack !== fresh) { try { this.autoTrack.stop?.(); } catch {} this.autoTrack = null; }
        this.micTrack = fresh;
      } else {
        try { fresh.stop?.(); } catch {}
        if (!this.autoTrack) return fail('unavailable');
        this.micTrack = this.autoTrack;
      }
      this.micTrack.enabled = true;
      return 'ok';
    } catch {
      return fail('denied');
    }
  }

  setListenGate(on: boolean) {
    this.listenGate = on;
    if (this.remoteAudio) this.remoteAudio.enabled = on;
    this.syncAudioUnit();
  }

  setAppVolume(pct: number) {
    this.appVolume = Math.max(0, Math.min(100, pct)) / 100;
    try { this.remoteAudio?._setVolume?.(this.appVolume); } catch {}
  }

  private teardownForReconnect() {
    const keepUrl = useWebrtcStore.getState().url;
    this.disconnect();
    if (keepUrl) useWebrtcStore.setState({ url: keepUrl, status: '연결 중…' });
  }

  disconnect() {
    this.offeredSpectate = null;
    cancelVideoStaleClear();
    resetAllClouds();
    const ip = this.ip;
    this.connGen++;
    if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
    try { this.micTrack?.stop?.(); } catch {}
    this.micTrack = null;
    this.micHeld = false;
    iosAudioSession.setEnabled(false);
    try { this.autoTrack?.stop?.(); } catch {}
    this.autoTrack = null;
    this.remoteAudio = null;
    this.audioTx = null;
    this.connecting = null;
    setStreamerCommandSender(null);
    setVisionRequestSender(null);
    this.clearCmdPending('스트리머 연결 해제');
    this.cmdDc = null;
    this.visionDc = null;
    noteVisionLink('disconnected');
    if (this.wdTimer) { clearInterval(this.wdTimer); this.wdTimer = null; }
    const oldPc = this.pc;
    this.pc = null;
    try {
      oldPc?.close?.();
    } catch {}
    if (ip)
      fetch(`${streamerBase(ip)}/api/webrtc/hangup`, {
        method: 'POST',
        headers: this.headers(),
        body: JSON.stringify({ clientId: CLIENT_ID, token: useSettings.getState().webrtcToken ?? '' }),
      }).catch(() => {});
    useWebrtcStore.setState({ url: null, status: '미연결', retryS: undefined });
  }
}

export const webrtcClient = new WebrtcClient();
setKeyframeProbe(async () => inboundVideoFromStats(await webrtcClient.stats())?.keyFramesDecoded);
useRobot.subscribe((s, prev) => { if (s.isMine !== prev.isMine) webrtcClient.reofferForRole(); });
