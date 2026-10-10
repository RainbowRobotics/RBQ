import { create } from 'zustand';
import { robotAuth } from '@/lib/auth';
import { isDemo } from '@/lib/demoFlag';
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
import { isDesktop } from '@/lib/desktopBridge';
import { inboundVideoFromStats, readStats } from '@/lib/connectionRoute';
import { setKeyframeProbe } from '@/lib/videoKeyframe';
import { desktopVideo } from '@/lib/desktopVideo';
import { desktopAudio } from '@/lib/desktopAudio';
import { simEngine } from './simEngine';
import { armVideoStaleClear, cancelVideoStaleClear } from './videoStale';
import { rest } from './rest';

const CLIENT_ID = 'rbq-app-web-' + Math.random().toString(16).slice(2, 10);
const VISION_STALE_MS = 3000;
const VISION_WATCHDOG_TICK_MS = 1000;
const ROBOT_CLOSED_RETRY_MS = 250;

type WebrtcState = { url: string | null; stream?: unknown; imgMode?: boolean; status: string; retryS?: number };
export const useWebrtcStore = create<WebrtcState>(() => ({ url: null, stream: null, imgMode: false, status: '미연결' }));
export const useWebrtcStream = () => useWebrtcStore();

function forceH264Pt96(sdp: string): string {
  const lines = sdp.split('\r\n');
  const mIdx = lines.findIndex((l) => l.startsWith('m=video'));
  if (mIdx < 0) return sdp;
  let end = lines.findIndex((l, i) => i > mIdx && l.startsWith('m='));
  if (end < 0) end = lines.length;
  const kept = lines.slice(mIdx + 1, end).filter((l) => l.length > 0 && !/^a=(rtpmap|fmtp|rtcp-fb):/.test(l));
  const mParts = lines[mIdx].split(' ');
  const rebuilt = [
    `${mParts[0]} ${mParts[1]} ${mParts[2]} 96`,
    ...kept,
    'a=rtpmap:96 H264/90000',
    'a=rtcp-fb:96 nack',
    'a=rtcp-fb:96 nack pli',
    'a=rtcp-fb:96 goog-remb',
    'a=fmtp:96 level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f',
  ];
  const out = [...lines.slice(0, mIdx), ...rebuilt, ...lines.slice(end)].filter((l) => l.length > 0);
  return out.join('\r\n') + '\r\n';
}

function waitIce(pc: RTCPeerConnection): Promise<void> {
  return new Promise((res) => {
    if (pc.iceGatheringState === 'complete') return res();
    const f = () => {
      if (pc.iceGatheringState === 'complete') {
        pc.removeEventListener('icegatheringstatechange', f);
        res();
      }
    };
    pc.addEventListener('icegatheringstatechange', f);
    setTimeout(res, 3000);
  });
}

class WebrtcClient {
  private pc: RTCPeerConnection | null = null;
  private ip = '';
  private retryMs = 1000;
  private retryTimer: ReturnType<typeof setTimeout> | null = null;
  private audioTx: RTCRtpTransceiver | null = null;
  private micTrack: MediaStreamTrack | null = null;
  private audioEl: HTMLAudioElement | null = null;
  private listenGate = false;
  private appVolume = 1.0;
  private cmdDc: RTCDataChannel | null = null;
  private visionDc: RTCDataChannel | null = null;
  private lastSourceId: number | null = null;
  private cmdSeq = 0;
  private cmdPending = new Map<string, { resolve: (v: any) => void; reject: (e: any) => void; timer: ReturnType<typeof setTimeout> }>();
  streamerCommand(method: string, path: string, body: object = {}): Promise<any> {
    if (simEngine.active) return Promise.reject(new Error(`${path} — 물리 시뮬 중에는 로봇으로 보내지 않는다`));
    return new Promise((resolve, reject) => {
      const dc = this.cmdDc;
      if (!dc || dc.readyState !== 'open') { reject(new Error(`${path} — 스트리머 command 채널 미연결`)); return; }
      const id = 's-' + (++this.cmdSeq) + '-' + Math.random().toString(16).slice(2, 6);
      const timer = setTimeout(() => {
        if (this.cmdPending.delete(id)) reject(new Error(`${path} — 스트리머 command 응답 없음(8s)`));
      }, 8000);
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
  private headers(): Record<string, string> {
    return { ...robotAuth(), 'Content-Type': 'application/json' };
  }
  private base() {
    return streamerBase(this.ip);
  }

  private connecting: Promise<void> | null = null;

  private visionRxAt = 0;
  private wdTimer: ReturnType<typeof setInterval> | null = null;
  private recheckOwner = false;
  private armVisionPeerListener(pc: RTCPeerConnection, ip: string) {
    pc.addEventListener('connectionstatechange', () => {
      if (this.pc !== pc) return;
      const st = pc.connectionState;
      if (st === 'failed' || st === 'closed') this.failAndRetry(pc, ip, '영상 연결 끊김');
    });
  }

  private startVisionStaleWatch(pc: RTCPeerConnection | null, ip: string) {
    if (!pc) return;
    this.visionRxAt = Date.now();
    if (this.wdTimer) { clearInterval(this.wdTimer); this.wdTimer = null; }
    this.wdTimer = setInterval(() => {
      if (this.pc !== pc) { if (this.wdTimer) { clearInterval(this.wdTimer); this.wdTimer = null; } return; }
      if (Date.now() - this.visionRxAt > VISION_STALE_MS) this.failAndRetry(pc, ip, '영상 수신 없음');
    }, VISION_WATCHDOG_TICK_MS);
  }

  private failAndRetry(pc: RTCPeerConnection | null, ip: string, reason: string) {
    if (this.pc !== pc) return;
    this.pc = null;
    try { pc?.close(); } catch {}
    this.audioTx = null;
    const wait = this.retryMs;
    this.retryMs = Math.min(this.retryMs * 2, 8000);
    useWebrtcStore.setState({ status: reason, retryS: Math.round(wait / 1000) });
    armVideoStaleClear(() => { if (!this.pc) useWebrtcStore.setState({ url: null, stream: null }); });
    this.retryTimer = setTimeout(() => { this.retryTimer = null; if (this.ip === ip) this.connect(ip); }, wait);
  }

  private offeredSpectate: boolean | null = null;
  private roleTimer: ReturnType<typeof setTimeout> | null = null;
  reofferForRole() {
    if (this.roleTimer) clearTimeout(this.roleTimer);
    this.roleTimer = setTimeout(() => {
      this.roleTimer = null;
      if (!this.ip) return;
      if (!isDesktop() && (this.offeredSpectate === null || this.offeredSpectate === !useRobot.getState().isMine)) return;
      const ip = this.ip; this.disconnect(); this.ip = ''; this.retryMs = 1000; this.connect(ip);
    }, 800);
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
    if (isDesktop()) { this.ip = ip; desktopVideo.connect(ip); return; }
    if (this.pc && this.ip === ip) return;
    this.disconnect();
    if (ip !== this.ip) this.retryMs = 1000;
    this.ip = ip;
    useWebrtcStore.setState({ status: '연결 중…', retryS: undefined });
    noteVisionLink('connecting');
    this.connecting = (async () => {
      let pc: RTCPeerConnection | null = null;
      try {
        pc = new RTCPeerConnection({ iceServers: [] });
        this.pc = pc;
        this.armVisionPeerListener(pc, ip);
        pc.addEventListener('track', (e) => {
          if (e.track.kind === 'audio') {
            if (!this.audioEl) {
              this.audioEl = document.createElement('audio');
              this.audioEl.autoplay = true;
              document.body.appendChild(this.audioEl);
              this.applySpeakerDevice(useSettings.getState().gcsSpeakerId);
            }
            this.audioEl.srcObject = new MediaStream([e.track]);
            this.audioEl.muted = !this.listenGate;
            this.audioEl.volume = this.appVolume;
            return;
          }
          if (e.streams && e.streams[0]) useWebrtcStore.setState({ stream: e.streams[0], status: '', retryS: undefined });
        });
        pc.addTransceiver('video', { direction: 'recvonly' });
        this.audioTx = pc.addTransceiver('audio', { direction: 'sendrecv' });
        attachPointcloudChannel(pc);
        attachElevationChannel(pc);
        attachWallmapChannel(pc);
        this.cmdDc = pc.createDataChannel('command');
        this.cmdDc.onmessage = (e) => this.onCmdResponse(e.data);
        this.cmdDc.onopen = () => {
          setStreamerCommandSender((m, p, b) => this.streamerCommand(m, p, b));
          this.streamerCommand('PUT', '/api/vision/stream/live', { id: this.lastSourceId ?? 0 }).catch(() => {});
          if (this.listenGate)
            this.streamerCommand('PUT', '/api/audio/ptt_listen', { state: 'start' }).catch(() => {});
          const a = useSettings.getState().audio;
          this.streamerCommand('PUT', '/api/audio/speaker_volume', { percent: Math.round(a.robotSpk) }).catch(() => {});
          this.streamerCommand('PUT', '/api/audio/mic_volume', { percent: Math.round(a.robotMic) }).catch(() => {});
        };
        this.cmdDc.onclose = () => { setStreamerCommandSender(null); this.clearCmdPending('스트리머 command 채널 닫힘'); };
        this.visionDc = pc.createDataChannel('vision-state');
        this.visionDc.binaryType = 'arraybuffer';
        this.visionDc.onopen = () => {
          const dc = this.visionDc;
          setVisionRequestSender((bytes) => { if (dc?.readyState === 'open') dc.send(bytes as Uint8Array<ArrayBuffer>); });
          noteVisionLink('connected');
          this.startVisionStaleWatch(pc, ip);
        };
        this.visionDc.onclose = () => {
          setVisionRequestSender(null);
          noteVisionLink('disconnected');
          if (this.pc === pc) { this.recheckOwner = true; this.retryMs = ROBOT_CLOSED_RETRY_MS; this.failAndRetry(pc, ip, '영상 연결 끊김'); }
        };
        this.visionDc.onmessage = (e) => { this.visionRxAt = Date.now(); if (e.data instanceof ArrayBuffer) handleVisionStateFrame(e.data); };
        const offer = await pc.createOffer();
        await pc.setLocalDescription({ type: 'offer', sdp: forceH264Pt96(offer.sdp!) });
        await waitIce(pc);
        if (this.recheckOwner) {
          this.recheckOwner = false;
          try {
            const d: any = await rest.getOwnership(ip);
            useRobot.getState().setOwnership({ owner: d?.ownerIP || '', myIp: d?.requesterIP || '', isMine: !!d?.IsOwner });
          } catch {}
          if (this.pc !== pc) return;
        }
        await waitRoleKnown();
        if (this.pc !== pc) return;
        this.offeredSpectate = !useRobot.getState().isMine;
        const res = await fetch(`${this.base()}/api/webrtc/offer`, {
          method: 'POST',
          headers: this.headers(),
          body: JSON.stringify({ sdp: pc.localDescription?.sdp, clientId: CLIENT_ID, takeover: !this.offeredSpectate, spectate: this.offeredSpectate, token: useSettings.getState().webrtcToken ?? '' }),
        });
        const ct = res.headers.get('content-type') ?? '';
        if (!res.ok || !ct.includes('application/json')) {
          this.failAndRetry(pc, ip, '카메라 없음 (로봇 미연결)');
          return;
        }
        const ans = await res.json().catch(() => null);
        if (!ans?.sdp) {
          if (ans?.status === 'busy') { this.failAndRetry(pc, ip, '영상 자리 없음'); return; }
          this.failAndRetry(pc, ip, '카메라 없음 (영상 응답 없음)');
          return;
        }
        await pc.setRemoteDescription({ type: 'answer', sdp: String(ans.sdp) });
        if (this.pc !== pc) return;
        this.retryMs = 1000;
        cancelVideoStaleClear();
        useWebrtcStore.setState({ status: '영상 대기…', retryS: undefined });
      } catch (e: any) {
        const msg = String(e?.message ?? e);
        console.warn('[webrtc] offer failed:', msg);
        const clean = /JSON|Unexpected token|Failed to fetch|NetworkError/i.test(msg)
          ? '카메라 없음 (로봇 미연결)' : '영상 연결 실패';
        this.failAndRetry(pc, ip, clean);
      }
    })();
  }

  get live() {
    if (isDesktop()) return desktopVideo.live;
    return !!this.connecting || !!this.pc;
  }

  stats() { return readStats(this.pc); }

  ensureConnected(ip: string) {
    if (this.ip === ip) {
      if (this.connecting) return;
      if (this.pc && (this.pc.connectionState === 'connected' || this.pc.connectionState === 'connecting')) return;
    }
    this.connect(ip);
  }

  setSource(streamId: number) {
    if (isDesktop()) { desktopVideo.setSource(streamId); return; }
    if (!this.ip) return;
    this.lastSourceId = streamId;
    if (this.cmdDc?.readyState !== 'open') return;
    this.streamerCommand('PUT', '/api/vision/stream/live', { id: streamId }).catch(() => {});
  }

  async setSpeak(on: boolean): Promise<'ok' | 'denied' | 'unavailable'> {
    if (isDesktop()) {
      if (!on) { desktopAudio.stopSpeak(); return 'ok'; }
      return desktopAudio.startSpeak();
    }
    if (!on) {
      if (this.micTrack) this.micTrack.enabled = false;
      return 'ok';
    }
    if (!this.pc) this.retryNowIfWaiting();
    try { await this.connecting; } catch {}
    if (!this.pc || !this.audioTx) return 'unavailable';
    try {
      if (!this.micTrack) {
        const micId = useSettings.getState().gcsMicId;
        const stream = await navigator.mediaDevices.getUserMedia({
          audio: micId ? { deviceId: micId } : true,
        });
        this.micTrack = stream.getAudioTracks()[0];
        await this.audioTx.sender.replaceTrack(this.micTrack);
      }
      this.micTrack.enabled = true;
      return 'ok';
    } catch {
      return 'denied';
    }
  }

  applySpeakerDevice(deviceId: string) {
    const el: any = this.audioEl;
    if (el && typeof el.setSinkId === 'function') el.setSinkId(deviceId || '').catch(() => {});
  }

  resetMicTrack() {
    if (isDesktop()) { desktopAudio.resetMic(); return; }
    if (!this.micTrack) return;
    try { this.micTrack.stop(); } catch {}
    this.audioTx?.sender.replaceTrack(null).catch(() => {});
    this.micTrack = null;
  }

  setListenGate(on: boolean) {
    if (isDesktop()) { desktopAudio.setListen(on); return; }
    this.listenGate = on;
    if (this.audioEl) this.audioEl.muted = !on;
  }

  setAppVolume(pct: number) {
    if (isDesktop()) { desktopAudio.setVolume(pct); return; }
    this.appVolume = Math.max(0, Math.min(100, pct)) / 100;
    if (this.audioEl) this.audioEl.volume = this.appVolume;
  }

  disconnect() {
    this.offeredSpectate = null;
    cancelVideoStaleClear();
    resetAllClouds();
    if (isDesktop()) { desktopVideo.disconnect(); return; }
    const ip = this.ip;
    if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
    try { this.micTrack?.stop(); } catch {}
    this.micTrack = null;
    this.audioTx = null;
    this.connecting = null;
    setStreamerCommandSender(null);
    setVisionRequestSender(null);
    this.clearCmdPending('스트리머 연결 해제');
    this.cmdDc = null;
    this.visionDc = null;
    noteVisionLink('disconnected');
    if (this.audioEl) { try { this.audioEl.remove(); } catch {} this.audioEl = null; }
    if (this.wdTimer) { clearInterval(this.wdTimer); this.wdTimer = null; }
    const oldPc = this.pc;
    this.pc = null;
    try {
      oldPc?.close();
    } catch {}
    if (ip)
      fetch(`${streamerBase(ip)}/api/webrtc/hangup`, {
        method: 'POST',
        headers: this.headers(),
        body: JSON.stringify({ clientId: CLIENT_ID, token: useSettings.getState().webrtcToken ?? '' }),
      }).catch(() => {});
    useWebrtcStore.setState({ url: null, stream: null, status: '미연결', retryS: undefined });
  }
}

export const webrtcClient = new WebrtcClient();
if (!isDesktop()) setKeyframeProbe(async () => inboundVideoFromStats(await webrtcClient.stats())?.keyFramesDecoded);
useRobot.subscribe((s, prev) => { if (s.isMine !== prev.isMine) webrtcClient.reofferForRole(); });
if (typeof window !== 'undefined') (window as any).__rbqVideo = webrtcClient;
