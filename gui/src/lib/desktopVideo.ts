import { useWebrtcStore } from './webrtcClient';
import { setStreamerCommandSender, setVisionRequestSender } from './commandBus';
import { desktopAudio } from './desktopAudio';
import { handlePointcloudFrame, resetAllClouds } from '@/lib/pointcloud';
import { handleVisionStateFrame } from './visionStateDc';
import { handleElevationFrame } from './elevationMap';
import { handleWallMapFrame } from './wallMap';
import { useRobot } from '@/store/robot';
import { noteVisionLink } from '@/store/telemetry';
import { simEngine } from './simEngine';
import { armVideoStaleClear, cancelVideoStaleClear } from './videoStale';
import { keyframeWanted, noteKeyframe } from './videoKeyframe';

const VID = { FRAME: 0x00, COMMAND: 0x01, AUDIO: 0x02, MIC: 0x03, VSTATE: 0x04, PCD: 0x05, ELEVATION: 0x06, WALLMAP: 0x07, CTRL: 0xff } as const;

class DesktopVideo {
  private ws: WebSocket | null = null;
  private ip = '';
  private imgEl: HTMLImageElement | null = null;
  private curUrl: string | null = null;
  private idrPending = false;
  private pendingUrl: string | null = null;
  private groundImgEl: HTMLImageElement | null = null;
  private pendingGroundUrl: string | null = null;
  private cmdOpen = false;
  private lastSourceId: number | null = null;
  private cmdSeq = 0;
  private cmdPending = new Map<string, { resolve: (v: any) => void; reject: (e: any) => void; timer: ReturnType<typeof setTimeout> }>();

  private wanted = false;
  private retryMs = 1000;
  private retryTimer: ReturnType<typeof setTimeout> | null = null;
  private lost(status: string) {
    useWebrtcStore.setState({ status, retryS: undefined });
    this.armStale();
    noteVisionLink('disconnected');
    if (!this.wanted || this.retryTimer) return;
    const wait = this.retryMs;
    this.retryMs = Math.min(this.retryMs * 2, 8000);
    useWebrtcStore.setState({ retryS: Math.round(wait / 1000) });
    this.retryTimer = setTimeout(() => {
      this.retryTimer = null;
      if (!this.wanted) return;
      const ip = this.ip;
      try { this.ws?.close(); } catch { }
      this.ws = null;
      this.connect(ip);
    }, wait);
  }

  connect(ip: string) {
    if (this.ws && this.ip === ip && this.ws.readyState <= WebSocket.OPEN) return;
    if (ip !== this.ip) this.retryMs = 1000;
    this.disconnect();
    this.wanted = true;
    this.ip = ip;
    this.cmdOpen = false;
    useWebrtcStore.setState({ status: '연결 중…', imgMode: true, retryS: undefined });
    noteVisionLink('connecting');

    const proto = typeof location !== 'undefined' && location.protocol === 'https:' ? 'wss://' : 'ws://';
    const host = typeof location !== 'undefined' ? location.host : '127.0.0.1:8090';
    const ws = new WebSocket(`${proto}${host}/webrtc-video?own=${useRobot.getState().isMine ? 1 : 0}`);
    ws.binaryType = 'arraybuffer';
    this.ws = ws;
    ws.onmessage = (e) => this.onFrame(e.data as ArrayBuffer);
    ws.onclose = () => { if (this.ws === ws) this.lost('카메라 연결 끊김'); };
    ws.onerror = () => { if (this.ws === ws) this.lost('카메라 연결 실패'); };

    setStreamerCommandSender((m, p, b) => this.streamerCommand(m, p, b));
    setVisionRequestSender((bytes) => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) throw new Error('카메라 채널 미연결');
      const frame = new Uint8Array(1 + bytes.length);
      frame[0] = VID.VSTATE; frame.set(bytes, 1);
      this.ws.send(frame as unknown as ArrayBufferView);
    });
    desktopAudio.setSender((pcm) => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) return;
      const frame = new Uint8Array(1 + pcm.length);
      frame[0] = VID.MIC; frame.set(pcm, 1);
      try { this.ws.send(frame as unknown as ArrayBufferView); } catch { }
    });
  }

  setImgEl(el: HTMLImageElement | null) {
    this.imgEl = el;
    if (el && this.pendingUrl) { el.src = this.pendingUrl; this.pendingUrl = null; }
  }

  setGroundImgEl(el: HTMLImageElement | null) {
    this.groundImgEl = el;
    if (el && this.pendingGroundUrl) { el.src = this.pendingGroundUrl; this.pendingGroundUrl = null; }
  }

  private onFrame(data: ArrayBuffer) {
    const b = new Uint8Array(data);
    if (b.length < 1) return;
    const t = b[0], pl = b.subarray(1);
    if (t === VID.FRAME) {
      if (pl.length < 3) return;
      const url = URL.createObjectURL(new Blob([pl], { type: 'image/jpeg' }));
      const prev = this.curUrl;
      this.curUrl = url;
      if (this.imgEl) this.imgEl.src = url;
      else this.pendingUrl = url;
      if (this.groundImgEl) this.groundImgEl.src = url;
      else this.pendingGroundUrl = url;
      const el = this.imgEl;
      if (this.idrPending && el) {
        this.idrPending = false;
        if (!keyframeWanted()) noteKeyframe();
        else if (el.decode) el.decode().then(noteKeyframe, noteKeyframe); else noteKeyframe();
      }
      if (prev) { try { URL.revokeObjectURL(prev); } catch { } }
      cancelVideoStaleClear();
      if (!useWebrtcStore.getState().imgMode) useWebrtcStore.setState({ imgMode: true, status: '', retryS: undefined });
      if (this.cmdOpen === false) useWebrtcStore.setState({ status: '', retryS: undefined });
    } else if (t === VID.AUDIO) {
      desktopAudio.playPcm(pl);
    } else if (t === VID.PCD) {
      handlePointcloudFrame(pl.slice().buffer);
    } else if (t === VID.VSTATE) {
      handleVisionStateFrame(data.slice(1));
    } else if (t === VID.ELEVATION) {
      handleElevationFrame(data.slice(1));
    } else if (t === VID.WALLMAP) {
      handleWallMapFrame(data.slice(1));
    } else if (t === VID.COMMAND) {
      this.onCmdResponse(new TextDecoder().decode(pl));
    } else if (t === VID.CTRL) {
      let c: any; try { c = JSON.parse(new TextDecoder().decode(pl)); } catch { return; }
      if (c.t === 'open' && c.ch === 'command') { this.cmdOpen = true; this.setSource(this.lastSourceId ?? 0); }
      else if (c.t === 'open' && c.ch === 'vision-state') { noteVisionLink('connected'); this.retryMs = 1000; }
      else if (c.t === 'close' && c.ch === 'vision-state') noteVisionLink('disconnected');
      else if (c.t === 'pc') { if (c.state === 'failed' || c.state === 'closed') this.lost('카메라 연결 끊김'); }
      else if (c.t === 'idr') this.idrPending = true;
      else if (c.t === 'error') this.lost('카메라 오류');
    }
  }

  private clearFrame() {
    if (this.imgEl) { try { this.imgEl.removeAttribute('src'); } catch { } }
    if (this.groundImgEl) { try { this.groundImgEl.removeAttribute('src'); } catch { } }
    this.pendingGroundUrl = null;
    if (this.curUrl) { try { URL.revokeObjectURL(this.curUrl); } catch { } this.curUrl = null; }
    if (this.pendingUrl) { try { URL.revokeObjectURL(this.pendingUrl); } catch { } this.pendingUrl = null; }
    useWebrtcStore.setState({ imgMode: false });
  }

  private armStale() { armVideoStaleClear(() => this.clearFrame()); }

  get live() { return !!this.ws && this.ws.readyState === WebSocket.OPEN; }

  setSource(streamId: number) {
    this.lastSourceId = streamId;
    if (!this.cmdOpen) return;
    this.streamerCommand('PUT', '/api/vision/stream/live', { id: streamId }).catch(() => {});
  }

  streamerCommand(method: string, path: string, body: object = {}): Promise<any> {
    if (simEngine.active) return Promise.reject(new Error(`${path} — 물리 시뮬 중에는 로봇으로 보내지 않는다`));
    return new Promise((resolve, reject) => {
      if (!this.ws || this.ws.readyState !== WebSocket.OPEN) { reject(new Error(`${path} — 카메라 채널 미연결`)); return; }
      const id = 'd-' + (++this.cmdSeq) + '-' + Math.random().toString(16).slice(2, 6);
      const timer = setTimeout(() => { if (this.cmdPending.delete(id)) reject(new Error(`${path} — 응답 없음(8s)`)); }, 8000);
      this.cmdPending.set(id, { resolve, reject, timer });
      const env = JSON.stringify({ id, method, path, body: { ...body, msg_id: id } });
      const bytes = new TextEncoder().encode(env);
      const frame = new Uint8Array(1 + bytes.length);
      frame[0] = VID.COMMAND; frame.set(bytes, 1);
      try { this.ws.send(frame as unknown as ArrayBufferView); } catch (e) { clearTimeout(timer); this.cmdPending.delete(id); reject(e); }
    });
  }

  private onCmdResponse(data: string) {
    let env: any; try { env = JSON.parse(data); } catch { return; }
    const p = env && this.cmdPending.get(env.id);
    if (!p) return;
    clearTimeout(p.timer); this.cmdPending.delete(env.id);
    const status = Number(env.status ?? 0);
    if (status >= 200 && status < 400) p.resolve(env.body ?? {});
    else p.reject(new Error(`${env.path ?? ''} → status ${status}`));
  }

  disconnect() {
    this.wanted = false;
    if (this.retryTimer) { clearTimeout(this.retryTimer); this.retryTimer = null; }
    cancelVideoStaleClear();
    resetAllClouds();
    desktopAudio.stop();
    setStreamerCommandSender(null);
    setVisionRequestSender(null);
    for (const [, p] of this.cmdPending) { clearTimeout(p.timer); p.reject(new Error('카메라 연결 종료')); }
    this.cmdPending.clear();
    try { this.ws?.close(); } catch { }
    this.ws = null;
    this.cmdOpen = false;
    this.clearFrame();
    useWebrtcStore.setState({ imgMode: false, status: '미연결', retryS: undefined });
    noteVisionLink('disconnected');
  }
}

export const desktopVideo = new DesktopVideo();
