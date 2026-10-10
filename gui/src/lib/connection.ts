import { useRobot } from '@/store/robot';
import { useTelemetry, noteMotionLink } from '@/store/telemetry';
import { useSettings } from '@/store/settings';
import { useFeatures } from '@/store/capability';
import { simEngine } from '@/lib/simEngine';
import { useDockView } from '@/store/dockView';
import { useVisionToggles } from '@/store/visionToggles';
import { parseRobotState, parseDeviceStates, parsePduState, SIZEOF } from './robotState';
import { activeRobotKind, autoRobotVersion, motionFrames, robotKinds } from '@/modules/registry';
import { rest, actions, HttpError } from './rest';
import { setCommandSender } from './commandBus';
import { restBase } from './endpoints';
import { robotAuth, reportAuthFailure } from './auth';
import { routeDetail, readStats } from './connectionRoute';
import { currentTicket } from './connectTicket';
// @ts-ignore
import { RTCPeerConnection, RTCSessionDescription } from './rtcPeer';
import { isDesktop } from './desktopBridge';
import { isDemo, startDemo, demoRest } from './demo';
import { gait as gaitApi, RL_GAIT } from './gait';
import { DesktopWebrtcBridge } from './desktopWebrtc';
import { modeField, rendezvousEnabled, rendezvousUrl, robotId, resolveIceServers, type IceServer } from './remoteIce';
import { rendezvousExchange } from './rendezvousClient';
import type { MotionName, LogLevel } from '@/types/robot';

const SEND_HZ_MS = 25;
const ZERO_TAIL_FRAMES = 20;
const BACKOFF_MIN_MS = 500;
const BACKOFF_MAX_MS = 8000;
const STALE_MS = 12000;
const WATCHDOG_MS = 2500;
const VACANT_GRACE_MS = 3000;
const VACANT_JITTER_MS = 2000;
const LOG_LEVELS = new Set<LogLevel>(['TRACE', 'DEBUG', 'INFO', 'SUCCESS', 'WARNING', 'ERROR', 'FATAL']);

type Axes = { lx: number; ly: number; rx: number; ry: number };

class Connection {
  private pc: any = null;
  private webLogDc: any = null;
  private motionDc: any = null;
  private clientId = 'rbq-app-' + Math.random().toString(16).slice(2, 10);
  private ip = '';
  private wantConnected = false;
  private openGen = 0;
  private axes: Axes = { lx: 0, ly: 0, rx: 0, ry: 0 };
  private triggers = { l: 0, r: 0 };
  private buttons = new Uint8Array(16);
  private sendTimer: ReturnType<typeof setInterval> | null = null;
  private zeroTail = 0;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private commandDc: any = null;
  private estopDc: any = null;
  private extDcs = new Map<string, any>();
  private offerAuth: Record<string, string> = {};
  private cmdPending = new Map<string, {
    method: string; path: string; body: any;
    onOk: (b: any) => void; onErr: (e: HttpError) => void;
    createdMs: number; sentMs: number;
  }>();
  private cmdSeq = 0;
  private cmdTimer: ReturnType<typeof setInterval> | null = null;
  private motionInFlight = new Set<string>();
  private walkPutTimer: ReturnType<typeof setTimeout> | null = null;
  private tiltPutTimer: ReturnType<typeof setTimeout> | null = null;
  private lastRxAt = 0;
  private watchdogTimer: ReturnType<typeof setInterval> | null = null;
  private ownPollTimer: ReturnType<typeof setInterval> | null = null;
  private lastOwnCheckAt = 0;
  private lastOwnerPushAt = 0;
  lastTripPushAt = 0;
  autoClaim = true;
  private backoffMs = BACKOFF_MIN_MS;

  private detachTimer: ReturnType<typeof setTimeout> | null = null;
  private clearDetachTimer() {
    if (this.detachTimer) { clearTimeout(this.detachTimer); this.detachTimer = null; }
  }

  connect(ip: string) {
    this.clearDetachTimer();
    if (isDemo()) { startDemo({ onMessage: (d) => this.onMessage(d) }); return; }
    if (this.ip && this.ip !== ip) this.dropPendingWrites();
    this.ip = ip;
    this.wantConnected = true;
    useFeatures.getState().clearFeatures();
    useRobot.getState().setIp(ip);
    useRobot.getState().setConnError(null);
    rest.version(this.ip)
      .then((v: any) => {
        const m = /^(\d+)\.(\d+)/.exec(String(v?.version ?? ''));
        if (m) useTelemetry.getState().setSensorLayout(+m[1] * 100 + +m[2] < 119 ? 'legacy118' : 'current');
      })
      .catch(() => {});
    void this.open();
  }

  disconnect() {
    this.clearDetachTimer();
    this.wantConnected = false;
    this.clearReconnect();
    this.stopWatchdog();
    this.stopOwnPoll();
    this.clearVacantTimer();
    this.stopSendLoop();
    this.teardownPc();
    useFeatures.getState().clearFeatures();
    for (const k of robotKinds) k.reset();
    useRobot.getState().setConn('disconnected');
    noteMotionLink('disconnected');
  }

  stats() { return readStats(this.pc); }

  private teardownPc() {
    const old = this.pc;
    this.pc = null;
    if (old) { try { old.onconnectionstatechange = null; old.close(); } catch {} }
    this.clearInputs();
    this.webLogDc = null;
    this.motionDc = null;
    this.commandDc = null;
    this.estopDc = null;
    this.extDcs.clear();
  }

  private async open() {
    const gen = ++this.openGen;
    this.clearReconnect();
    useRobot.getState().setConn('connecting');
    noteMotionLink('connecting');
    this.teardownPc();
    this.lastRxAt = Date.now();
    let pc: any = null;
    try {
      const desktop = isDesktop();
      const iceServers: IceServer[] = desktop ? [] : await resolveIceServers();
      if (gen !== this.openGen || !this.wantConnected) return;
      pc = desktop
        ? new DesktopWebrtcBridge(robotKinds.map((k) => k.channel))
        : new RTCPeerConnection(iceServers.length ? { iceServers } : undefined);
      this.pc = pc;

      this.webLogDc = pc.createDataChannel('log');
      this.webLogDc.onmessage = (e: any) => this.onMessage(e.data);
      this.motionDc = pc.createDataChannel('motion-state', { ordered: false, maxRetransmits: 0 });
      try { this.motionDc.binaryType = 'arraybuffer'; } catch {}
      this.motionDc.onmessage = (e: any) => this.onMotionState(e.data);
      this.commandDc = pc.createDataChannel('command');
      this.commandDc.onopen = () => this.cmdFlush();
      this.commandDc.onmessage = (e: any) => this.onCommandResponse(e.data);
      this.estopDc = pc.createDataChannel('estop');
      for (const k of robotKinds) {
        k.reset();
        const dc = pc.createDataChannel(k.channel);
        dc.onmessage = (e: any) => { this.lastRxAt = Date.now(); k.onMessage(typeof e.data === 'string' ? e.data : String(e.data)); };
        this.extDcs.set(k.channel, dc);
      }

      pc.onconnectionstatechange = () => {
        if (this.pc !== pc) return;
        const st = pc.connectionState;
        if (st === 'connected') {
          this.backoffMs = BACKOFF_MIN_MS;
          this.establishedAt = Date.now();
          useRobot.getState().setConn('connected');
          setTimeout(() => {
            const p = this.pc;
            if (!p || typeof (p as any).getStats !== 'function') return;
            (p as any).getStats()
              .then((r: any) => {
                const arr = typeof r?.forEach === 'function' ? (() => { const a: any[] = []; r.forEach((v: any) => a.push(v)); return a; })() : [];
                const d = routeDetail(arr);
                useRobot.getState().setRoute(d.route, d.proto);
              })
              .catch(() => {});
          }, 1500);
          this.lastRxAt = Date.now();
          this.startWatchdog();
          this.startOwnPoll();
          this.fetchFeatures();
          if (this.autoClaim) this.claimIfFree(); else this.refreshOwnership();
        } else if (st === 'failed' || st === 'closed') {
          this.onDrop();
        }
      };

      if (desktop) {
        pc.connect();
      } else {
        await pc.setLocalDescription(await pc.createOffer());
        const sdp = await this.gatheredSdp(pc);
        const ans = await this.postOffer(sdp);
        if (this.pc !== pc) return;
        if (!ans?.sdp) throw new Error('offer answer에 sdp 없음');
        await pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: ans.sdp }));
      }
    } catch (e: any) {
      if (this.pc !== pc) return;
      if (/not allowed/.test(String(e?.message ?? ''))) {
        this.wantConnected = false;
        this.clearReconnect();
        useRobot.getState().setConn('disconnected');
        useRobot.getState().setConnError('이 로봇에 접속할 권한이 없습니다 — 코드가 만료됐거나, 관리자가 대시보드에서 배정을 해제했습니다.');
        return;
      }
      if (e?.message === 'NO_TICKET') {
        useRobot.getState().setConnError('원격 접속에는 접근 코드가 필요합니다 — 허브의 로봇 선택 ▾ 에서 [접근 코드로 로그인…]');
        this.onDrop();
        return;
      }
      if (e?.message === 'ROBOT_AUTH_REQUIRED') {
        if (!reportAuthFailure(undefined, false, this.offerAuth)) { this.onDrop(); return; }
        this.wantConnected = false;
        this.clearReconnect();
        useRobot.getState().setConn('disconnected');
        useRobot.getState().setConnError('로봇 비밀번호가 맞지 않습니다 — 로봇 비밀번호를 입력하세요.');
        return;
      }
      if (e?.message === 'INCOMPATIBLE_ROBOT') {
        this.wantConnected = false;
        useRobot.getState().setConn('disconnected');
        useRobot.getState().setConnError(
          '이 로봇의 펌웨어는 앱과 호환되지 않습니다 (구세대 프로토콜 — /api/webrtc/offer 없음). 로봇을 pkt 세대 펌웨어로 업데이트하거나 다른 로봇에 연결하세요.');
        return;
      }
      this.onDrop();
    }
  }

  private gatheredSdp(pc: any): Promise<string> {
    return new Promise((resolve) => {
      if (pc.iceGatheringState === 'complete') { resolve(pc.localDescription.sdp); return; }
      const check = () => {
        if (pc.iceGatheringState === 'complete') {
          pc.removeEventListener?.('icegatheringstatechange', check);
          resolve(pc.localDescription.sdp);
        }
      };
      pc.addEventListener?.('icegatheringstatechange', check);
      setTimeout(() => resolve(pc.localDescription ? pc.localDescription.sdp : ''), 2000);
    });
  }

  private async postOffer(sdp: string): Promise<{ sdp?: string }> {
    useRobot.getState().setVia(rendezvousEnabled() ? 'rendezvous' : 'direct');
    if (rendezvousEnabled()) {
      const ticket = await currentTicket(this.clientId);
      if (!ticket) throw new Error('NO_TICKET');
      const ans = await rendezvousExchange({
        url: rendezvousUrl(), robot: robotId(), service: 'motion',
        clientId: this.clientId, sdp,
        token: useSettings.getState().webrtcToken ?? '',
        ticket,
      });
      return { sdp: ans.sdp };
    }
    this.offerAuth = robotAuth();
    const r = await fetch(`${restBase(this.ip)}/api/webrtc/offer`, {
      method: 'POST',
      headers: { ...this.offerAuth, 'Content-Type': 'application/json' },
      body: JSON.stringify({
        sdp,
        takeover: true,
        clientId: this.clientId,
        token: useSettings.getState().webrtcToken ?? '',
        ...modeField(),
      }),
    });
    if (r.status === 404 || r.status === 405) throw new Error('INCOMPATIBLE_ROBOT');
    if (r.status === 401) throw new Error('ROBOT_AUTH_REQUIRED');
    if (!r.ok) throw new Error('offer ' + r.status);
    return r.json();
  }

  private establishedAt = 0;
  private dropHistory: number[] = [];

  private onDrop() {
    this.stopWatchdog();
    this.stopOwnPoll();
    this.clearVacantTimer();
    this.stopSendLoop();
    this.teardownPc();
    useFeatures.getState().clearFeatures();
    if (this.establishedAt) {
      this.establishedAt = 0;
      const now = Date.now();
      this.dropHistory = this.dropHistory.filter((t) => now - t < 30000);
      this.dropHistory.push(now);
      if (this.dropHistory.length >= 3 && this.wantConnected) {
        this.dropHistory = [];
        this.wantConnected = false;
        this.clearReconnect();
        useRobot.getState().setConn('disconnected');
        useRobot.getState().setTakeoverConflict(true);
        return;
      }
    }
    useRobot.getState().setConn(this.wantConnected ? 'connecting' : 'disconnected');
    noteMotionLink(this.wantConnected ? 'connecting' : 'disconnected');
    if (this.wantConnected && !this.reconnectTimer) {
      this.reconnectTimer = setTimeout(() => { this.reconnectTimer = null; void this.open(); }, this.backoffMs);
      this.backoffMs = Math.min(this.backoffMs * 2, BACKOFF_MAX_MS);
    }
  }

  private clearReconnect() {
    if (this.reconnectTimer) { clearTimeout(this.reconnectTimer); this.reconnectTimer = null; }
  }

  private startWatchdog() {
    if (this.watchdogTimer) return;
    this.watchdogTimer = setInterval(() => {
      if (this.pc && Date.now() - this.lastRxAt > STALE_MS) this.onDrop();
    }, WATCHDOG_MS);
  }

  private stopWatchdog() {
    if (this.watchdogTimer) { clearInterval(this.watchdogTimer); this.watchdogTimer = null; }
  }

  private startOwnPoll() {
    if (this.ownPollTimer) return;
    this.ownPollTimer = setInterval(() => {
      if (!this.pc) return;
      const now = Date.now();
      if (now - this.lastOwnerPushAt > 3000 && now - this.lastOwnCheckAt > 1000) {
        this.lastOwnCheckAt = now;
        this.refreshOwnership();
      }
    }, 1000);
  }

  private stopOwnPoll() {
    if (this.ownPollTimer) { clearInterval(this.ownPollTimer); this.ownPollTimer = null; }
  }

  private onMessage(data: any) {
    this.lastRxAt = Date.now();
    let obj: any;
    try { obj = typeof data === 'string' ? JSON.parse(data) : JSON.parse(String(data)); }
    catch { return; }
    const s = useRobot.getState();
    const t = obj?.t;
    if (simEngine.active && t === 'robot_status') return;
    switch (t) {
      case 'robot_status': s.applyRobot(obj); break;
      case 'pc_status': s.applyPc(obj); break;
      case 'trip':
        this.lastTripPushAt = Date.now();
        s.applyTrip(obj);
        break;
      case 'gamepad_status':
        this.lastOwnerPushAt = Date.now();
        s.setOwner(obj.owner ?? '');
        this.claimIfVacant(obj.owner ?? '');
        break;
      default:
        if (t === undefined && obj && typeof obj === 'object') {
          const lv = String(obj.level || 'INFO').toUpperCase() as LogLevel;
          s.pushLog({
            ts: typeof obj.timestamp === 'string' ? obj.timestamp.slice(11, 23) : '',
            process: obj.application || '',
            level: LOG_LEVELS.has(lv) ? lv : 'INFO',
            msg: obj.message ?? '',
          });
        }
    }
  }

  dcCommand(method: string, path: string, body: object = {}): Promise<any> {
    const sim = simEngine.intercept(method, path, body as Record<string, unknown>);
    if (sim) return Promise.resolve(sim.body);
    if (isDemo()) { try { return Promise.resolve(demoRest(path, method, body)); } catch (e) { return Promise.reject(e); } }
    return new Promise((resolve, reject) => {
      const id = 'c-' + (++this.cmdSeq) + '-' + Math.random().toString(16).slice(2, 6);
      const now = Date.now();
      this.cmdPending.set(id, {
        method, path, body: { ...body, msg_id: id },
        onOk: resolve, onErr: reject, createdMs: now, sentMs: 0,
      });
      if (!this.cmdTimer) this.cmdTimer = setInterval(() => this.cmdTick(), 500);
      this.cmdTick();
    });
  }

  private cmdEnvelope(id: string) {
    const p = this.cmdPending.get(id)!;
    return JSON.stringify({ id, method: p.method, path: p.path, body: p.body });
  }

  private cmdSend(id: string): boolean {
    if (!this.commandDc || this.commandDc.readyState !== 'open') return false;
    if (simEngine.active && (this.cmdPending.get(id)?.body as { cmd?: string } | undefined)?.cmd !== 'estop') return false;
    try { this.commandDc.send(this.cmdEnvelope(id)); } catch { return false; }
    return true;
  }

  private cmdTick() {
    if (this.cmdPending.size === 0) return;
    const now = Date.now();
    for (const [id, p] of Array.from(this.cmdPending)) {
      if (now - p.createdMs > 8000) {
        this.cmdPending.delete(id);
        p.onErr(new HttpError(0, `${p.path} — command 응답 없음(8s)`));
        continue;
      }
      if (now - p.sentMs >= 1000 && this.cmdSend(id)) p.sentMs = now;
    }
  }

  private dropPendingWrites() {
    for (const [id, p] of Array.from(this.cmdPending)) {
      if (p.method === 'GET') continue;
      this.cmdPending.delete(id);
      p.onErr(new HttpError(0, `${p.path} — 다른 로봇으로 바뀌어 보내지 않음`));
    }
  }

  private cmdFlush() {
    const now = Date.now();
    for (const id of this.cmdPending.keys()) {
      if (this.cmdSend(id)) this.cmdPending.get(id)!.sentMs = now;
    }
  }

  private onCommandResponse(data: any) {
    let env: any;
    try { env = JSON.parse(String(data)); } catch { return; }
    const p = env && this.cmdPending.get(env.id);
    if (!p) return;
    this.cmdPending.delete(env.id);
    if (env.status < 400) p.onOk(env.body ?? {});
    else {
      const why = env.body?.reason || env.body?.error;
      p.onErr(new HttpError(env.status, `${p.path} → ${env.status}${why ? ` (${why})` : ''}`));
    }
  }

  private getOwnership(): Promise<any> {
    return rendezvousEnabled()
      ? this.dcCommand('GET', '/api/gamepad/ownership')
      : rest.getOwnership(this.ip);
  }

  async refreshOwnership() {
    try {
      const d = await this.getOwnership();
      useRobot.getState().setOwnership({ owner: d.ownerIP || '', myIp: d.requesterIP || '', isMine: !!d.IsOwner });
      this.claimIfVacant(d.ownerIP || '');
    } catch {}
  }

  private vacantTimer: ReturnType<typeof setTimeout> | null = null;
  private claimIfVacant(owner: string) {
    if (owner) { this.clearVacantTimer(); return; }
    if (!this.autoClaim || !this.pc || !this.wantConnected || this.vacantTimer) return;
    const r = useRobot.getState();
    if (r.isMine || !r.myIp) return;
    const delay = VACANT_GRACE_MS + Math.random() * VACANT_JITTER_MS;
    this.vacantTimer = setTimeout(async () => {
      this.vacantTimer = null;
      if (!this.autoClaim || !this.pc || !this.wantConnected) return;
      const s = useRobot.getState();
      if (s.isMine || s.owner) return;
      try {
        const d: any = await this.getOwnership();
        if (d?.ownerIP) { useRobot.getState().setOwnership({ owner: d.ownerIP, myIp: d.requesterIP || '', isMine: !!d.IsOwner }); return; }
      } catch { return; }
      if (!this.pc || !this.wantConnected || useRobot.getState().isMine) return;
      this.claimOwnership().catch(() => {});
    }, delay);
  }
  private clearVacantTimer() {
    if (this.vacantTimer) { clearTimeout(this.vacantTimer); this.vacantTimer = null; }
  }

  private async claimIfFree() {
    try {
      const d: any = await this.getOwnership();
      const owner = d?.ownerIP || '';
      useRobot.getState().setOwnership({ owner, myIp: d?.requesterIP || '', isMine: !!d?.IsOwner });
      if (!owner || d?.IsOwner) await this.claimOwnership();
    } catch {
      if (rendezvousEnabled()) console.warn('[conn] 소유권 조회 실패 — 랑데부에선 자동 claim 생략');
      else this.claimOwnership().catch(() => {});
    }
  }

  extSend(channel: string, msg: object): boolean {
    const dc = this.extDcs.get(channel);
    if (!dc || dc.readyState !== 'open') return false;
    try { dc.send(JSON.stringify(msg)); return true; } catch { return false; }
  }

  async claimOwnership() {
    this.autoClaim = true;
    try {
      const d = await this.dcCommand('POST', '/api/gamepad/ownership');
      useRobot.getState().setOwnership({ owner: d.ownerIP || '', myIp: d.requesterIP || '', isMine: !!d.IsOwner });
      return !!d.IsOwner;
    } catch {
      return false;
    }
  }

  private fetchFeatures() {
    rest.robotFeatures(this.ip)
      .then((r) => {
        const f = r?.features;
        if (!f) return;
        useFeatures.getState().setFeatures(f);
        const v = autoRobotVersion(f);
        if (v) useSettings.getState().setRobotVersion(v.key);
      })
      .catch(() => {});
  }

  setAxes(side: 'L' | 'R', nx: number, ny: number) {
    if (side === 'L') { this.axes.lx = nx; this.axes.ly = ny; }
    else { this.axes.rx = nx; this.axes.ry = ny; }
    if (!this.allZero()) this.startSendLoop();
  }

  setButtons(b16: Uint8Array) {
    this.buttons.set(b16.subarray(0, 16));
    if (!this.allZero()) this.startSendLoop();
  }

  setTriggers(l: number, r: number) {
    this.triggers.l = l;
    this.triggers.r = r;
    if (!this.allZero()) this.startSendLoop();
  }

  private allZero() {
    const { lx, ly, rx, ry } = this.axes;
    return lx === 0 && ly === 0 && rx === 0 && ry === 0
      && this.triggers.l === 0 && this.triggers.r === 0
      && this.buttons.every((v) => v === 0);
  }

  private startSendLoop() {
    if (this.sendTimer) return;
    this.sendTimer = setInterval(() => this.tickSend(), this.sendIntervalMs);
  }

  private sendIntervalMs = SEND_HZ_MS;
  setPublishHz(hz: number) {
    const ms = Math.round(1000 / Math.min(100, Math.max(10, hz)));
    if (ms === this.sendIntervalMs) return;
    this.sendIntervalMs = ms;
    if (this.sendTimer) { this.stopSendLoop(); this.startSendLoop(); }
  }

  clearInputs() {
    this.axes = { lx: 0, ly: 0, rx: 0, ry: 0 };
    this.triggers = { l: 0, r: 0 };
    this.buttons = new Uint8Array(16);
  }

  detachForSim(): string | null {
    if (isDemo()) return null;
    if (!this.wantConnected) return null;
    const ip = this.ip;
    this.clearInputs();
    if (this.motionDc && this.motionDc.readyState === 'open') {
      const buf = new ArrayBuffer(40);
      try { this.motionDc.send(buf); } catch {}
    }
    this.wantConnected = false;
    this.clearReconnect();
    this.detachTimer = setTimeout(() => { this.detachTimer = null; this.disconnect(); }, 150);
    return ip;
  }

  private stopSendLoop() {
    if (this.sendTimer) { clearInterval(this.sendTimer); this.sendTimer = null; }
    this.zeroTail = 0;
  }

  private tickSend() {
    if (simEngine.active) {
      simEngine.setCommand(this.axes.ly, -this.axes.lx, this.axes.rx, this.axes.ry);
      if (this.allZero()) this.stopSendLoop();
      return;
    }
    const kind = activeRobotKind();
    if (kind) {
      const dc = this.extDcs.get(kind.channel);
      if (!dc || dc.readyState !== 'open' || (dc.bufferedAmount ?? 0) > 16 * 1024) return;
      try { dc.send(this.joyFrame()); } catch {}
      this.tailOrStop();
      return;
    }
    if (!this.motionDc || this.motionDc.readyState !== 'open') return;
    const now = Date.now();
    if (now - this.lastOwnerPushAt > 3000 && now - this.lastOwnCheckAt > 1000) {
      this.lastOwnCheckAt = now;
      this.refreshOwnership();
    }
    const st = useRobot.getState();
    if (st.owner && st.myIp && st.owner !== st.myIp) {
      return;
    }
    if ((this.motionDc.bufferedAmount ?? 0) > 16 * 1024) return;
    try { this.motionDc.send(this.joyFrame()); } catch {}
    this.tailOrStop();
  }

  private tailOrStop() {
    if (this.allZero()) {
      if (++this.zeroTail >= ZERO_TAIL_FRAMES) this.stopSendLoop();
    } else {
      this.zeroTail = 0;
    }
  }

  private joyFrame(): ArrayBuffer {
    const { lx, ly, rx, ry } = this.axes;
    const buf = new ArrayBuffer(40);
    const dv = new DataView(buf);
    dv.setFloat32(0, lx, true);
    dv.setFloat32(4, ly, true);
    dv.setFloat32(8, rx, true);
    dv.setFloat32(12, ry, true);
    dv.setFloat32(16, this.triggers.l, true);
    dv.setFloat32(20, this.triggers.r, true);
    new Uint8Array(buf, 24, 16).set(this.buttons);
    return buf;
  }

  private onMotionState(data: any) {
    this.lastRxAt = Date.now();
    if (!(data instanceof ArrayBuffer) || data.byteLength < 3) return;
    const u = new Uint8Array(data);
    if (u[0] !== 0x4d || u[1] !== 0x53) return;
    noteMotionLink('connected');
    const id = u[2], body = 3;
    const tel = useTelemetry.getState();
    try {
      switch (id) {
        case 1: {
          if (data.byteLength < body + SIZEOF.ROBOT_STATE) return;
          if (simEngine.active) return;
          tel.applyRobotState(parseRobotState(data, body));
          break;
        }
        case 7:
          if (data.byteLength >= body + SIZEOF.PDU_STATE) tel.applyPduState(parsePduState(data, body));
          break;
        case 22:
          if (data.byteLength >= body + SIZEOF.DEVICE_STATES) tel.applyDeviceStates(parseDeviceStates(data, body));
          break;
        default: {
          const h = motionFrames.get(id);
          if (h && data.byteLength >= body + h.size) h.onFrame(data, body);
        }
      }
    } catch {}
  }

  private newMsgId() {
    return 'msg_' + Math.random().toString(16).slice(2, 10);
  }

  private surface(level: 'ERROR' | 'WARNING', msg: string) {
    const d = new Date();
    const p = (n: number, w = 2) => String(n).padStart(w, '0');
    useRobot.getState().pushLog({
      ts: `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${p(d.getMilliseconds(), 3)}`,
      process: 'App', level, msg,
    });
  }

  private sendEstop() {
    const kind = activeRobotKind();
    if (kind) {
      if (!kind.estop.send()) this.surface('ERROR', 'E-Stop 전송 실패 — 채널이 닫혀 있습니다. 물리 비상정지를 사용하세요');
      return;
    }
    let sent = false;
    if (this.estopDc && this.estopDc.readyState === 'open') {
      const eid = this.newMsgId();
      try {
        this.estopDc.send(JSON.stringify({ id: eid, method: 'POST', path: '/api/motion/command', body: { msg_id: eid, cmd: 'estop' } }));
        sent = true;
      } catch {}
    }
    if (simEngine.active && !this.wantConnected) {
      if (sent) useVisionToggles.getState().setDockOverride(true);
      return;
    }
    useVisionToggles.getState().setDockOverride(true);
    this.dcCommand('POST', '/api/motion/command', { cmd: 'estop' })
      .then(() => this.refreshOwnership())
      .catch(() => this.surface('ERROR', 'E-Stop 전송 실패 — 응답 없음. 물리 E-Stop을 사용하세요'));
  }

  sendMotion(name: MotionName, rawWalk = false) {
    if (name === 'estop') {
      this.sendEstop();
      simEngine.active && simEngine.motion(name);
      return;
    }
    if (simEngine.active && simEngine.motion(name)) return;
    useVisionToggles.getState().setDockOverride(name !== 'dock');
    if (name === 'dock') useDockView.getState().show();
    if (name === 'walk' && !rawWalk) {
      if (this.motionInFlight.has(name)) return;
      this.motionInFlight.add(name);
      const id = useSettings.getState().visionWalkEnabled ? RL_GAIT.RL_WALK_VISION : RL_GAIT.RL_WALK;
      gaitApi.aiWalk(this.ip, id)
        .then(() => { this.motionInFlight.delete(name); this.refreshOwnership(); })
        .catch((e) => {
          this.motionInFlight.delete(name);
          const st = (e as HttpError).status;
          if (st === 403) this.refreshOwnership();
          this.surface('ERROR', `모션 'walk' 실패 — ${st ? `HTTP ${st}${st === 403 ? ' (제어권 없음)' : ''}` : '응답 없음'}`);
        });
      return;
    }
    if (this.motionInFlight.has(name)) return;
    this.motionInFlight.add(name);
    this.dcCommand('POST', '/api/motion/command', { cmd: name })
      .then(() => { this.motionInFlight.delete(name); this.refreshOwnership(); })
      .catch((e) => {
        this.motionInFlight.delete(name);
        const st = (e as HttpError).status;
        if (st === 403) this.refreshOwnership();
        this.surface('ERROR', `모션 '${name}' 실패 — ${st ? `HTTP ${st}${st === 403 ? ' (제어권 없음)' : ''}` : '응답 없음'}`);
      });
  }

  putWalkParams(p: { body_height: number; max_speed: number; foot_height: number }) {
    if (this.walkPutTimer) clearTimeout(this.walkPutTimer);
    this.walkPutTimer = setTimeout(() => { actions.walkParams(this.ip, p).catch(() => {}); }, 200);
  }

  putBodyTilt(deg: number, walk: { body_height: number; max_speed: number }) {
    if (this.tiltPutTimer) clearTimeout(this.tiltPutTimer);
    this.tiltPutTimer = setTimeout(() => { actions.setBodyTilt(this.ip, deg, walk).catch(() => {}); }, 200);
  }
}

export const connection = new Connection();
setCommandSender((method, path, body) => connection.dcCommand(method, path, body ?? {}));
(globalThis as any).__rbqConn = connection;
