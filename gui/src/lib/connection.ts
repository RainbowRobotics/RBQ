import { useRobot } from '@/store/robot';
import { useTelemetry, noteMotionLink, noteJoyUnstable } from '@/store/telemetry';
import { sensorLayoutFor } from './sensorState';
import { useSettings } from '@/store/settings';
import { useFeatures } from '@/store/capability';
import { simEngine } from '@/lib/simEngine';
import { useDockView } from '@/store/dockView';
import { useVisionToggles } from '@/store/visionToggles';
import { parseRobotState, parseDeviceStates, parsePduState, SIZEOF, CRUISE_GAIT_IDS } from './robotState';
import { activeRobotKind, autoRobotVersion, connRejectReason, motionFrames, robotKinds, remote, remoteActive } from '@/modules/registry';
import { rest, actions, HttpError } from './rest';
import { setCommandSender, type CommandOptions } from './commandBus';
import { restBase } from './endpoints';
import { resolveEndpoints } from './resolveEndpoints';
import { secureRobot, reportTlsIssue, type TlsPlatform, type TlsIssue } from './robotTls';
import { robotTlsPlatform } from './robotTlsPlatform';
import { attachTunnel } from './robotTunnel';
import { robotAuth, reportAuthFailure } from './auth';
import { routeDetail, readStats } from './connectionRoute';
// @ts-ignore
import { RTCPeerConnection, RTCSessionDescription } from './rtcPeer';
import { isDemo, demoDriver } from './demoFlag';
import { gait as gaitApi, RL_GAIT } from './gait';
import { modeField, resolveIceServers, type IceServer } from './remoteIce';
import { installId, installIdReady } from './installId';
import { ID_JOY_LINK, JOY_LINK_STATS_BYTES, JoyLinkHealth, monoMs, parseJoyLinkStats, withJoyTail, type JoyLinkStats } from './joyLink';
import { t } from './i18n';
import type { MotionName, LogLevel } from '@/types/robot';

const SEND_HZ_MS = 25;
const ZERO_TAIL_FRAMES = 20;
const CRUISE_KEEPALIVE_TICKS = 8;
const JOY_MAX_BUFFERED = 150;
const JOY_SEED_FRAMES = 3;
const BACKOFF_MIN_MS = 500;
const BACKOFF_MAX_MS = 8000;
const STALE_MS = 12000;
const WATCHDOG_MS = 2500;
const STATE_LAG_MS = 3000;
const NEGOTIATE_MS = 20000;
const NEGOTIATE_REMOTE_MS = 25000;
const LIVENESS_MS = 5000;
const LIVENESS_GAP_MS = 25000;
const HANGUP_MAX_MS = 1500;
const NOT_SENT = '연결이 없어 보내지 않았습니다';
const TLS_PORTS = [8080, 8081, 8082] as const;
const FAIL_REJUDGE_MS = 30_000;
const isEstop = (body: unknown) => (body as { cmd?: string } | undefined)?.cmd === 'estop';
const LOG_LEVELS = new Set<LogLevel>(['TRACE', 'DEBUG', 'INFO', 'SUCCESS', 'WARNING', 'ERROR', 'FATAL']);

type Axes = { lx: number; ly: number; rx: number; ry: number };

class Connection {
  private pc: any = null;
  private webLogDc: any = null;
  private motionDc: any = null;
  private get clientId() { return 'rbq-app-' + installId(); }
  private ip = '';
  private wantConnected = false;
  private openGen = 0;
  private negoTimer: ReturnType<typeof setTimeout> | null = null;
  private offerAbort: AbortController | null = null;
  private hangupDone: Promise<void> = Promise.resolve();
  private tls: Promise<'ok' | 'refused'> = Promise.resolve('ok');
  private tlsGen = 0;
  private tlsHosts: string[] = [];
  private suspended = false;
  private established = false;
  private failRejudgeAt = new Map<string, number>();
  private liveness = false;
  private aliveTimer: ReturnType<typeof setInterval> | null = null;
  private axes: Axes = { lx: 0, ly: 0, rx: 0, ry: 0 };
  private triggers = { l: 0, r: 0 };
  private buttons = new Uint8Array(16);
  private sendTimer: ReturnType<typeof setInterval> | null = null;
  private joyV2 = false;
  private joySeq = 0;
  joyLink: JoyLinkStats | null = null;
  private joyHealth = new JoyLinkHealth();
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
  private lastStateAt = 0;
  private lastRxAt = 0;
  private watchdogTimer: ReturnType<typeof setInterval> | null = null;
  private ownPollTimer: ReturnType<typeof setInterval> | null = null;
  private lastOwnCheckAt = 0;
  private lastOwnerPushAt = 0;
  lastTripPushAt = 0;
  private backoffMs = BACKOFF_MIN_MS;

  private detachTimer: ReturnType<typeof setTimeout> | null = null;
  private clearDetachTimer() {
    if (this.detachTimer) { clearTimeout(this.detachTimer); this.detachTimer = null; }
  }

  connect(ip: string) {
    this.clearDetachTimer();
    const demo = demoDriver();
    if (demo) { demo.start({ onMessage: (d) => this.onMessage(d) }); return; }
    if (this.ip && this.ip !== ip) this.rejectPending((p) => p.method !== 'GET', '다른 로봇으로 바뀌어 보내지 않음');
    this.ip = ip;
    this.wantConnected = true;
    this.established = false;
    useFeatures.getState().clearFeatures();
    useRobot.getState().setIp(ip);
    useRobot.getState().setConnError(null);
    void this.open();
    void this.secured(ip).then((ok) => (ok ? rest.version(ip) : undefined))
      .then((v: any) => { if (v) useTelemetry.getState().setSensorLayout(sensorLayoutFor(v.version)); })
      .catch(() => {});
  }

  async secured(host: string): Promise<boolean> {
    for (;;) {
      const t = this.tls;
      await t.then(() => {}, () => {});
      if (t === this.tls) return this.tlsHosts.includes(host);
    }
  }

  private blockAll(ip: string, vision: string) {
    for (const port of TLS_PORTS) robotTlsPlatform.block(port === 8080 ? ip : vision, port);
  }

  private dropJudgement() {
    this.tlsGen++;
    this.tls = Promise.resolve('ok');
    this.tlsHosts = [];
  }

  private secure(): Promise<'ok' | 'refused'> {
    const ip = this.ip;
    const vision = resolveEndpoints(ip, useRobot.getState().visionIp).vision;
    const gen = ++this.tlsGen;
    const live = () => gen === this.tlsGen;
    this.tlsHosts = [ip, vision];
    const p: TlsPlatform = {
      probe: (h, port) => robotTlsPlatform.probe(h, port),
      engage: async (h, port, fp) => {
        if (!live()) return;
        if (this.suspended) { robotTlsPlatform.block(h, port); return; }
        await robotTlsPlatform.engage(h, port, fp);
        if (this.suspended && live()) robotTlsPlatform.block(h, port);
      },
      block: (h, port) => { if (live()) robotTlsPlatform.block(h, port); },
      kept: (h, port) => robotTlsPlatform.kept(h, port),
      pinnedFp: (h, port) => robotTlsPlatform.pinnedFp(h, port),
    };
    const report = (i: TlsIssue) => { if (live() && !robotTlsPlatform.plainHeld()) reportTlsIssue(i); };
    const run: Promise<'ok' | 'refused'> = secureRobot(ip, vision, p, report, { live: this.established }).then(
      ({ result }) => {
        if (!live()) return this.tls;
        if (result === 'refused') this.blockAll(ip, vision);
        return result === 'refused' && robotTlsPlatform.plainHeld() ? 'ok' : result;
      },
      (e) => {
        if (!live()) return this.tls;
        this.blockAll(ip, vision);
        throw e;
      },
    );
    run.catch(() => {});
    this.tls = run;
    return run;
  }

  resecure() {
    this.suspended = false;
    if (!this.wantConnected || !this.ip || remoteActive()) return;
    const ip = this.ip;
    this.secure().then((r) => { if (r === 'refused' && this.ip === ip) this.disconnect(); }, () => {});
  }

  resecureFor(base: string) {
    const m = /^(?:https:\/\/([^/:]+)|http:\/\/(127\.0\.0\.1):)/.exec(base);
    if (m) this.resecureAfterFailure(m[1] ?? m[2]);
  }

  resecureAfterFailure(host: string) {
    if (this.suspended || !this.wantConnected || !this.ip || remoteActive()) return;
    const now = Date.now();
    const age = now - (this.failRejudgeAt.get(host) ?? -Infinity);
    if (age >= 0 && age < FAIL_REJUDGE_MS) return;
    this.failRejudgeAt.set(host, now);
    this.resecure();
  }

  suspendRelays() {
    this.suspended = true;
    if (!this.ip || remoteActive()) return;
    const vision = resolveEndpoints(this.ip, useRobot.getState().visionIp).vision;
    for (const port of TLS_PORTS) {
      const host = port === 8080 ? this.ip : vision;
      if (robotTlsPlatform.base(host, port)?.startsWith('http://127.0.0.1:')) robotTlsPlatform.block(host, port);
    }
  }

  disconnect() {
    this.clearDetachTimer();
    this.wantConnected = false;
    this.established = false;
    this.openGen++;
    this.dropJudgement();
    this.failRejudgeAt.clear();
    this.clearReconnect();
    this.stopWatchdog();
    this.stopOwnPoll();
    this.stopSendLoop();
    const hadPc = !!this.pc;
    this.teardownPc();
    if (hadPc) this.hangup();
    this.rejectPending((p) => p.method !== 'GET', '연결 해제로 보내지 않음');
    if (this.walkPutTimer) { clearTimeout(this.walkPutTimer); this.walkPutTimer = null; }
    if (this.tiltPutTimer) { clearTimeout(this.tiltPutTimer); this.tiltPutTimer = null; }
    this.motionInFlight.clear();
    useRobot.getState().setOwnership({ owner: '', myIp: '', isMine: false });
    useFeatures.getState().clearFeatures();
    for (const k of robotKinds) k.reset();
    useRobot.getState().applyHealth(undefined);
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
    attachTunnel(null);
    this.extDcs.clear();
    this.clearNego();
    this.offerAbort?.abort();
    this.offerAbort = null;
    this.stopAlive();
    this.rejectPending((p) => p.method !== 'GET' && !isEstop(p.body), '연결이 끊겨 보내지 않음');
  }

  private clearNego() {
    if (this.negoTimer) { clearTimeout(this.negoTimer); this.negoTimer = null; }
  }

  private hangup() {
    if (!this.ip || remoteActive()) return;
    const ac = new AbortController();
    const timer = setTimeout(() => ac.abort(), HANGUP_MAX_MS);
    try {
      this.hangupDone = fetch(`${restBase(this.ip)}/api/webrtc/hangup`, {
        method: 'POST', signal: ac.signal,
        headers: { ...robotAuth(), 'Content-Type': 'application/json' },
        body: JSON.stringify({ clientId: this.clientId, token: useSettings.getState().webrtcToken ?? '' }),
      }).then(() => {}, () => {}).finally(() => clearTimeout(timer));
    } catch { clearTimeout(timer); }
  }

  private startAlive() {
    if (!this.liveness || this.aliveTimer) return;
    let last = monoMs();
    const beat = () => {
      const now = monoMs();
      if (now - last > LIVENESS_GAP_MS) { this.onDrop(); return; }
      last = now;
      if (this.commandDc?.readyState !== 'open') return;
      try { this.commandDc.send('{"t":"alive"}'); } catch {}
    };
    beat();
    this.aliveTimer = setInterval(beat, LIVENESS_MS);
  }

  private stopAlive() {
    if (this.aliveTimer) { clearInterval(this.aliveTimer); this.aliveTimer = null; }
  }

  private async open() {
    const gen = ++this.openGen;
    this.clearReconnect();
    useRobot.getState().setConn('connecting');
    noteMotionLink('connecting');
    this.teardownPc();
    this.negoTimer = setTimeout(() => {
      this.negoTimer = null;
      if (gen === this.openGen && this.wantConnected) this.onDrop();
    }, remoteActive() ? NEGOTIATE_REMOTE_MS : NEGOTIATE_MS);
    const offerAbort = new AbortController();
    this.offerAbort = offerAbort;
    this.liveness = false;
    this.lastRxAt = Date.now();
    this.lastStateAt = 0;
    this.joyV2 = false;
    this.joyLink = null;
    this.joyHealth = new JoyLinkHealth();
    const tls = remoteActive() ? null : this.secure();
    if (!tls) this.dropJudgement();
    let pc: any = null;
    try {
      const iceServers: IceServer[] = await resolveIceServers();
      if (gen !== this.openGen || !this.wantConnected) return;
      pc = new RTCPeerConnection(iceServers.length ? { iceServers } : undefined);
      this.pc = pc;

      this.webLogDc = pc.createDataChannel('log');
      this.webLogDc.onmessage = (e: any) => this.onMessage(e.data);
      this.motionDc = pc.createDataChannel('motion-state', { ordered: false, maxRetransmits: 0 });
      try { this.motionDc.binaryType = 'arraybuffer'; } catch {}
      this.motionDc.onmessage = (e: any) => this.onMotionState(e.data);
      this.commandDc = pc.createDataChannel('command');
      this.commandDc.onopen = () => { this.cmdFlush(); this.startAlive(); };
      this.commandDc.onmessage = (e: any) => this.onCommandResponse(e.data);
      this.commandDc.onclose = () => { if(this.pc===pc)this.onDrop(); };
      this.estopDc = pc.createDataChannel('estop');
      attachTunnel(pc.createDataChannel('tunnel'), () => { this.lastRxAt = Date.now(); });
      useRobot.getState().applyHealth(undefined);
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
          this.clearNego();
          this.established = true;
          useRobot.getState().setConn('connected');
          useRobot.getState().setConnError(null);
          setTimeout(() => {
            const p = this.pc;
            if (gen !== this.openGen || !p || typeof (p as any).getStats !== 'function') return;
            (p as any).getStats()
              .then((r: any) => {
                if (gen !== this.openGen) return;
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
          this.refreshOwnership();
        } else if (st === 'failed' || st === 'closed') {
          this.onDrop();
        }
      };

      await pc.setLocalDescription(await pc.createOffer());
      const sdp = await this.gatheredSdp(pc);
      if (tls && (await tls) === 'refused') throw new Error('TLS_REFUSED');
      if (this.pc !== pc || gen !== this.openGen) return;
      const ans = await this.postOffer(sdp, offerAbort.signal);
      if (this.pc !== pc || gen !== this.openGen) return;
      if (!ans?.sdp) throw new Error('offer answer에 sdp 없음');
      this.liveness = Array.isArray(ans.features) && ans.features.includes('liveness');
      await pc.setRemoteDescription(new RTCSessionDescription({ type: 'answer', sdp: ans.sdp }));
    } catch (e: any) {
      if (this.pc !== pc) return;
      if (/not allowed|remote check is not open/.test(String(e?.message ?? ''))) {
        this.wantConnected = false;
        this.clearReconnect();
        useRobot.getState().setConn('disconnected');
        useRobot.getState().setConnError(connRejectReason()
          ?? '이 로봇에 접속할 권한이 없습니다 — 코드가 만료됐거나, 관리자가 대시보드에서 배정을 해제했습니다.');
        return;
      }
      if (e?.message === 'NO_TICKET') {
        useRobot.getState().setConnError(connRejectReason() ?? remote.net?.noPassText() ?? null);
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
      if (e?.message === 'TLS_REFUSED') {
        this.wantConnected = false;
        useRobot.getState().setConn('disconnected');
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

  private async postOffer(sdp: string, signal: AbortSignal): Promise<{ sdp?: string; features?: string[] }> {
    await installIdReady;
    const net = remoteActive() ? remote.net! : null;
    useRobot.getState().setVia(net ? 'remote' : 'direct');
    if (net) {
      const ticket = await net.pass(this.clientId);
      if (!ticket) throw new Error('NO_TICKET');
      if (signal.aborted) throw new Error('offer aborted');
      return net.exchange('motion', {
        clientId: this.clientId, sdp,
        token: useSettings.getState().webrtcToken ?? '',
        pass: ticket,
      });
    }
    await this.hangupDone;
    this.offerAuth = robotAuth();
    const r = await fetch(`${restBase(this.ip)}/api/webrtc/offer`, {
      method: 'POST', signal,
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

  private onDrop() {
    this.openGen++;
    this.stopWatchdog();
    this.stopOwnPoll();
    this.stopSendLoop();
    this.teardownPc();
    useFeatures.getState().clearFeatures();
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
      else if (this.pc && this.lastStateAt && Date.now() - this.lastStateAt > STATE_LAG_MS) noteMotionLink('connecting');
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
    if (simEngine.active && (t === 'robot_status' || t === 'robot_health')) return;
    switch (t) {
      case 'robot_status': {
        const wasRunning = !!s.robot?.autostart?.running;
        s.applyRobot(obj);
        if (wasRunning && !obj?.autostart?.running) this.fetchFeatures();
        break;
      }
      case 'robot_health': s.applyHealth({ overall: obj.overall, findings: obj.findings ?? [] }); break;
      case 'pc_status': s.applyPc(obj); break;
      case 'trip':
        this.lastTripPushAt = Date.now();
        s.applyTrip(obj);
        break;
      case 'gamepad_status':
        this.lastOwnerPushAt = Date.now();
        s.setOwner(obj.owner ?? '');
        s.setOwnerSupport(!!obj.ownerSupport);
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

  dcCommand(method: string, path: string, body: object = {}, options?: CommandOptions): Promise<any> {
    const signal = options?.signal;
    const aborted = () => Object.assign(new Error(`${path} — 조작 취소`), { name: 'AbortError' });
    if (signal?.aborted) return Promise.reject(aborted());
    const sim = simEngine.intercept(method, path, body as Record<string, unknown>);
    if (sim) return Promise.resolve(sim.body);
    const demo = demoDriver();
    if (demo) { try { return Promise.resolve(demo.rest(path, method, body)); } catch (e) { return Promise.reject(e); } }
    if (method !== 'GET' && !isEstop(body) && this.commandDc?.readyState !== 'open') {
      this.surface('ERROR', t(NOT_SENT));
      return Promise.reject(new HttpError(0, `${path} — ${NOT_SENT}`));
    }
    return new Promise((resolve, reject) => {
      const id = 'c-' + (++this.cmdSeq) + '-' + Math.random().toString(16).slice(2, 6);
      const now = Date.now();
      const onAbort = () => {
        this.cmdPending.delete(id);
        signal?.removeEventListener('abort', onAbort);
        reject(aborted());
      };
      const cleanup = () => signal?.removeEventListener('abort', onAbort);
      this.cmdPending.set(id, {
        method, path, body: { ...body, msg_id: id },
        onOk: value => { cleanup(); resolve(value); },
        onErr: error => { cleanup(); reject(error); }, createdMs: now, sentMs: 0,
      });
      signal?.addEventListener('abort', onAbort, { once: true });
      if (!this.cmdTimer) this.cmdTimer = setInterval(() => this.cmdTick(), 500);
      this.cmdTick();
    });
  }

  private rejectPending(pick: (p: { method: string; body: any }) => boolean, why: string) {
    for (const [id, p] of Array.from(this.cmdPending)) {
      if (!pick(p)) continue;
      this.cmdPending.delete(id);
      p.onErr(new HttpError(0, `${p.path} — ${why}`));
    }
  }

  private cmdEnvelope(id: string) {
    const p = this.cmdPending.get(id)!;
    return JSON.stringify({ id, method: p.method, path: p.path, body: p.body });
  }

  private cmdSend(id: string): boolean {
    if (!this.commandDc || this.commandDc.readyState !== 'open') return false;
    if (simEngine.active && !isEstop(this.cmdPending.get(id)?.body)) return false;
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
    return remoteActive()
      ? this.dcCommand('GET', '/api/gamepad/ownership')
      : rest.getOwnership(this.ip);
  }

  async refreshOwnership() {
    const gen = this.openGen;
    try {
      const d = await this.getOwnership();
      if (gen !== this.openGen) return;
      useRobot.getState().setOwnership({ owner: d.ownerIP || '', myIp: d.requesterIP || '', isMine: !!d.IsOwner });
      useRobot.getState().setOwnerSupport(!!d.ownerSupport);
    } catch {}
  }

  extSend(channel: string, msg: object): boolean {
    const dc = this.extDcs.get(channel);
    if (!dc || dc.readyState !== 'open') return false;
    try { dc.send(JSON.stringify(msg)); return true; } catch { return false; }
  }

  async claimOwnership() {
    const gen = this.openGen;
    try {
      const d = await this.dcCommand('POST', '/api/gamepad/ownership');
      if (gen !== this.openGen) return false;
      useRobot.getState().setOwnership({ owner: d.ownerIP || '', myIp: d.requesterIP || '', isMine: !!d.IsOwner });
      return !!d.IsOwner;
    } catch {
      return false;
    }
  }

  fetchFeatures() {
    const gen = this.openGen;
    rest.robotFeatures(this.ip)
      .then((r) => {
        if (gen !== this.openGen) return;
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

  releaseAll() {
    this.axes.lx = this.axes.ly = this.axes.rx = this.axes.ry = 0;
    this.triggers.l = this.triggers.r = 0;
    this.buttons.fill(0);
    if (this.sendTimer) this.tickSend();
  }

  keepJoyAlive() {
    if (this.sendTimer) return;
    this.zeroTail = ZERO_TAIL_FRAMES;
    this.startSendLoop();
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
      try { this.motionDc.send(this.motionJoy(new ArrayBuffer(40))); } catch {}
    }
    this.wantConnected = false;
    this.clearReconnect();
    this.detachTimer = setTimeout(() => { this.detachTimer = null; this.disconnect(); }, 150);
    return ip;
  }

  private stopSendLoop() {
    if (this.sendTimer) { clearInterval(this.sendTimer); this.sendTimer = null; }
    this.zeroTail = 0;
    this.joyHealth.idle();
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
      if (!dc || dc.readyState !== 'open' || (dc.bufferedAmount ?? 0) > JOY_MAX_BUFFERED) return;
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
    const blocked = (this.motionDc.bufferedAmount ?? 0) > JOY_MAX_BUFFERED;
    if (this.allZero()) this.joyHealth.idle();
    else if (this.joyHealth.tick(monoMs(), blocked, this.joyV2)) noteJoyUnstable();
    if (blocked) return;
    const blind = this.lastStateAt > 0 && now - this.lastStateAt > STATE_LAG_MS;
    const keepAlive = CRUISE_GAIT_IDS.has(st.robot?.gait_id ?? -1);
    const idleTicks = this.allZero() ? this.zeroTail - ZERO_TAIL_FRAMES : -1;
    if (idleTicks < 0 || idleTicks % CRUISE_KEEPALIVE_TICKS === 0) {
      try { this.motionDc.send(this.motionJoy(blind ? new ArrayBuffer(40) : this.joyFrame())); } catch {}
    }
    this.tailOrStop(keepAlive);
  }

  private tailOrStop(keepAlive = false) {
    if (this.allZero()) {
      if (++this.zeroTail >= ZERO_TAIL_FRAMES && !keepAlive) this.stopSendLoop();
    } else {
      this.zeroTail = 0;
    }
  }

  private motionJoy(frame40: ArrayBuffer): ArrayBuffer {
    return this.joyV2 ? withJoyTail(frame40, ++this.joySeq, monoMs()) : frame40;
  }

  private onJoyLink(data: ArrayBuffer, body: number) {
    if (data.byteLength < body + JOY_LINK_STATS_BYTES) return;
    this.joyLink = parseJoyLinkStats(new DataView(data), body);
    if (this.joyHealth.stats(monoMs(), this.joyLink)) noteJoyUnstable();
    if (this.joyV2) return;
    this.joyV2 = true;
    if (!this.allZero() || !this.motionDc || this.motionDc.readyState !== 'open') return;
    for (let i = 0; i < JOY_SEED_FRAMES; i++) {
      try { this.motionDc.send(this.motionJoy(new ArrayBuffer(40))); } catch {}
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
    this.lastRxAt = this.lastStateAt = Date.now();
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
        case ID_JOY_LINK:
          this.onJoyLink(data, body);
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
    if (name === 'ai_eco') {
      if (this.motionInFlight.has(name)) return;
      this.motionInFlight.add(name);
      const id = useSettings.getState().visionWalkEnabled ? RL_GAIT.RL_TROT_VISION_ECO : RL_GAIT.RL_TROT_ECO;
      gaitApi.aiWalk(this.ip, id)
        .then(() => { this.motionInFlight.delete(name); this.refreshOwnership(); })
        .catch((e) => {
          this.motionInFlight.delete(name);
          const st = (e as HttpError).status;
          if (st === 403) this.refreshOwnership();
          this.surface('ERROR', `모션 'ai_eco' 실패 — ${st ? `HTTP ${st}${st === 403 ? ' (제어권 없음)' : ''}` : '응답 없음'}`);
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
setCommandSender((method, path, body, options) => connection.dcCommand(method, path, body ?? {}, options));
(globalThis as any).__rbqConn = connection;
