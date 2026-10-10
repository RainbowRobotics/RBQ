import { useEffect, useState } from 'react';
import { STANDING_RAD } from '@/lib/robotPose';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { walkPercentToSi } from '@/lib/rest';
import { PROGRAM, WALKREADY_CMD } from '@/lib/robotState';
import type { RobotState, JointState } from '@/lib/robotState';
import type { RobotStatus } from '@/types/robot';
import { LEVELS, levelById, stepCourse, respawnPoint, fmtTime, type Progress } from '@/lib/simCourse';
import { useSimProgress } from '@/store/simProgress';
import { t } from '@/lib/i18n';

export type SimGait =
  | 'CONTROL_OFF' | 'SITTING' | 'STANDING' | 'TROTTING' | 'TROT_STAIRS'
  | 'WAVING' | 'TROT_RUNNING' | 'RL_TROT' | 'RL_PRONK' | 'RL_BOUND' | 'RL_PACE' | 'FALL_MODE';

const MOTION_MAP: Record<string, SimGait> = {
  sit: 'SITTING', pos_sit: 'SITTING',
  stand: 'STANDING', pos_stand: 'STANDING', lock: 'STANDING',
  walk: 'TROTTING', stairs: 'TROT_STAIRS', run: 'TROT_RUNNING', wave: 'WAVING',
  ai_walk: 'RL_TROT', ai_vision: 'RL_TROT', ai_vision_slow: 'RL_TROT', ai_run: 'TROT_RUNNING',
  ai_pronk: 'RL_PRONK', ai_bound: 'RL_BOUND', ai_pace: 'RL_PACE',
  rl_walk: 'RL_TROT', rl_walk_vision: 'RL_TROT',
  estop: 'FALL_MODE',
  auto_start: 'STANDING',
};

const UNSUPPORTED: Record<string, string> = {
  ai_2leg_f: 'AI 2Leg F', ai_2leg_l: 'AI 2Leg L', ai_2leg_r: 'AI 2Leg R', ai_3leg_hl: 'AI 3Leg',
  dock: 'Docking', zmp_calib: 'ZMP Calibrate',
};

const HW_ONLY: { path: string; label: string }[] = [
  { path: '/api/pdu', label: '전원(PDU)' },
  { path: '/api/dock', label: '도킹' },
  { path: '/api/payload', label: '페이로드' },
  { path: '/api/system/reboot', label: '재부팅' },
];

const GAIT_ID: Record<SimGait, number> = {
  FALL_MODE: -2, CONTROL_OFF: -1, SITTING: 0, STANDING: 1, TROTTING: 3,
  TROT_STAIRS: 4, WAVING: 5, TROT_RUNNING: 6, RL_TROT: 30, RL_BOUND: 35, RL_PACE: 36, RL_PRONK: 37,
};

const DEFAULT_GAIT: SimGait = 'TROTTING';

const clamp = (v: number, a: number, b: number) => (v < a ? a : v > b ? b : v);
const WALKING = new Set<SimGait>(['TROTTING', 'TROT_STAIRS', 'TROT_RUNNING', 'RL_TROT', 'RL_PRONK', 'RL_BOUND', 'RL_PACE']);

export type Shape = { t: 'b' | 'c'; p: [number, number, number]; s: number[]; m: string; e: number; yaw?: number; roll?: number };
export type ShapeFile = { colors: Record<string, string>; shapes: Shape[] };

export type SimCam = 'third' | 'front' | 'back' | 'stairs';
export type CourseState = { passed: number; total: number; done: boolean; elapsed: number; falls: number };

export type SimState = {
  ready: boolean;
  error: string | null;
  gait: SimGait;
  joints: number[];
  rpy: [number, number, number];
  xyz: [number, number, number];
  rtf: number;
  sps: number;
  shapes: ShapeFile | null;
  map: string;
  course: CourseState;
  cam: SimCam;
  note: string | null;
};

type Listener = (s: SimState) => void;

type WorkerMsg =
  | { t: 'ready'; timestep: number; nu: number; shapes?: ShapeFile; map?: string }
  | { t: 'err'; m: string }
  | { t: 'pose'; joints: number[]; rpy: [number, number, number]; xyz: [number, number, number];
      gait: string; vel: [number, number, number]; torque: number[]; rtf: number; sps: number };

class SimEngine {
  private refs = 0;
  private listeners = new Set<Listener>();
  private send: ((json: string) => void) | null = null;
  private queue: string[] = [];

  private cmd = { vx: 0, vy: 0, wz: 0 };
  private gait: SimGait = DEFAULT_GAIT;
  private maxSpeed = 1.0;
  private extJoy = false;
  private commissioned = true;
  private trip = { distMm: 0, stepCnt: 0, timeS: 0, last: [0, 0] as [number, number], swing: false, lastT: 0 };
  private lastVel: [number, number, number] = [0, 0, 0];
  private lastTorque: number[] = [];

  private state: SimState = {
    ready: false, error: null, gait: DEFAULT_GAIT,
    joints: STANDING_RAD, rpy: [0, 0, 0], xyz: [0, 0, 0.5], rtf: 0, sps: 0, shapes: null, map: LEVELS[0].id, note: null,
    course: { passed: 0, total: LEVELS[0].gates.length, done: false, elapsed: 0, falls: 0 }, cam: 'third',
  };

  get available() {
    return true;
  }

  get active() {
    return this.refs > 0;
  }

  debug() {
    const r = useTelemetry.getState().robot;
    return {
      ...this.state,
      active: this.active,
      hostAttached: !!this.send,
      cmd: this.cmd,
      maxSpeed: this.maxSpeed,
      robot: r ? { gaitId: r.gaitId, isFall: r.isFall, worldPos: r.worldPos, rpy: r.imu?.rpy } : null,
    };
  }

  subscribe(fn: Listener) {
    this.listeners.add(fn);
    fn(this.state);
    return () => {
      this.listeners.delete(fn);
    };
  }

  private emit(patch: Partial<SimState>) {
    this.state = { ...this.state, ...patch };
    this.listeners.forEach((l) => l(this.state));
  }


  attachHost(send: (json: string) => void) {
    this.send = send;
    return () => {
      if (this.send === send) this.send = null;
      this.emit({ ready: false, rtf: 0, sps: 0 });
    };
  }

  onMessage(raw: string) {
    let m: WorkerMsg;
    try { m = JSON.parse(raw) as WorkerMsg; } catch { return; }
    if (m.t === 'ready') {
      this.emit({ ready: true, error: null, shapes: m.shapes ?? this.state.shapes, map: m.map ?? this.state.map });
      this.restartCourse();
      this.post({ t: 'speed', mps: this.maxSpeed });
      if (this.gait !== 'STANDING') this.post({ t: 'gait', g: this.gait });
      for (const q of this.queue) this.send?.(q);
      this.queue = [];
    } else if (m.t === 'err') {
      this.emit({ error: m.m, ready: false });
    } else if (m.t === 'pose') {
      this.lastVel = m.vel;
      this.lastTorque = m.torque;
      this.emit({ joints: m.joints, rpy: m.rpy, xyz: m.xyz, rtf: m.rtf, sps: m.sps });
      this.publishTelemetry();
      this.tickCourse();
    }
  }

  private post(m: object) {
    const j = JSON.stringify(m);
    if (this.send && this.state.ready) this.send(j);
    else this.queue.push(j);
  }


  acquire() {
    this.refs++;
  }

  release() {
    this.refs = Math.max(0, this.refs - 1);
  }

  refresh() {
    this.emit({ ready: false, error: null });
    this.reloadSeq++;
    this.listeners.forEach((l) => l(this.state));
  }
  reloadSeq = 0;

  setMap(id: string) {
    if (!this.state.ready && !this.state.error) return;
    const lv = levelById(id);
    if (!lv) return;
    if (id === this.state.map) { this.reset(); return; }
    this.gait = DEFAULT_GAIT;
    this.cmd = { vx: 0, vy: 0, wz: 0 };
    this.emit({ ready: false, map: id, gait: DEFAULT_GAIT });
    this.publishGait();
    this.send?.(JSON.stringify({ t: 'map', name: id }));
    this.note(`${lv.no}. ${t(lv.name)}`);
  }
  setCam(cam: SimCam) {
    this.emit({ cam });
  }

  reset() {
    this.trip = { distMm: 0, stepCnt: 0, timeS: 0, last: [0, 0], swing: false, lastT: 0 };
    this.gait = 'STANDING';
    this.cmd = { vx: 0, vy: 0, wz: 0 };
    const lv = levelById(this.state.map);
    if (lv) this.post({ t: 'spawn', ...lv.spawn, fresh: true });
    else this.post({ t: 'reset' });
    this.restartCourse();
    this.setGait(DEFAULT_GAIT);
  }

  setMaxSpeed(mps: number) {
    this.maxSpeed = Math.max(0.1, mps);
    this.post({ t: 'speed', mps: this.maxSpeed });
    this.note(`최고 속도 ${this.maxSpeed.toFixed(1)} m/s`);
  }

  setWalkPosture(si: { max_speed: number; body_height: number }, tiltDeg: number) {
    this.maxSpeed = Math.max(0.1, si.max_speed);
    this.post({ t: 'speed', mps: this.maxSpeed });
    this.post({ t: 'posture', h: si.body_height, tilt: (tiltDeg * Math.PI) / 180 });
    this.note(`속도 ${this.maxSpeed.toFixed(1)} m/s · 높이 ${si.body_height >= 0 ? '+' : ''}${(si.body_height * 100).toFixed(0)}cm · 기울기 ${tiltDeg.toFixed(0)}° (기립 중 반영)`);
  }

  private stickPose = { roll: 0, pitch: 0, yaw: 0, h: 0 };

  setCommand(vx: number, vy: number, wz: number, ry = 0) {
    const walking = this.gait === 'TROTTING' || this.gait === 'TROT_STAIRS'
      || this.gait === 'TROT_RUNNING' || this.gait === 'RL_TROT'
      || this.gait === 'RL_PRONK' || this.gait === 'RL_BOUND' || this.gait === 'RL_PACE';
    if (!walking) {
      if (this.cmd.vx || this.cmd.vy || this.cmd.wz) {
        this.cmd = { vx: 0, vy: 0, wz: 0 };
        this.post({ t: 'cmd', vx: 0, vy: 0, wz: 0 });
      }
      if (this.gait === 'STANDING') {
        const n = {
          pitch: -clamp(vx, -1, 1) * 0.349,
          roll: -clamp(vy, -1, 1) * 0.4,
          yaw: -clamp(wz, -1, 1) * 0.436,
          h: clamp(ry, -1, 1) > 0 ? clamp(ry, 0, 1) * 0.05 : clamp(ry, -1, 0) * 0.15,
        };
        const p = this.stickPose;
        if (Math.abs(n.pitch - p.pitch) > 1e-3 || Math.abs(n.roll - p.roll) > 1e-3 || Math.abs(n.yaw - p.yaw) > 1e-3 || Math.abs(n.h - p.h) > 1e-3) {
          this.stickPose = n;
          this.post({ t: 'stick', ...n });
        }
      } else if ((vx || vy || wz) && this.gait === 'SITTING') {
        this.note('일어선 뒤 스틱으로 자세를, 보행 모션 뒤 이동을');
      }
      return;
    }
    const n = { vx: clamp(vx, -1, 1), vy: clamp(vy, -1, 1), wz: -clamp(wz, -1, 1) };
    if (n.vx === this.cmd.vx && n.vy === this.cmd.vy && n.wz === this.cmd.wz) return;
    this.cmd = n;
    this.post({ t: 'cmd', ...n });
  }

  motion(cmd: string): boolean {
    const label = UNSUPPORTED[cmd];
    if (label) {
      this.note(`${label} — 시뮬 미지원(학습 정책이 트롯 하나뿐)`);
      return true;
    }
    const g = MOTION_MAP[cmd];
    if (!g) return false;
    this.setGait(g);
    return true;
  }

  aiWalk(id: number): boolean {
    if (id === 45) this.setGait('TROT_RUNNING');
    else if (id === 48 || id === 49) this.setGait('RL_TROT');
    else return false;
    return true;
  }

  intercept(method: string, path: string, body: Record<string, unknown>): { body: unknown } | null {
    if (!this.active) return null;

    if (path === '/api/motion/command' && body.cmd === 'estop') return null;

    if (path === '/api/motion/command' && typeof body.user_command === 'number') {
      return { body: this.userCommand(Number(body.target), Number(body.user_command), body) };
    }

    if (path === '/api/motion/walk_parameters') {
      const si = body.unit === 'si'
        ? { max_speed: Number(body.max_speed), body_height: Number(body.body_height) }
        : walkPercentToSi({ max_speed: Number(body.max_speed), body_height: Number(body.body_height) });
      this.setWalkPosture(si, Number(body.body_tilt ?? 0));
      return { body: { result: 'ok' } };
    }

    if (path === '/api/gamepad/ownership') {
      return { body: { owner: 'me', is_mine: true, result: 'ok' } };
    }

    if (path === '/api/gamepad/external') {
      this.extJoy = !!body.enable || !!body.external || !!body.value;
      this.note(`외부 조종 ${this.extJoy ? 'ON' : 'OFF'}`);
      return { body: { result: 'ok' } };
    }

    if (path === '/api/trip/reset') {
      this.trip = { distMm: 0, stepCnt: 0, timeS: 0, last: [...this.trip.last], swing: false, lastT: this.trip.lastT };
      this.note('주행 기록 초기화');
      return { body: { result: 'ok' } };
    }

    const hw = HW_ONLY.find((h) => path.startsWith(h.path));
    if (hw) {
      this.note(`${hw.label} — 로봇 전용(시뮬 미지원)`);
      return { body: { result: 'ok' } };
    }

    if (method === 'GET') return null;
    return { body: { result: 'ok' } };
  }

  hwNote(label: string) {
    this.note(`${label} — 로봇 전용(시뮬 미지원)`);
  }

  private userCommand(target: number, cmd: number, body: Record<string, unknown>): unknown {
    if (target === PROGRAM.WalkReady) {
      if (cmd === WALKREADY_CMD.GO_RECOVERY_READY) {
        this.setGait('STANDING');
        this.note('에러 클리어 · 복구 준비');
        return { result: 'ok' };
      }
      if (cmd === WALKREADY_CMD.FALL_RECOVERY_MOTION) {
        this.setGait('STANDING');
        this.note('낙상 복구 — 다시 일어섭니다');
        return { result: 'ok' };
      }
      if (cmd === WALKREADY_CMD.JOINT_LOCK_UNLOCK || cmd === WALKREADY_CMD.JOINT_SPACE_JOG) {
        this.note('관절 조그·잠금 — 로봇 전용(시뮬 미지원)');
        return { result: 'ok' };
      }
    }
    if (target === PROGRAM.QuadWalk && cmd === 120) {
      const mode = Number((body.para_char as number[] | undefined)?.[0] ?? 0);
      const step = mode === 1 ? 0.25 : mode === 2 ? -0.25 : 0;
      this.setMaxSpeed(clamp(this.maxSpeed + step, 0.5, 2.5));
      return { result: 'ok' };
    }
    const done: Record<number, string> = {
      100: 'CAN 체크 완료', 101: 'Find Pose 완료',
      209: 'IMU 영점 완료', 225: '가속도계 보정 완료',
      115: 'ZMP 캘리브레이션 완료',
    };
    if (done[cmd]) {
      this.commissioned = true;
      this.publishGait();
      this.note(done[cmd]);
      return { result: 'ok' };
    }
    this.note(`명령 ${cmd} — 시뮬 미지원`);
    return { result: 'ok' };
  }

  private setGait(g: SimGait) {
    if (g === this.gait) return;
    this.gait = g;
    this.cmd = { vx: 0, vy: 0, wz: 0 };
    this.post({ t: 'gait', g });
    this.emit({ gait: g });
    this.publishGait();
  }

  private prog: Progress = { passed: 0, done: false };
  private courseT0 = 0;
  private falls = 0;
  private lastFacingNote = 0;

  private restartCourse() {
    const lv = levelById(this.state.map);
    this.prog = { passed: 0, done: false };
    this.courseT0 = performance.now();
    this.falls = 0;
    this.emit({ course: { passed: 0, total: lv?.gates.length ?? 0, done: false, elapsed: 0, falls: 0 } });
  }

  private tickCourse() {
    const lv = levelById(this.state.map);
    if (!lv || !this.state.ready) return;
    if (this.gait === 'FALL_MODE' || this.gait === 'CONTROL_OFF') return;
    const [x, y, z] = this.state.xyz;
    const walking = WALKING.has(this.gait);
    const { prog, ev } = stepCourse(lv, this.prog, { x, y, z, yaw: this.state.rpy[2] }, performance.now(), walking);
    this.prog = prog;
    const elapsed = prog.done ? this.state.course.elapsed : (performance.now() - this.courseT0) / 1000;
    if (ev?.t === 'fall') {
      this.falls++;
      this.post({ t: 'spawn', ...ev.to });
      const what = ev.stuck ? t('끼었습니다') : t('떨어졌습니다');
      this.note(prog.passed > 0 ? `${what} — ${t('게이트')} ${prog.passed} ${t('에서 다시')}` : `${what} — ${t('시작점에서 다시')}`);
    } else if (ev?.t === 'gate') {
      this.note(`${t('게이트')} ${ev.n}/${ev.total}`);
    } else if (ev?.t === 'wrongFacing') {
      const now = performance.now();
      if (now - this.lastFacingNote > 2500) { this.lastFacingNote = now; this.note(`${t('게이트')} ${ev.n}: ${t('옆걸음 게이트 — 화살표 방향을 본 채 지나가세요')}`); }
    } else if (ev?.t === 'clear') {
      const best = useSimProgress.getState().recordClear(lv.id, elapsed);
      this.note(`${t('클리어')}! ${fmtTime(elapsed)}${best ? ` · ${t('최고 기록')}` : ''}`);
    }
    const c = this.state.course;
    if (c.passed !== prog.passed || c.done !== prog.done || c.falls !== this.falls || Math.floor(c.elapsed * 10) !== Math.floor(elapsed * 10)) {
      this.emit({ course: { passed: prog.passed, total: lv.gates.length, done: prog.done, elapsed, falls: this.falls } });
    }
  }

  private publishGait() {
    const r = useRobot.getState();
    const base = r.robot ?? {
      battery_pct: 100, battery_voltage: 0,
      imu: true, can_bus: this.commissioned, find_pose: this.commissioned, control_started: true,
    };
    r.applyRobot({ ...base, gait_name: this.gait as RobotStatus['gait_name'], gait_id: GAIT_ID[this.gait] });
  }

  private noteTimer: ReturnType<typeof setTimeout> | null = null;
  private note(msg: string) {
    this.emit({ note: msg });
    if (this.noteTimer) clearTimeout(this.noteTimer);
    this.noteTimer = setTimeout(() => this.emit({ note: null }), 3500);
  }

  private publishTelemetry() {
    const s = this.state;
    const now = performance.now() / 1000;
    const dt = this.trip.lastT ? Math.min(0.2, now - this.trip.lastT) : 0;
    this.trip.lastT = now;

    const dx = s.xyz[0] - this.trip.last[0];
    const dy = s.xyz[1] - this.trip.last[1];
    this.trip.last = [s.xyz[0], s.xyz[1]];
    this.trip.distMm += Math.hypot(dx, dy) * 1000;
    this.trip.timeS += dt;
    const walking = Math.hypot(this.cmd.vx, this.cmd.vy) > 0.05;
    const swingNow = walking && Math.abs(s.joints[8]) > 1.4;
    if (swingNow && !this.trip.swing) this.trip.stepCnt++;
    this.trip.swing = swingNow;

    const joints: JointState[] = s.joints.map((pos, i) => ({
      connected: true, temperature: 38, locked: true,
      position: pos, torque: this.lastTorque[i] ?? 0,
      current: (this.lastTorque[i] ?? 0) / (i % 3 === 2 ? 3.5 : 2.6),
      run: this.gait !== 'FALL_MODE', calib: true, errors: [], statorTemp: 42,
    }));

    const prev = useTelemetry.getState().robot;
    const [roll, pitch, yaw] = s.rpy;
    const cy = Math.cos(yaw / 2), sy = Math.sin(yaw / 2);
    const cp = Math.cos(pitch / 2), sp = Math.sin(pitch / 2);
    const cr = Math.cos(roll / 2), sr = Math.sin(roll / 2);
    const next: RobotState = {
      ...(prev as RobotState),
      time: this.trip.timeS,
      gaitId: GAIT_ID[this.gait],
      isFall: this.gait === 'FALL_MODE' || Math.abs(roll) > 1.0 || Math.abs(pitch) > 1.0,
      extJoy: this.extJoy,
      attached: { arm: false, ext1: false, ext2: false, cctv: false, thermal: false, ptz: false },
      battery: prev?.battery ?? { percentage: 87, voltage: 54.1, current: 0 },
      imu: {
        quaternion: [
          cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
          cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy,
        ],
        rpy: [roll, pitch, yaw],
        gyro: [0, 0, this.cmd.wz],
        acc: [0, 0, 9.81],
      },
      worldPos: [s.xyz[0], s.xyz[1], s.xyz[2]],
      worldRpy: [roll, pitch, yaw],
      jointCount: 12,
      joints,
      tripTotals: { distMm: Math.round(this.trip.distMm), stepCnt: this.trip.stepCnt, timeS: Math.round(this.trip.timeS) },
      extDev: prev?.extDev ?? {},
      armStat: prev?.armStat ?? {
        canCheck: false, brakeRelease: false, conStart: false, isPacking: false,
        isReady: false, isHome: false, isStraight: false, motionCmd: 0, missionType: 0,
        manualControl: false, lockPosition: false,
      },
    };
    void this.lastVel;
    useTelemetry.getState().applyRobotState(next);
  }
}

export const simEngine = new SimEngine();
if (typeof globalThis !== 'undefined') {
  (globalThis as { __rbqSim?: unknown }).__rbqSim = simEngine;
}

export function useSimCourse() {
  const [v, setV] = useState<Pick<SimState, 'map' | 'course' | 'cam'> | null>(null);
  useEffect(() => simEngine.subscribe((s) => setV((old) =>
    old && old.map === s.map && old.course === s.course && old.cam === s.cam ? old : { map: s.map, course: s.course, cam: s.cam })), []);
  return v;
}
