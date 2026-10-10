import { loadMujocoSim, type MujocoSim } from '@/lib/mujocoSim.web';
import { RlTrotPolicy } from '@/lib/rlPolicy';

type Cmd =
  | { t: 'gait'; g: string }
  | { t: 'cmd'; vx: number; vy: number; wz: number }
  | { t: 'speed'; mps: number }
  | { t: 'posture'; h: number; tilt: number }
  | { t: 'stick'; roll: number; pitch: number; yaw: number; h: number }
  | { t: 'tune'; abdRoll?: number; rollH?: number }
  | { t: 'map'; name: string }
  | { t: 'spawn'; x: number; y: number; z: number; yaw: number; fresh?: boolean }
  | { t: 'reset' };

const clamp = (v: number, a: number, b: number) => (v < a ? a : v > b ? b : v);
const lerp = (a: number, b: number, t: number) => a + (b - a) * t;

const LIMITS: Record<string, { vx: number; vy: number; wz: number }> = {
  RL_PRONK: { vx: 0.3, vy: 0.1, wz: 0.4 },
  RL_BOUND: { vx: 0.4, vy: 0.1, wz: 0.4 },
  RL_PACE: { vx: 0.3, vy: 0.15, wz: 0.5 },
  TROTTING: { vx: 1.0, vy: 0.5, wz: 1.2 },
  TROT_STAIRS: { vx: 0.5, vy: 0.3, wz: 0.8 },
  TROT_RUNNING: { vx: 1.6, vy: 0.6, wz: 1.4 },
  RL_TROT: { vx: 1.0, vy: 0.5, wz: 1.2 },
};
const SCRIPTED: Record<string, { phase: number[]; freq: number; hip: number; knee: number; lean: number }> = {
  RL_PRONK: { phase: [0, 0, 0, 0], freq: 2.4, hip: 0.05, knee: 0.12, lean: 0.06 },
  RL_BOUND: { phase: [0, 0, Math.PI, Math.PI], freq: 2.4, hip: 0.05, knee: 0.10, lean: 0.045 },
  RL_PACE: { phase: [0, Math.PI, 0, Math.PI], freq: 2.6, hip: 0.06, knee: 0.14, lean: 0.06 },
};
const L1 = 0.33, L2 = 0.345, HIP_HALF_X = 0.19725 + 0.11493;
const FOOT_HALF_Y = 0.09 + 0.10285;
const ABD_YAW_SIGN = 1, YAW_X_SIGN = 1;
let tune = { abdRoll: -0.5, rollH: 0.5 };
const SIT_RAD = [0, 1.6, -2.6, 0, 1.6, -2.6, 0, 1.6, -2.6, 0, 1.6, -2.6];
const STANDING_RAD = [0, 0.873, -1.571, 0, 0.873, -1.571, 0, 0.873, -1.571, 0, 0.873, -1.571];

const post = (m: unknown) => {
  const w = window as unknown as { ReactNativeWebView?: { postMessage(s: string): void } };
  const s = JSON.stringify(m);
  if (w.ReactNativeWebView) w.ReactNativeWebView.postMessage(s);
  else parent?.postMessage(s, '*');
};

let sim: MujocoSim | null = null;
let policy: RlTrotPolicy | null = null;
let gait = 'STANDING';
let cmd = { vx: 0, vy: 0, wz: 0 };
let maxSpeed = 1.0;
let staticTarget: number[] | null = STANDING_RAD;
let poseTarget: number[] | null = null;
let polTarget: Float32Array | null = null;
let waveT = 0;
let scriptT = 0;
let posture = { h: 0, tilt: 0 };
let stick = { roll: 0, pitch: 0, yaw: 0, h: 0 };

function standTargets(def: number[], h: number, pitch: number, roll: number, yaw: number): number[] {
  const out = def.slice();
  for (let leg = 0; leg < 4; leg++) {
    const hip0 = def[leg * 3 + 1], knee0 = def[leg * 3 + 2];
    const fx = L1 * Math.sin(hip0) + L2 * Math.sin(hip0 + knee0);
    const fz0 = L1 * Math.cos(hip0) + L2 * Math.cos(hip0 + knee0);
    const front = leg >= 2 ? 1 : -1;
    const right = leg % 2 === 0 ? 1 : -1;
    const H = clamp(fz0 + h - front * HIP_HALF_X * Math.sin(pitch) - tune.rollH * right * FOOT_HALF_Y * Math.sin(roll), 0.30, 0.58);
    const dxYaw = YAW_X_SIGN * right * FOOT_HALF_Y * Math.sin(yaw);
    const dyYaw = -front * HIP_HALF_X * Math.sin(yaw);
    const cx = Math.cos(pitch), sx = Math.sin(pitch);
    const bx = (fx + dxYaw) * cx - H * sx;
    const bz = (fx + dxYaw) * sx + H * cx;
    const d2 = bx * bx + bz * bz;
    const knee = -Math.acos(clamp((d2 - L1 * L1 - L2 * L2) / (2 * L1 * L2), -1, 1));
    const hip = Math.atan2(bx, bz) - Math.atan2(L2 * Math.sin(knee), L1 + L2 * Math.cos(knee));
    out[leg * 3] = def[leg * 3] + tune.abdRoll * roll + ABD_YAW_SIGN * Math.atan2(dyYaw, H);
    out[leg * 3 + 1] = hip;
    out[leg * 3 + 2] = knee;
  }
  return out;
}

function setGait(g: string) {
  if (g === gait) return;
  const wasFallen = gait === 'FALL_MODE';
  gait = g;
  waveT = 0;
  staticTarget = g === 'SITTING' ? SIT_RAD : g === 'STANDING' ? (policy?.meta.defaultAngles ?? STANDING_RAD) : null;
  scriptT = 0;
  if (LIMITS[g] && !SCRIPTED[g] && policy && sim) policy.reset(sim.pose().joints);
  if (wasFallen && g !== 'FALL_MODE') sim?.reset();
  if (sim) poseTarget = sim.pose().joints.slice();
  cmd = { vx: 0, vy: 0, wz: 0 };
}

function handle(c: Cmd) {
  if (c.t === 'gait') setGait(c.g);
  else if (c.t === 'speed') maxSpeed = Math.max(0.1, c.mps);
  else if (c.t === 'posture') posture = { h: clamp(c.h, -0.25, 0.10), tilt: clamp(c.tilt, -0.35, 0.35) };
  else if (c.t === 'tune') tune = { abdRoll: c.abdRoll ?? tune.abdRoll, rollH: c.rollH ?? tune.rollH };
  else if (c.t === 'map') { if (c.name !== mapName) loadMap(c.name).catch((e) => { mapLoading = false; post({ t: 'err', m: String(e) }); }); }
  else if (c.t === 'stick') stick = { roll: clamp(c.roll, -0.25, 0.25), pitch: clamp(c.pitch, -0.35, 0.35), yaw: clamp(c.yaw, -0.4, 0.4), h: clamp(c.h, -0.15, 0.05) };
  else if (c.t === 'spawn') {
    const was = gait;
    if (c.fresh) posture = { h: 0, tilt: 0 };
    sim?.setSpawn([c.x, c.y, c.z], c.yaw);
    stick = { roll: 0, pitch: 0, yaw: 0, h: 0 };
    sim?.reset();
    policy?.reset(policy.meta.defaultAngles);
    polTarget = null;
    gait = '';
    setGait('STANDING');
    if (!c.fresh && LIMITS[was]) setGait(was);
  } else if (c.t === 'reset') {
    posture = { h: 0, tilt: 0 };
    stick = { roll: 0, pitch: 0, yaw: 0, h: 0 };
    sim?.reset();
    policy?.reset(policy.meta.defaultAngles);
    polTarget = null;
    gait = '';
    setGait('STANDING');
  } else if (c.t === 'cmd') {
    const lim = LIMITS[gait];
    if (!lim) { cmd = { vx: 0, vy: 0, wz: 0 }; return; }
    cmd = {
      vx: clamp(c.vx, -1, 1) * lim.vx * maxSpeed,
      vy: clamp(c.vy, -1, 1) * lim.vy * maxSpeed,
      wz: clamp(c.wz, -1, 1) * lim.wz,
    };
  }
}

let mapName = 'l01';
let loopStop: (() => void) | null = null;

let mapLoading = false;

async function loadMap(name: string) {
  if (mapLoading) return;
  mapLoading = true;
  mapName = name;
  const base = (globalThis as { __MUJOCO_BASE?: string }).__MUJOCO_BASE ?? '/mujoco';
  const pol = policy ?? (await RlTrotPolicy.load(`${base}/policy/rbq10_trot`));
  loopStop?.(); loopStop = null;
  sim?.dispose(); sim = null;
  const shapes = await fetch(`${base}/model/level_${name}.shapes.json`).then((r) => r.json()).catch(() => null);
  const sp = shapes?.spawn as { x: number; y: number; z: number; yaw: number } | undefined;
  const s = await loadMujocoSim(`model/level_${name}.xml`,
    { joints: pol.meta.defaultAngles, height: 0.62, pos: sp ? [sp.x, sp.y, sp.z] : undefined, yaw: sp?.yaw }, { interactive: true });
  pol.reset(pol.meta.defaultAngles);
  policy = pol; sim = s; staticTarget = pol.meta.defaultAngles;
  gait = ''; setGait('STANDING'); posture = { h: 0, tilt: 0 }; stick = { roll: 0, pitch: 0, yaw: 0, h: 0 };
  post({ t: 'ready', timestep: s.timestep, nu: s.nu, shapes, map: name });
  loopStop = loop(s, pol);
  mapLoading = false;
}

async function main() {
  try { await loadMap(mapName); }
  catch (e) { mapLoading = false; post({ t: 'err', m: e instanceof Error ? e.message : String(e) }); }
}

function loop(s: MujocoSim, pol: RlTrotPolicy): () => void {
  let alive = true;
  const CTRL_DT = pol.meta.policyDt;
  const MAX_TICKS = 8;
  let last = 0, budget = 0, steps = 0, acc = 0, teleAcc = 0;
  const torque = new Array<number>(s.nu);

  const control = () => {
    const q = s.pose().joints;
    const v = s.jointVels();
    if (gait === 'FALL_MODE') {
      for (let i = 0; i < s.nu; i++) torque[i] = -1.5 * v[i];
    } else if (SCRIPTED[gait]) {
      const sc = SCRIPTED[gait];
      scriptT += CTRL_DT;
      const def = pol.meta.defaultAngles;
      const drive = LIMITS[gait] ? cmd.vx / (LIMITS[gait].vx * Math.max(maxSpeed, 0.1)) : 0;
      for (let leg = 0; leg < 4; leg++) {
        const ph = 2 * Math.PI * sc.freq * scriptT + sc.phase[leg];
        const push = Math.max(0, Math.sin(ph));
        const fold = Math.max(0, -Math.sin(ph)) * 0.45;
        const b = leg * 3;
        torque[b] = pol.kp[b] * (def[b] - q[b]) - pol.kd[b] * v[b];
        const hipT = def[b + 1] - sc.hip * push + sc.hip * 0.4 * fold + sc.lean * drive;
        const kneeT = def[b + 2] + sc.knee * push - sc.knee * 0.5 * fold;
        torque[b + 1] = pol.kp[b + 1] * (hipT - q[b + 1]) - pol.kd[b + 1] * v[b + 1];
        torque[b + 2] = pol.kp[b + 2] * (kneeT - q[b + 2]) - pol.kd[b + 2] * v[b + 2];
      }
    } else if (LIMITS[gait]) {
      const ts = s.trunkState();
      polTarget = pol.step(q, v, ts.quat, ts.gyro, [cmd.vx, cmd.vy, cmd.wz]);
      for (let i = 0; i < s.nu; i++) torque[i] = pol.kp[i] * (polTarget[i] - q[i]) - pol.kd[i] * v[i];
    } else {
      const base = staticTarget ?? pol.meta.defaultAngles;
      const pose = gait === 'STANDING' && (posture.h || posture.tilt || stick.roll || stick.pitch || stick.yaw || stick.h);
      const tgt = pose
        ? standTargets(base, clamp(posture.h + stick.h, -0.25, 0.10), clamp(posture.tilt + stick.pitch, -0.35, 0.35), stick.roll, stick.yaw)
        : base.slice();
      if (gait === 'WAVING') {
        waveT += CTRL_DT;
        const w = Math.sin(waveT * 6);
        tgt[6] = 0.35 * w; tgt[7] = -0.9; tgt[8] = -1.0 + 0.25 * w;
      }
      if (!poseTarget) poseTarget = q.slice();
      const pt = poseTarget;
      const rate = clamp(CTRL_DT * 3, 0, 1);
      for (let i = 0; i < s.nu; i++) {
        pt[i] = lerp(pt[i], tgt[i], rate);
        torque[i] = pol.kp[i] * (pt[i] - q[i]) - pol.kd[i] * v[i];
      }
    }
    s.setCtrl(torque);
  };

  const frame = () => {
    if (!alive) return;
    setTimeout(frame, 4);
    const now = performance.now();
    const dt = last ? Math.min(0.1, (now - last) / 1000) : 0;
    last = now;
    if (dt <= 0) return;
    budget += dt;
    let ticks = 0;
    while (budget >= CTRL_DT && ticks < MAX_TICKS) {
      control();
      steps += s.advance(CTRL_DT);
      budget -= CTRL_DT;
      ticks++;
    }
    if (budget > CTRL_DT * MAX_TICKS) budget = 0;

    acc += dt; teleAcc += dt;
    if (teleAcc >= 1 / 30) {
      teleAcc = 0;
      const p = s.pose();
      post({ t: 'pose', joints: p.joints, rpy: p.rpy, xyz: p.xyz, gait,
        vel: s.trunkVel(), torque: s.jointTorques(),
        rtf: (steps * s.timestep) / Math.max(acc, 1e-6), sps: steps / Math.max(acc, 1e-6) });
      if (acc >= 1) { acc = 0; steps = 0; }
    }
  };
  frame();
  return () => { alive = false; };
}

(window as unknown as { __simCmd: (j: string) => void }).__simCmd = (j: string) => {
  try { handle(JSON.parse(j) as Cmd); } catch {}
};
window.addEventListener('message', (e) => {
  try { handle(JSON.parse(String((e as MessageEvent).data)) as Cmd); } catch {}
});

main();
