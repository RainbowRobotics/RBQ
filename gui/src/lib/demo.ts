import { useTelemetry } from '@/store/telemetry';
import { simEngine } from '@/lib/simEngine';
import { useRobot } from '@/store/robot';
import { useAccount } from '@/store/account';
import { useSettings } from '@/store/settings';
import { useViewport } from '@/store/viewport';
import { useGamepad } from '@/store/gamepad';
import { useFeatures } from '@/store/capability';
import type { LedSetting, LedSide } from '@/lib/ledBottom';
import type { AutostartInfo, InitState, ZmpCalibInfo, GyroCalibInfo, AccCalibInfo } from '@/types/robot';
import type { RobotState, JointState } from '@/lib/robotState';
import type { SensorState } from '@/lib/sensorState';
import { demoBlackboxChannels } from '@/lib/blackboxSchema';
import type { RobotFeatures } from '@/lib/rest';
import { FW_DEMO_SCENARIOS, demoFwStatus, demoFwLiveCheck, demoFwLiveRun, type DemoFwJson } from '@/lib/boardFirmwareDemo';

export { isDemo } from './demoFlag';
import { setDemoBaseLevel, setDemoBaseRobot, exitDemo } from './demoFlag';

const QS = typeof location === 'undefined' || typeof location.search !== 'string' ? null : new URLSearchParams(location.search);
const URL_FW = QS?.get('fw') || null;
const URL_OFFLINE = QS?.get('offline') === '1';
const URL_LEVEL = (() => { const n = Number(QS?.get('level')); return n === 1 || n === 2 || n === 3 ? n : null; })();

const J0 = [0, 0.716, -1.396, 0, 0.716, -1.396, 0, 0.716, -1.396, 0, 0.716, -1.396, 0, 0.3, -0.5, 0.2, 0.9, 0.1, 0];

function fakeJoint(i: number, t: number): JointState {
  const leg = i < 12;
  const phase = (i % 4) * (Math.PI / 2);
  return {
    connected: true, temperature: 34 + (i % 5), locked: true,
    position: J0[i] + (leg && t > 0 ? Math.sin(t * 2 + phase) * 0.18 : 0),
    torque: leg ? Math.sin(t * 2 + phase) * 6 : 1.2,
    current: leg ? Math.sin(t * 2 + phase) * 9 : 1.4,
    run: knobs.controlOn, calib: true, errors: [], statorTemp: 36 + (i % 7),
  };
}

function fakeRobotState(t: number): RobotState {
  const walking = knobs.walking ?? (Math.floor(t / 14) % 2 === 0);
  const gaitId = knobs.gaitId ?? (knobs.fall ? -2 : walking ? 3 : 1);
  return {
    time: t, gaitId, isFall: knobs.fall, extJoy: knobs.extJoy, isStanding: gaitId > 0,
    attached: { arm: knobs.arm, ext1: false, ext2: false, cctv: true, thermal: true, ptz: knobs.ptz },
    battery: { percentage: knobs.battery ?? 87 - (Math.floor(t / 60) % 20), voltage: 54.1, current: walking ? 6.4 : 1.1 },
    imu: {
      quaternion: [1, 0, 0, 0],
      rpy: [Math.sin(t * 1.9) * 0.02, Math.sin(t * 2.3) * 0.03, (t * 0.05) % (Math.PI * 2)],
      gyro: [Math.sin(t * 3) * 0.05, Math.cos(t * 2.7) * 0.05, 0.01],
      acc: [0.03, -0.02, 9.81],
    },
    worldPos: [Math.cos(t * 0.05) * 3, Math.sin(t * 0.05) * 3, 0.48],
    worldRpy: [0, 0, (t * 0.05 + Math.PI / 2) % (Math.PI * 2)],
    jointCount: 19,
    joints: Array.from({ length: 19 }, (_, i) => fakeJoint(i, walking ? t : 0)),
    tripTotals: { distMm: 3529500 + Math.floor(t * 800), stepCnt: 42000 + Math.floor(t * 2), timeS: 86400 + Math.floor(t) },
    extDev: {},
    armStat: { canCheck: true, brakeRelease: true, conStart: true, isPacking: false, isReady: true, isHome: false, isStraight: false },
  } as unknown as RobotState;
}

function fakeSensors(): SensorState[] {
  const base = {
    attached: true, powered: true, detected: true, connected: true, idle: false, sleep: false,
    running: true, failed: false, rgb: false, rgbOn: false, ir: false, irOn: false,
    depth: false, depthOn: false, projector: false, projectorOn: false, sensorEnabled: true,
    commUsb: true, commLan: false, camCalibRunning: false, camCalibSuccess: false,
    day: true, night: false, zoom: 0, errorId: 0, fps: [0, 0, 0] as [number, number, number],
  };
  return [
    { ...base, name: 'FT0', rgb: true, rgbOn: true, ir: true, irOn: true, depth: true, depthOn: true,
      projector: true, projectorOn: true, fps: [30, 30, 15] },
    { ...base, name: 'RR0', rgb: true, rgbOn: true, fps: [30, 0, 0] },
    { ...base, name: 'BT0', rgb: true, rgbOn: true, fps: [15, 0, 0] },
    { ...base, name: 'CTV', connected: false, rgb: true, rgbOn: true, zoom: 2, fps: [25, 0, 0] },
    { ...base, name: 'TML', attached: false, powered: false, detected: false, connected: false, running: false },
  ];
}

const LIDAR_DEVICE_IDX = 14;
const PTZ_DEVICE_IDX = 15;

function fakeDevices() {
  return Array.from({ length: 45 }, (_, i) => {
    const v = (i === LIDAR_DEVICE_IDX && !!knobs.lidar) || (i === PTZ_DEVICE_IDX && !!knobs.ptz);
    return { attached: v, powered: v, connected: v, normal: v, running: v };
  });
}

const DEMO_LOG_LINES = [
  ['Network', 'INFO', 'client connected (demo)'], ['QuadWalk', 'DEBUG', 'trot cadence 2.1Hz'],
  ['Motion', 'SUCCESS', 'gait → TROTTING'], ['WalkReady', 'WARNING', 'low battery margin (18%)'],
  ['Estimation', 'SUCCESS', 'state estimator converged'], ['Vision', 'INFO', 'marker detected id=3'],
  ['Network', 'INFO', 'telemetry stream 40Hz'], ['PDU', 'ERROR', 'rail 5 overcurrent (demo)'],
] as const;

const knobs = {
  walking: null as boolean | null,
  gaitId: (URL_FW ? 0 : null) as number | null,
  fall: false,
  battery: null as number | null,
  extJoy: false,
  arm: true, ptz: true,
  lidar: true,
  controlOn: true,
  level: (URL_LEVEL ?? 3) as 1 | 2 | 3,
  fw: URL_FW ?? 'update',
  offline: URL_OFFLINE,
  prevConn: undefined as { robotId: string; rendezvousUrl: string } | undefined,
  autostart: null as null | 'ok' | 'running' | 'comm' | 'home' | 'power' | 'gyro' | 'emo' | 'precheck' | 'classic',
  logs: typeof location === 'undefined' || !new URLSearchParams(location.search).has('quiet'),
  zmp: null as null | 'aligning' | 'running' | 'done' | 'failed',
  gyro: null as null | 'resetting' | 'pass' | 'fail' | 'standing',
  acc: null as null | 'running' | 'done' | 'standing',
  imuConnected: true,
  canFd: typeof location !== 'undefined' && new URLSearchParams(location.search).has('fd'),
  led: null as null | 'unset' | 'stale' | 'error' | 'oldfw',
};

let zmpRun = 0;
let gyroRun = 0;
let gyroT0 = 0;
let accRun = 0;
let accT0 = 0;

function fakeAcc(kind: NonNullable<typeof knobs.acc>): AccCalibInfo {
  const state = { running: 1, done: 2, standing: 3 }[kind];
  const pct = kind === 'running' ? Math.min(99, Math.floor((Date.now() - accT0) / 2000 * 100))
    : kind === 'done' ? 100 : 0;
  return {
    run: accRun, state, reason: kind === 'standing' ? 1 : 0, percent: pct,
    norm_before: 9.62, norm_after: 9.81, ratio: 1.02,
  };
}

function fakeGyro(kind: NonNullable<typeof knobs.gyro>): GyroCalibInfo {
  const total = 5000;
  const state = { resetting: 1, pass: 2, fail: 2, standing: 3 }[kind];
  return {
    run: gyroRun, state, reason: kind === 'standing' ? 1 : 0, pass: kind === 'pass',
    limit_dps: 1.0,
    elapsed_ms: kind === 'resetting' ? Math.min(total, Date.now() - gyroT0) : total,
    total_ms: total,
    bias_dps: kind === 'fail' ? [0.31, -1.42, 0.12] : [0.05, -0.08, 0.02],
  };
}
function fakeZmp(kind: NonNullable<typeof knobs.zmp>, t: number): ZmpCalibInfo {
  const state = { aligning: 1, running: 2, done: 3, failed: 4 }[kind];
  const percent = kind === 'running' ? Math.min(99, Math.floor((t % 20) * 5)) : kind === 'done' ? 100 : kind === 'failed' ? 61 : 0;
  return { run: zmpRun, state, reason: kind === 'failed' ? 3 : 0, percent, err_mm: Math.max(0, 8 - percent / 12) };
}

function fakeAutostart(kind: NonNullable<typeof knobs.autostart>): AutostartInfo {
  const IDLE: InitState = 0, RUN: InitState = 1, PASS: InitState = 2,
        WARN: InitState = 3, FAIL: InitState = 4;
  const fill = <T,>(n: number, v: T) => Array.from({ length: n }, () => v);
  const base: AutostartInfo = {
    running: false, can_fd: true, leg_rail_v: 48.4,
    step: 7, steps: [IDLE, PASS, PASS, PASS, PASS, PASS, PASS, PASS],
    step_ms: [0, 320, 5200, 1400, 4300, 1600, 5800, 1300], elapsed_ms: 19920,
    fail_code: 0, fail_ch: -1, can_mode_warn: false,
    pre_standing: false, emo_blocked: false,
    ch_comm: fill(16, PASS), ch_param: fill(16, PASS), ch_home: fill(12, PASS),
    home_err_deg: fill(12, 0.4), power_retry: 0, gyro_try: 0,
    gyro_bias_dps: [0.03, 0.02, 0.11], acc_norm_before: 9.62, acc_norm: 9.79, acc_pct: 100,
  };
  switch (kind) {
    case 'running': return { ...base, running: true, step: 5, elapsed_ms: 11200,
      steps: [IDLE, PASS, PASS, PASS, PASS, RUN, IDLE, IDLE],
      step_ms: [0, 320, 5200, 1400, 4300, 0, 0, 0],
      ch_home: [...fill(7, PASS), ...fill(5, IDLE)],
      gyro_bias_dps: undefined, acc_norm_before: 0, acc_norm: 0, acc_pct: 0 };
    case 'comm': return { ...base, step: 3, elapsed_ms: 6900,
      steps: [IDLE, PASS, PASS, FAIL, IDLE, IDLE, IDLE, IDLE],
      step_ms: [0, 320, 5200, 1400, 0, 0, 0, 0],
      fail_code: 2001006001, fail_ch: 1,
      ch_comm: fill(16, PASS).map((v, i) => (i === 1 ? FAIL : v)),
      ch_param: fill(16, IDLE), ch_home: fill(12, IDLE),
      gyro_bias_dps: undefined, acc_norm_before: 0, acc_norm: 0, acc_pct: 0 };
    case 'home': return { ...base, step: 5, elapsed_ms: 12500,
      steps: [IDLE, PASS, PASS, PASS, PASS, FAIL, IDLE, IDLE],
      step_ms: [0, 320, 5200, 1400, 4300, 1600, 0, 0],
      fail_code: 2001005009, fail_ch: 9,
      ch_home: fill(12, PASS).map((v, i) => (i === 9 || i === 11 ? FAIL : v)),
      home_err_deg: fill(12, 0.4).map((v, i) => (i === 9 ? -11.9 : i === 11 ? 27.4 : v)),
      gyro_bias_dps: undefined, acc_norm_before: 0, acc_norm: 0, acc_pct: 0 };
    case 'power': return { ...base, step: 2, elapsed_ms: 6300,
      steps: [IDLE, PASS, FAIL, IDLE, IDLE, IDLE, IDLE, IDLE],
      step_ms: [0, 320, 6000, 0, 0, 0, 0, 0],
      fail_code: 2005004001, fail_ch: -1, power_retry: 1, leg_rail_v: 0,
      ch_comm: fill(16, IDLE), ch_param: fill(16, IDLE), ch_home: fill(12, IDLE),
      gyro_bias_dps: undefined, acc_norm_before: 0, acc_norm: 0, acc_pct: 0 };
    case 'gyro': return { ...base, elapsed_ms: 24100,
      steps: [IDLE, PASS, PASS, PASS, PASS, PASS, WARN, PASS],
      step_ms: [0, 320, 5200, 1400, 4300, 1600, 10200, 1300],
      gyro_try: 1, gyro_bias_dps: [0.21, 1.82, 0.34] };
    case 'emo': return { ...base,
      steps: [IDLE, PASS, PASS, PASS, PASS, PASS, PASS, WARN], emo_blocked: true };
    case 'precheck': return { ...base, step: 1, elapsed_ms: 300,
      steps: [IDLE, FAIL, IDLE, IDLE, IDLE, IDLE, IDLE, IDLE],
      step_ms: [0, 300, 0, 0, 0, 0, 0, 0], fail_code: 0, fail_ch: -1, pre_standing: true,
      ch_comm: fill(16, IDLE), ch_param: fill(16, IDLE), ch_home: fill(12, IDLE),
      gyro_bias_dps: undefined, acc_norm_before: 0, acc_norm: 0, acc_pct: 0 };
    case 'classic': return { ...base, can_fd: false, elapsed_ms: 17400,
      steps: [IDLE, PASS, PASS, PASS, IDLE, PASS, PASS, PASS],
      step_ms: [0, 310, 3100, 1400, 0, 1600, 5800, 1300],
      ch_param: fill(16, IDLE), leg_rail_v: 48.1 };
    case 'ok':
    default: return base;
  }
}

let started = false;
export function startDemo(conn: { onMessage: (data: string) => void }) {
  if (started) return;
  started = true;
  const st = useSettings.getState();
  let base = st.accessLevel;
  try { base = JSON.parse(localStorage.getItem('rbq-settings') || '')?.state?.accessLevel ?? base; } catch {}
  setDemoBaseLevel(base as 1 | 2 | 3);
  if (st.accessLevel < 3) st.setAccessLevel(3);
  try {
    const b = document.createElement('div');
    b.textContent = 'DEMO';
    b.dataset.demoBadge = '1';
    b.title = '데모 모드 — 클릭하면 종료';
    Object.assign(b.style, {
      position: 'fixed', left: '0', top: '50%', transform: 'translateY(-50%)', zIndex: '99999',
      cursor: 'pointer', background: '#B5680C', color: '#fff', font: '700 10px monospace',
      letterSpacing: '1px', padding: '6px 4px', borderRadius: '0 6px 6px 0', opacity: '0.85',
      writingMode: 'vertical-rl',
    } as CSSStyleDeclaration);
    b.onclick = exitDemo;
    document.body.appendChild(b);
  } catch {}
  {
    const r = useRobot.getState();
    let ip = r.ip, visionIp = r.visionIp;
    try {
      const p = JSON.parse(localStorage.getItem('rbq-robot') || '')?.state;
      ip = p?.ip ?? ip; visionIp = p?.visionIp ?? visionIp;
    } catch {}
    setDemoBaseRobot({ ip, visionIp });
  }
  useRobot.getState().setIp('demo');
  useRobot.getState().setConn('connected');
  useRobot.getState().setOwnership({ owner: 'demo-me', myIp: 'demo-me', isMine: true });
  const t0 = Date.now();
  let logI = 0;
  const loop = setInterval(() => {
    const t = (Date.now() - t0) / 1000;
    if (useSettings.getState().accessLevel !== knobs.level) useSettings.getState().setAccessLevel(knobs.level);
    if (knobs.offline) {
      if (useRobot.getState().conn !== 'disconnected') { useRobot.getState().setConn('disconnected'); useFeatures.getState().clearFeatures(); }
      return;
    }
    if (useRobot.getState().conn !== 'connected') {
      useRobot.getState().setConn('connected');
      useRobot.getState().setOwnership({ owner: 'demo-me', myIp: 'demo-me', isMine: true });
    }
    const simOn = simEngine.active;
    if (!simOn) useTelemetry.getState().applyRobotState(fakeRobotState(t));
    useTelemetry.getState().applyDeviceStates(fakeDevices());
    useTelemetry.getState().applySensorStates(fakeSensors());
    {
      const want = demoFeatures();
      const f = useFeatures.getState().features;
      if (!f || !!f.fw_update !== !!want.fw_update || !!f.can_fd !== !!want.can_fd) useFeatures.getState().setFeatures(want);
    }
    const walking = knobs.walking ?? (Math.floor(t / 14) % 2 === 0);
    const cur = useRobot.getState().robot;
    conn.onMessage(JSON.stringify({
      t: 'robot_status', battery_pct: knobs.battery ?? 87, battery_voltage: 54.1,
      gait_id: simOn ? (cur?.gait_id ?? 1) : (knobs.gaitId ?? (knobs.fall ? -2 : walking ? 3 : 1)),
      gait_name: simOn ? (cur?.gait_name ?? 'STANDING') : (knobs.fall ? 'FALL_MODE' : walking ? 'TROTTING' : 'STANDING'),
      imu: true, imu_connected: knobs.imuConnected, can_bus: true, find_pose: true, control_started: knobs.controlOn,
      ...(knobs.autostart ? { autostart: fakeAutostart(knobs.autostart) } : {}),
      ...(knobs.zmp ? { zmp_calib: fakeZmp(knobs.zmp, t) } : {}),
      ...(knobs.gyro ? { gyro_calib: fakeGyro(knobs.gyro) } : {}),
      ...(knobs.acc ? { acc_calib: fakeAcc(knobs.acc) } : {}),
    }));
    conn.onMessage(JSON.stringify({
      t: 'pc_status', cpu_temp_c: 51 + Math.sin(t / 9) * 3, cpu_throttled: false,
      cpu_core_usage: [34, 22, 41, 18].map((v) => v + Math.round(Math.sin(t + v) * 8)),
      mem_total_kb: 32 * 1024 * 1024, mem_available_kb: 23 * 1024 * 1024, mem_used_pct: 28,
      swap_total_kb: 18874368, swap_used_kb: 7651328, swap_used_pct: 40,
    }));
    if (knobs.logs && Math.floor(t) % 3 === 0) {
      const [app, level, message] = DEMO_LOG_LINES[logI++ % DEMO_LOG_LINES.length];
      conn.onMessage(JSON.stringify({ application: app, level, message, timestamp: new Date().toISOString().replace('T', ' ').slice(0, 23) }));
    }
  }, 250);

  (globalThis as any).__rbqDemo = {
    set(patch: Partial<typeof knobs>) {
      if ('zmp' in patch) zmpRun++;
      if ('gyro' in patch) { gyroRun++; gyroT0 = Date.now(); }
      if ('acc' in patch) { accRun++; accT0 = Date.now(); }
      Object.assign(knobs, patch);
      if ('fw' in patch) { fwLive = null; fwChanged = false; fwNotify(); }
      return { ...knobs };
    },
    knobs: () => ({ ...knobs }),
    fwScenarios: [...FW_DEMO_SCENARIOS],
    gamepad(on: boolean) {
      useGamepad.getState().setDevices(on ? [{
        id: 0, name: 'Demo Gamepad', vendorId: 0x045e, productId: 0x02ea, descriptor: 'demo',
        sources: 0, keys: Array.from({ length: 17 }, (_, i) => i),
        axes: Array.from({ length: 4 }, (_, i) => ({ axis: i, label: `AXIS_${i}`, min: -1, max: 1, flat: 0 })),
      } as any] : []);
    },
    account(n = 2, level: 1 | 2 | 3 = 3) {
      const mk = (i: number) => ({
        accountId: 'demo', accountName: 'demo', level,
        robotSerial: `RBQ100000000${String(i).padStart(2, '0')}`,
        robotName: `RBQ100000000${String(i).padStart(2, '0')}`,
        lanIp: '192.168.0.10', wanIp: null,
        rendezvousUrl: 'ws://demo.rendezvous:8888/ws',
        robotId: `RBQ100000000${String(i).padStart(2, '0')}`,
        webrtcToken: null, expiresAt: null,
      });
      const list = Array.from({ length: n }, (_, i) => mk(i + 3));
      const acc = list[0] ?? { ...mk(3), robotSerial: null, robotName: null, robotId: null, rendezvousUrl: null };
      useAccount.getState().setSession('0000', acc as any, list as any);
      useSettings.getState().setAccountLevel(level);
      if (list[0]?.robotId) {
        const g = useSettings.getState();
        if (knobs.prevConn === undefined) knobs.prevConn = { robotId: g.robotId, rendezvousUrl: g.rendezvousUrl };
        g.setRobotId(list[0].robotId);
        g.setRendezvousUrl(list[0].rendezvousUrl!);
      }
    },
    logout() {
      useAccount.getState().clear(); useSettings.getState().setAccountLevel(null);
      if (knobs.prevConn) {
        useSettings.getState().setRobotId(knobs.prevConn.robotId);
        useSettings.getState().setRendezvousUrl(knobs.prevConn.rendezvousUrl);
        knobs.prevConn = undefined;
      }
    },
    level(n: 1 | 2 | 3) { knobs.level = n; useSettings.getState().setAccessLevel(n); },
    stop() { clearInterval(loop); },
    exit: exitDemo,
    stores: { settings: useSettings, robot: useRobot, telemetry: useTelemetry, gamepad: useGamepad, viewport: useViewport, features: useFeatures },
  };
}

function demoBlackboxValue(name: string, t: number, last: boolean): number {
  const idx = Number(/\[(\d+)\]$/.exec(name)?.[1] ?? 0);
  const ph = (i: number) => Math.sin(t * 2 + (i % 4) * 1.57);
  const stem = name.replace(/\[\d+\]$/, '');
  switch (stem) {
    case 'joint.pos': return J0[idx] + ph(idx) * 0.25;
    case 'ref.joint.pos': return J0[idx] + ph(idx) * 0.22;
    case 'joint.vel': return ph(idx + 1) * 1.8;
    case 'joint.torque': case 'ref.joint.torque': return ph(idx) * 6;
    case 'motor.cur': return ph(idx) * (idx % 3 === 2 ? 6 / 3.5 : 6 / 2.6);
    case 'motor.temp': return 41 + (idx % 5);
    case 'board.temp': return 36 + (idx % 7);
    case 'motor.connect': case 'motor.comm_stat': return 1;
    case 'can.motor.rx_hz': case 'can.wheel.rx_hz': return 500;
    case 'can.motor.gap_ms': case 'can.wheel.gap_ms': return 2;
    case 'can.ch.rx_hz': return 1800; case 'can.ch.tx_hz': return 1500;
    case 'can.ch.state': return 1;
    case 'imu.rpy.r': return Math.sin(t * 2) * 0.02;
    case 'imu.rpy.p': return Math.sin(t * 2.4) * 0.03;
    case 'imu.rpy.y': return t * 0.1;
    case 'imu.gyro.x': return 0.01; case 'imu.gyro.y': return 0.02;
    case 'imu.acc.x': return 0.03; case 'imu.acc.y': return -0.02; case 'imu.acc.z': return 9.81;
    case 'imu.connected': return 1;
    case 'process_time_ms': return 0.42;
    case 'cpu.temp': return 51;
    case 'cpu.core': return [34, 22, 41, 18][idx] ?? 0;
    case 'mem.total_kb': return 33554432; case 'mem.avail_kb': return 24117248;
    case 'lan2can.connected': return 1; case 'can.type': return 2;
    case 'status.con_start': case 'status.is_standing': case 'status.can_check':
    case 'status.find_home': case 'status.imu_success': case 'status.dq_success': return 1;
    case 'status.gait_id': return 3;
    case 'status.is_fall': return last ? 1 : 0;
    case 'cmd.vel_x': return 0.4; case 'cmd.gait_id': return 3; case 'cmd.updated': return 1;
    case 'joy.l_ud': return 0.5;
    default: break;
  }
  if (stem.startsWith('pdu.out.') || stem.startsWith('pdu.in.')) {
    const leaf = stem.slice(stem.lastIndexOf('.') + 1);
    const v48 = stem.includes('_48v');
    return leaf === 'state' ? 1 : leaf === 'v' ? (v48 ? 51.2 : 12.1) : 1.4;
  }
  if (stem.startsWith('pdu.bat.')) {
    const leaf = stem.slice(stem.lastIndexOf('.') + 1);
    return leaf === 'voltage' ? 54.1 : leaf === 'current' ? -3.2 : leaf === 'soc' ? 87 : leaf === 'detect' ? 1 : 0;
  }
  if (stem.startsWith('pdu.temp.')) return 38;
  return 0;
}

function demoBlackboxData(): string {
  const cols = demoBlackboxChannels();
  const rows = [cols.join('\t')];
  const N = 1500;
  for (let f = 0; f < N; f++) {
    const t = f / 100;
    const last = f >= N - 100;
    rows.push(cols.map((c) => String(demoBlackboxValue(c, t, last))).join('\t'));
  }
  return rows.join('\n');
}

const demoLogFile = () => Array.from({ length: 40 }, (_, i) => {
  const [app, level, message] = DEMO_LOG_LINES[i % DEMO_LOG_LINES.length];
  const ts = new Date(Date.now() - (40 - i) * 700).toISOString().replace('T', ' ').slice(0, 23);
  return JSON.stringify({ timestamp: ts, app, level, message });
}).join('\n');

const DEMO_PAYLOAD_SPECS: [string, string, number, number, number, number][] = [
  ['CUSTOM1', 'Custom 1', 0, 0, 0, 0],
  ['CUSTOM2', 'Custom 2', 0, 0, 0, 0],
  ['CUSTOM3', 'Custom 3', 0, 0, 0, 0],
  ['CUSTOM4', 'Custom 4', 0, 0, 0, 0],
  ['CUSTOM5', 'Custom 5', 0, 0, 0, 0],
  ['PTZ_CAM', 'PTZ Camera', 5.3, 0.065, 0, 0.19],
  ['LIDAR_LIVOX1', 'Livox LiDAR Front', 0.27, 0.373, 0, 0.128],
  ['LIDAR_LIVOX2', 'Livox LiDAR Rear', 0.27, -0.333, 0, 0.208],
  ['LIDAR_OUSTER', 'Ouster LiDAR', 0.65, -0.312, 0, 0.132],
  ['SOUND_CAM', 'Sound Camera', 0, 0, 0, 0],
  ['LTE_5G', 'LTE / 5G', 0.7, -0.275, 0, 0.085],
  ['SWITCH_HUB', 'Switch Hub', 0, 0, 0, 0],
  ['UPC_OUTER', 'External UPC', 0, 0, 0, 0],
];
const DEMO_PAYLOAD_MOUNTED = new Set(['PTZ_CAM', 'LIDAR_LIVOX1', 'LIDAR_LIVOX2', 'LIDAR_OUSTER', 'LTE_5G']);

function demoPayload() {
  const slots = DEMO_PAYLOAD_SPECS.map(([name, label, mass, x, y, z], id) => {
    const on = DEMO_PAYLOAD_MOUNTED.has(name);
    return {
      id, name, label, is_custom: id <= 4,
      mass_kg: on ? mass : 0,
      center_of_mass: { x_m: on ? x : 0, y_m: on ? y : 0, z_m: on ? z : 0 },
      default_mass_kg: mass,
      default_center_of_mass: { x_m: x, y_m: y, z_m: z },
    };
  });
  return {
    slots,
    limits: { mass_min_kg: 0, mass_max_kg: 20, mass_total_min_kg: 0, mass_total_max_kg: 20,
              com_x_max_m: 0.5, com_y_max_m: 0.3, com_z_max_m: 0.4 },
    payload: { mass_kg: slots[0].mass_kg, center_of_mass: slots[0].center_of_mass },
  };
}

function demoPduFd() {
  const names = ['LAN_12V_1', 'LAN_12V_2', 'LAN_12V_3', 'WAN_12V', 'TBD_12V', 'FRONT_12V', 'HIND_12V',
    'CAMERA_0', 'CAMERA_1', 'CAMERA_2', 'CAMERA_3', 'CAMERA_4', 'CAMERA_5', 'LEG_48V', 'ARM_48V', 'PDU_48V', 'SIDE_CAM_LEFT', 'SIDE_CAM_RIGHT', 'AMP'];
  const port_out = names.map((name, index) => {
    const v48 = index >= 13 && index <= 15;
    const v12 = index <= 6;
    const state = name === 'CAMERA_3' ? -1 : name === 'ARM_48V' ? 0 : 1;
    return { index, name, state, voltage: state === 1 ? (v48 ? 52.3 : v12 ? 12.1 : 5.0) : 0, current: state === 1 ? (v48 ? 1.9 : 0.3) : 0 };
  });
  const bat = (index: number, name: string, soc: number) => ({ index, name, voltage: 52.3, current: -1.9, soc,
    detect: true, ov: false, uv: false, ot: false, ut: false, occ: false, ocd: false, scd: false, cid: false });
  return {
    pdu_fd: {
      port_out,
      port_in: ['EMO', 'CHG_STATION', 'CHG_EXTERNAL'].map((name, index) => ({ index, name, state: 0, voltage: 0, current: 0 })),
      battery: [bat(0, 'LEFT', 78), bat(1, 'RIGHT', 76)],
      temperature: ['BAT_LEFT', 'BAT_RIGHT', 'TOP_POWER', 'PDU_POWER', 'PDU_SIGNAL'].map((name, index) => ({ index, name, value: 31 + index })),
    },
    timestamp: new Date().toISOString(), status: 'ok',
  };
}

let demoLedRx = 0;
const demoLedAge = (now: number) => now - Math.max(demoLedRx, Math.floor(now / 1000) * 1000);
const demoLed: Record<LedSide, LedSetting> = {
  right: { mode: 'blink', rgb: [0, 255, 0], on_ms: 800, off_ms: 800, count: 0 },
  left: { mode: 'on', rgb: [0, 127, 0], on_ms: 0, off_ms: 0, count: 0 },
};
function demoLedReport(s: LedSetting) {
  if (s.mode === 'off') return { ...s, actual: [0, 0, 0], limited: false, lpf_w: 0, legacy: false };
  const w = (s.rgb[0] * 10.4 + s.rgb[1] * 20 + s.rgb[2] * 20) / 255;
  const k = Math.min(1, (s.mode === 'on' ? 10.5 : 20) / Math.max(w, 1e-6));
  const duty = s.mode === 'on' ? 1 : s.on_ms / (s.on_ms + s.off_ms);
  return { ...s, actual: s.rgb.map((v) => Math.floor(v * k)), limited: k < 1, lpf_w: Math.round(Math.min(w * k * duty, 10.5) * 10) / 10, legacy: false };
}
function demoLedBottom(method: string, body: Record<string, any>) {
  if (knobs.led === 'error') throw Object.assign(new Error('/api/led/bottom → 404 (demo)'), { status: 404 });
  if (method === 'PUT') {
    const blink = body.mode === 'blink';
    const next: LedSetting = { mode: body.mode, rgb: body.rgb, on_ms: blink ? body.on_ms : 0, off_ms: blink ? body.off_ms : 0, count: blink ? body.count : 0 };
    const sides: LedSide[] = body.side === 'both' ? ['right', 'left'] : [body.side];
    setTimeout(() => { for (const sd of sides) demoLed[sd] = next; demoLedRx = Date.now(); }, 400);
    return { user_command: 1834, target: 'Motion' };
  }
  const unset = knobs.led === 'unset' || knobs.led === 'oldfw';
  const off: LedSetting = { mode: 'off', rgb: [0, 0, 0], on_ms: 0, off_ms: 0, count: 0 };
  return { led_bottom: {
    age_ms: knobs.led === 'oldfw' ? -1 : knobs.led === 'stale' ? 5200 : demoLedAge(Date.now()), configured: !unset,
    right: demoLedReport(unset ? off : demoLed.right), left: demoLedReport(unset ? off : demoLed.left),
    if_fw: knobs.led === 'oldfw' ? 261003 : 0,
  } };
}

function demoFeatures(): RobotFeatures {
  return { fire_fight: false, slam: true, wheel: false, qc: false, debug: false, can_fd: knobs.canFd,
    ...(knobs.fw !== 'unsupported' ? { fw_update: true } : null) };
}

type FwLive =
  | { kind: 'check'; t0: number; before: DemoFwJson }
  | { kind: 'run'; t0: number; before: DemoFwJson; target: 'all' | 'motors' | number; allowDowngrade: boolean; powerDown: boolean; id: number };
let fwLive: FwLive | null = null;
let fwRunId = 3;
let fwChanged = false;
const fwListeners = new Set<() => void>();
export function onDemoFwChange(fn: () => void): () => void { fwListeners.add(fn); return () => { fwListeners.delete(fn); }; }
function fwNotify() { fwListeners.forEach((fn) => { try { fn(); } catch { } }); }
function demoFwNow(): DemoFwJson {
  if (!fwLive) return demoFwStatus(knobs.fw === 'changed' && fwChanged ? 'changed:after' : knobs.fw);
  const dt = Date.now() - fwLive.t0;
  return fwLive.kind === 'check' ? demoFwLiveCheck(fwLive.before, dt)
    : demoFwLiveRun(fwLive.before, fwLive.target, fwLive.allowDowngrade, fwLive.powerDown, dt, fwLive.id);
}
const fwReject = (path: string, status: number, error: string) => Promise.reject(Object.assign(new Error(`${path} → ${status} (${error})`), { status }));
function demoFirmware(path: string, method: string, body: unknown): unknown {
  const b = (body ?? {}) as { board?: unknown; allow_downgrade?: unknown; power_down?: unknown; check_seq?: unknown };
  const ok = { status: 'ok', timestamp: new Date().toISOString(), demo: true };
  if (method === 'POST' && path.startsWith('/api/firmware/check')) {
    const before = demoFwNow();
    fwLive = { kind: 'check', t0: Date.now(), before };
    return { ...ok, check_seq: before.check_seq };
  }
  if (method === 'POST' && path.startsWith('/api/firmware/update')) {
    if (knobs.fw === 'changed' && !fwChanged && !fwLive) fwChanged = true;
    const before = demoFwNow();
    if (typeof b.check_seq === 'number' && b.check_seq !== before.check_seq) return fwReject('/api/firmware/update', 409, 'firmware status changed');
    if (typeof b.board === 'number' && b.board >= 0 && b.board < 16) return fwReject('/api/firmware/update', 400, 'motors are updated together — use board "motors"');
    fwLive = { kind: 'run', t0: Date.now(), before, target: b.board === 'all' || b.board === 'motors' ? b.board : Number(b.board),
      allowDowngrade: b.allow_downgrade === true, powerDown: b.power_down === true, id: ++fwRunId };
    return ok;
  }
  return demoFwNow();
}

export function demoRest(path: string, method = 'GET', body: Record<string, any> = {}): any {
  if (path.startsWith('/api/led/bottom')) return demoLedBottom(method, body);
  const today = new Date().toISOString().slice(0, 10).replace(/-/g, '');
  if (path.startsWith('/api/version')) return { version: '1.20.0-demo', branch: 'develop', commit_date: '2026-08-24 10:00:00', build_date: '2026-08-24 10:30:00' };
  if (path.startsWith('/api/robot/serial_number')) return { serial_number: 'RBQ-DEMO-001' };
  if (path.startsWith('/api/robot/features')) return { features: demoFeatures() };
  if (path.startsWith('/api/firmware/')) return demoFirmware(path, method, body);
  if (path.startsWith('/api/pdu/fd/port')) return { status: 'ok', demo: true };
  if (path.startsWith('/api/pdu/fd')) return demoPduFd();
  if (path.startsWith('/api/trip')) return { total_m: 3529.5, trip_a_m: 3350.3, trip_b_m: 3350.3 };
  if (path.startsWith('/api/blackbox/list')) return { files: ['data.log', 'systemlog.log', 'meta.json'].map((f) => ({ path: `${today}/10_30_00/${f}`, size: 1000, mtime: Date.now() })) };
  if (path.includes('data.log')) return demoBlackboxData();
  if (path.includes('systemlog.log')) return demoLogFile();
  if (path.includes('meta.json')) return JSON.stringify({ basename: '10_30_00', data_frame_count: 1500, data_tick_ms: 10, data_start_epoch_ms: Date.now() - 15000, trigger_epoch_ms: Date.now() });
  if (path.includes('video.json')) return '';
  if (path.startsWith('/api/systemlog/list')) return { files: [{ path: `${today}.log`, size: 4000, mtime: Date.now() }] };
  if (path.startsWith('/api/systemlog')) return demoLogFile();
  if (path.startsWith('/api/payload/parameters')) return demoPayload();
  if (path.startsWith('/api/dock/parameters')) return { dock: { offset_x: 0.133, offset_y: 0, count_req: 300, count_try: 10 } };
  if (path.startsWith('/api/gamepad/ownership')) return { IsOwner: true, ownerIP: 'demo-me', requesterIP: 'demo-me', status: 'ok' };
  if (path.startsWith('/api/motion/leg_home_set')) return {
    boards_alive: true, running: false, leg: 0, step: 9, steps: 9, tolerance_deg: 0.5, timestamp: new Date().toISOString(),
    legs: [0, 1, 2, 3].map((leg) => ({
      leg,
      joints: [leg * 3, leg * 3 + 1, leg * 3 + 2],
      expected_deg: [leg % 2 === 0 ? -42 : 42, 180, -158.2],
      ...(leg === 0 ? { ok: true, measured_deg: [-41.98, 179.88, -158.22], finished_at: new Date().toISOString() } : null),
    })),
  };
  return { status: 'ok', demo: true };
}
