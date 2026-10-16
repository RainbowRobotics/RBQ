import { FW_SLOT_NAMES, FW_BODY_SLOTS, isMotorSlot } from './boardFirmware';

export const FW_DEMO_SCENARIOS = [
  'update', 'latest', 'mixed', 'motor_downgrade', 'boot', 'standby', 'eeprom', 'check', 'unmanaged', 'legs_off', 'variant',
  'unchecked', 'checking', 'busy', 'running', 'motors', 'success', 'failed', 'rejected', 'motor_rejected', 'changed', 'unsupported', 'classic',
] as const;

type Board = {
  slot: number; name: string; present: boolean; mode: string;
  hw: { state: string; ver: string | null; remembered: boolean };
  fw_target_a: number; app_version: number | null; boot_version: number | null;
  file: { name: string | null; version: number | null; a: number | null };
  verdict: string; step: string; result: string; reason: number; retry: number;
  progress: { sent: number; total: number };
};
export type DemoFwJson = {
  status: 'ok'; timestamp: string; supported: boolean; busy: string; check_seq: number;
  run: { id: number; kind: string; state: string; stage: string; board: number; reason: number };
  boards: Board[];
};

const FILE_V = 261004;
const LABEL: Record<number, string> = { 16: 'PDU_FW', 17: 'IF_FW', 18: 'SIDE_FRONT_APP', 19: 'SIDE_HIND_APP', 20: 'TOP_APP' };
const fileName = (slot: number, a: number) => `${LABEL[slot] ?? 'MOTOR_APP'}_HW${a}_V${FILE_V}_20261002_153000.bin`;
const boardA = (slot: number) => (slot === 17 ? 2 : 1);
export const demoHwVer = (a: number, b = 1) => [a, b, 0, 0].join('.');
const SIZE = (slot: number) => (isMotorSlot(slot) ? 98_304 : 131_072);
const NO_FILE = { name: null, version: null, a: null };

function base(): Board[] {
  return FW_SLOT_NAMES.map((name, slot) => {
    const motor = isMotorSlot(slot);
    const a = boardA(slot);
    return {
      slot, name, present: true, mode: 'app',
      hw: motor ? { state: 'old_firmware', ver: null, remembered: false } : { state: 'ok', ver: demoHwVer(a), remembered: false },
      fw_target_a: motor ? 0 : a,
      app_version: FILE_V, boot_version: 260422,
      file: { name: fileName(slot, a), version: FILE_V, a },
      verdict: 'latest', step: 'idle', result: 'none', reason: 0, retry: 0,
      progress: { sent: 0, total: 0 },
    };
  });
}

function patch(bs: Board[], slot: number, p: Partial<Omit<Board, 'hw' | 'file' | 'progress'>> & {
  hw?: Partial<Board['hw']>; file?: Partial<Board['file']>; progress?: Partial<Board['progress']>;
}) {
  const b = bs[slot];
  Object.assign(b, { ...p, hw: { ...b.hw, ...p.hw }, file: { ...b.file, ...p.file }, progress: { ...b.progress, ...p.progress } });
}
const eachMotor = (bs: Board[], p: Parameters<typeof patch>[2]) => { for (let m = 0; m < 16; m++) patch(bs, m, p); };

const json = (boards: Board[], over: Partial<DemoFwJson> = {}): DemoFwJson => ({
  status: 'ok', timestamp: new Date().toISOString(), supported: true, busy: 'none', check_seq: 12,
  run: { id: 3, kind: 'none', state: 'none', stage: 'none', board: -1, reason: 0 },
  boards: boards.map(bootPresence), ...over,
});
const bootPresence = (b: Board): Board => (b.mode === 'boot' || b.mode === 'boot_no_app' ? { ...b, present: false } : b);
const run = (p: Partial<DemoFwJson['run']>): DemoFwJson['run'] => ({ id: 3, kind: 'all', state: 'running', stage: 'none', board: -1, reason: 0, ...p });

function updatable(): Board[] {
  const bs = base();
  patch(bs, 16, { app_version: 261003, verdict: 'update' });
  patch(bs, 17, { app_version: 261002, verdict: 'update' });
  patch(bs, 19, { app_version: 261005, verdict: 'downgrade' });
  patch(bs, 20, { app_version: 261001, verdict: 'update' });
  eachMotor(bs, { app_version: 261001, verdict: 'update' });
  return bs;
}
function plainUpdatable(): Board[] {
  const bs = updatable();
  patch(bs, 19, { verdict: 'latest', app_version: FILE_V });
  return bs;
}

function burned(bs: Board[], slots: number[]) {
  for (const s of slots) patch(bs, s, { app_version: FILE_V, verdict: 'latest', result: 'success', mode: 'app' });
}

function unchecked(): Board[] {
  const bs = base();
  for (const b of bs) Object.assign(b, { verdict: 'unchecked', app_version: null, fw_target_a: 0, hw: { state: 'unchecked', ver: null, remembered: false }, file: { ...NO_FILE } });
  return bs;
}

function legsOff(bs: Board[]) {
  eachMotor(bs, { present: false, mode: 'unknown', app_version: null, boot_version: null, fw_target_a: 0, verdict: 'no_response', hw: { state: 'unchecked', ver: null } });
}

function failed(code: number): DemoFwJson {
  const bs = plainUpdatable();
  patch(bs, 18, { result: 'skipped' }); patch(bs, 19, { result: 'skipped' });
  if (code === 12) {
    burned(bs, [16, 17, 20]);
    legsOff(bs);
    return json(bs, { run: run({ state: 'failed', stage: 'leg_on', board: -1, reason: 12 }) });
  }
  if (code === 9) {
    burned(bs, [16]);
    patch(bs, 17, { mode: 'boot_no_app', app_version: null, fw_target_a: 0, verdict: 'hw_unknown', result: 'failed', reason: 9, hw: { state: 'unchecked', ver: null }, file: { ...NO_FILE } });
    return json(bs, { run: run({ state: 'failed', stage: 'boards', board: 17, reason: 9 }) });
  }
  burned(bs, [16, 17]);
  if (code === 14) {
    patch(bs, 20, { mode: 'standby', app_version: FILE_V, result: 'failed', reason: 14, hw: { state: 'unsupported', ver: demoHwVer(1, 2) } });
    return json(bs, { run: run({ state: 'failed', stage: 'boards', board: 20, reason: 14 }) });
  }
  if (code === 0) {
    patch(bs, 20, { mode: 'boot', app_version: null, step: 'transfer', progress: { sent: 52_000, total: SIZE(20) } });
    return json(bs, { run: run({ state: 'failed', stage: 'boards', board: 20, reason: 0 }) });
  }
  patch(bs, 20, { mode: 'boot', app_version: null, result: 'failed', reason: code, retry: 3 });
  return json(bs, { run: run({ state: 'failed', stage: 'boards', board: 20, reason: code }) });
}

function rejected(code: number): DemoFwJson {
  if (code === 13) {
    const bs = updatable();
    patch(bs, 17, { present: false, mode: 'unknown', app_version: null, verdict: 'no_response', reason: 13 });
    return json(bs, { run: run({ kind: 'board', state: 'rejected', stage: 'precheck', board: 17, reason: 13 }) });
  }
  if (code === 7) {
    const bs = base();
    for (const b of bs) b.result = 'skipped';
    return json(bs, { run: run({ state: 'rejected', stage: 'precheck', board: -1, reason: 7 }) });
  }
  const bs = updatable();
  if (code === 5) {
    patch(bs, 20, { mode: 'standby', verdict: 'hw_unknown', reason: 5, fw_target_a: 1, hw: { state: 'blank', ver: null } });
    return json(bs, { run: run({ state: 'rejected', stage: 'precheck', board: 20, reason: 5 }) });
  }
  if (code === 6) {
    patch(bs, 18, { verdict: 'no_file', reason: 6, hw: { state: 'ok', ver: demoHwVer(2) }, file: { ...NO_FILE } });
    return json(bs, { run: run({ state: 'rejected', stage: 'precheck', board: 18, reason: 6 }) });
  }
  if (code === 8) {
    patch(bs, 19, { reason: 8 });
    return json(bs, { run: run({ state: 'rejected', stage: 'precheck', board: 19, reason: 8 }) });
  }
  return json(bs, { run: run({ state: 'rejected', stage: 'precheck', board: -1, reason: code }) });
}

export function demoFwStatus(scenario: string): DemoFwJson {
  const [name, arg] = scenario.split(':');
  const code = Number(arg);
  const bs = updatable();
  switch (name) {
    case 'latest': return json(base());
    case 'mixed': {
      const b = base();
      patch(b, 2, { app_version: 261003, verdict: 'update' });
      patch(b, 15, { verdict: 'same' });
      return json(b);
    }
    case 'motor_rejected': {
      const b = base();
      eachMotor(b, { app_version: 261005, verdict: 'downgrade', reason: 8 });
      return json(b, { run: run({ kind: 'board', state: 'rejected', stage: 'leg_off', board: 0, reason: 8 }) });
    }
    case 'motor_downgrade': {
      const b = base();
      eachMotor(b, { app_version: 261005, verdict: 'downgrade' });
      return json(b);
    }
    case 'boot': {
      const b = base();
      patch(b, 17, { mode: 'boot_no_app', app_version: null, fw_target_a: 0, verdict: 'hw_unknown', hw: { state: 'unchecked', ver: null }, file: { ...NO_FILE } });
      return json(b);
    }
    case 'standby': case 'eeprom':
      patch(bs, 19, { verdict: 'latest', app_version: FILE_V });
      patch(bs, 20, { mode: 'standby', verdict: 'hw_unknown', reason: 5, fw_target_a: 1, hw: { state: name === 'eeprom' ? 'no_eeprom' : 'blank', ver: null } });
      return json(bs);
    case 'check':
      patch(bs, 19, { verdict: 'latest', app_version: FILE_V });
      patch(bs, 18, { verdict: 'no_file', reason: 6, hw: { state: 'ok', ver: demoHwVer(2) }, fw_target_a: 1, file: { ...NO_FILE } });
      return json(bs);
    case 'unmanaged':
      eachMotor(bs, { verdict: 'unmanaged', app_version: 261003, file: { ...NO_FILE } });
      return json(bs);
    case 'legs_off': {
      const b = base();
      legsOff(b);
      return json(b);
    }
    case 'variant': {
      const b = base();
      patch(b, 17, { mode: 'standby', fw_target_a: 1, verdict: 'update', hw: { state: 'unsupported', ver: demoHwVer(2) } });
      return json(b);
    }
    case 'unchecked': return json(unchecked(), { check_seq: 0 });
    case 'checking': return json(unchecked(), { check_seq: 0, busy: 'check', run: run({ kind: 'none', state: 'checking', stage: 'none' }) });
    case 'busy': return json(base(), { busy: 'update' });
    case 'running': {
      const b = plainUpdatable();
      burned(b, [16, 17]);
      patch(b, 18, { result: 'skipped' }); patch(b, 19, { result: 'skipped' });
      patch(b, 20, { mode: 'boot', step: 'transfer', retry: 1, progress: { sent: 56_320, total: SIZE(20) } });
      return json(b, { busy: 'update', run: run({ stage: 'boards', board: 20 }) });
    }
    case 'motors': {
      const b = plainUpdatable();
      burned(b, [16, 17, 20, 0]);
      patch(b, 18, { result: 'skipped' }); patch(b, 19, { result: 'skipped' });
      patch(b, 1, { mode: 'boot', step: 'transfer', progress: { sent: 68_608, total: SIZE(1) } });
      return json(b, { busy: 'update', run: run({ stage: 'motors', board: 1 }) });
    }
    case 'success': {
      const b = plainUpdatable();
      burned(b, [16, 17, 20, ...Array.from({ length: 16 }, (_, m) => m)]);
      for (const x of b) if (x.result === 'none') x.result = 'skipped';
      return json(b, { check_seq: 13, run: run({ state: 'success', stage: 'leg_off', board: -1 }) });
    }
    case 'failed': return failed(Number.isFinite(code) ? code : 11);
    case 'rejected': return rejected(Number.isFinite(code) && code > 0 ? code : 8);
    case 'changed':
      if (arg === 'after') {
        patch(bs, 18, { app_version: 261002, verdict: 'update' });
        return json(bs, { check_seq: 13 });
      }
      return json(bs);
    case 'classic': return json(bs, { supported: false });
    case 'update': case 'unsupported': default: return json(bs);
  }
}


const clone = <T,>(v: T): T => JSON.parse(JSON.stringify(v)) as T;
const now = () => ({ timestamp: new Date().toISOString() });

export function demoFwLiveCheck(before: DemoFwJson, elapsedMs: number): DemoFwJson {
  if (elapsedMs < 2000) return { ...before, ...now(), busy: 'check', run: { ...before.run, state: 'checking' } };
  const after = before.check_seq === 0 ? demoFwStatus('update') : clone(before);
  return { ...after, ...now(), busy: 'none', check_seq: Math.max(before.check_seq, 12) + 1 };
}

type Phase = { stage: string; ms: number; slot?: number; step?: string; pct?: [number, number]; retry?: number };

const inBoot = (b: Board) => b.mode === 'boot' || b.mode === 'boot_no_app';

function plan(bs: Board[], bodies: number[], motors: number[], cut: boolean, legs: boolean): Phase[] {
  const ph: Phase[] = [{ stage: 'precheck', ms: 1000 }];
  const burn = (slot: number) => {
    const motor = isMotorSlot(slot);
    const st = motor ? 'motors' : 'boards';
    const k = motor ? 0.4 : 1;
    if (inBoot(bs[slot])) {
      ph.push({ stage: st, ms: 800, slot, step: 'to_app' });
      ph.push({ stage: st, ms: 600, slot, step: 'read_hw' });
    }
    ph.push({ stage: st, ms: 600 * k, slot, step: 'enter_boot' });
    if (slot === 20) ph.push({ stage: st, ms: 1800, slot, step: 'transfer', pct: [0, 60] });
    ph.push({ stage: st, ms: 2400 * k, slot, step: 'transfer', pct: [0, 100], retry: slot === 20 ? 1 : undefined });
    ph.push({ stage: st, ms: 600 * k, slot, step: 'verify' });
    ph.push({ stage: st, ms: 600 * k, slot, step: 'reboot' });
  };
  if (cut) ph.push({ stage: 'power_off', ms: 1500 });
  for (const s of FW_BODY_SLOTS) if (bodies.includes(s)) burn(s);
  if (motors.length) {
    ph.push({ stage: 'leg_on', ms: 2000 });
    for (const s of motors) burn(s);
  }
  if (legs) ph.push({ stage: 'leg_off', ms: 1500 });
  return ph;
}

export function demoFwLiveRun(before: DemoFwJson, target: 'all' | 'motors' | number, allowDowngrade: boolean, powerDown: boolean, elapsedMs: number, runId: number): DemoFwJson {
  const bs: Board[] = clone(before.boards);
  const all = target === 'all';
  const withMotors = all || target === 'motors';
  const reach = (b: Board) => b.present || inBoot(b);
  const want = (b: Board) => reach(b) && (b.verdict === 'update' || (b.verdict === 'downgrade' && allowDowngrade)
    || (isMotorSlot(b.slot) && b.verdict === 'same') || (inBoot(b) && b.verdict === 'hw_unknown'));
  const bodies = bs.filter((b) => !isMotorSlot(b.slot) && (all || b.slot === target) && want(b)).map((b) => b.slot);
  const motorsAsleep = withMotors && bs.filter((b) => isMotorSlot(b.slot)).every((m) => m.verdict === 'no_response');
  const motors = withMotors ? (motorsAsleep ? [0] : bs.filter((b) => isMotorSlot(b.slot) && want(b)).map((b) => b.slot)) : [];
  for (const b of bs) Object.assign(b, { step: 'idle', result: 'none', reason: 0, retry: 0, progress: { sent: 0, total: 0 } });
  if (all) {
    for (const b of bs) {
      if (!bodies.includes(b.slot) && !motors.includes(b.slot) && !(motorsAsleep && isMotorSlot(b.slot)) && (reach(b) || b.verdict === 'unmanaged')) b.result = 'skipped';
    }
  }
  const cut = all || bodies.includes(16) || powerDown;
  const legs = all || withMotors;
  const ph = plan(bs, bodies, motors, cut, legs);
  let t = elapsedMs;
  let i = 0;
  let last = -1;
  for (; i < ph.length && t >= ph[i].ms; i++) {
    t -= ph[i].ms;
    const p = ph[i];
    if (p.slot != null) last = p.slot;
    if (p.stage === 'leg_on' && motorsAsleep) {
      for (let m = 0; m < 16; m++) Object.assign(bs[m], { present: true, mode: 'app', app_version: m === 0 ? 261001 : FILE_V, boot_version: 260422, verdict: m === 0 ? 'update' : 'latest', result: m === 0 ? 'none' : 'skipped' });
    }
    if (p.slot != null && p.step === 'read_hw') {
      const a = boardA(p.slot);
      Object.assign(bs[p.slot], { hw: { state: 'ok', ver: demoHwVer(a), remembered: true }, fw_target_a: a, file: { name: fileName(p.slot, a), version: FILE_V, a } });
    }
    if (p.slot != null && p.step === 'reboot') {
      const b = bs[p.slot];
      const fixed = b.hw.state === 'ok' || b.hw.state === 'unsupported' ? { fw_target_a: b.file.a ?? b.fw_target_a, hw: { ...b.hw, state: 'ok' } } : {};
      Object.assign(b, { result: 'success', mode: 'app', present: true, app_version: b.file.version, verdict: 'latest', step: 'idle', retry: ph.some((q) => q.slot === p.slot && q.retry) ? 1 : 0, ...fixed });
    }
  }
  const kind = all ? 'all' : 'board';
  const lastBoard = all ? -1 : target === 'motors' ? (last >= 0 ? last : 0) : (target as number);
  if (i >= ph.length) {
    const stage = legs ? 'leg_off' : 'boards';
    return { ...before, ...now(), busy: 'none', boards: bs.map(bootPresence), check_seq: before.check_seq + 1, run: { id: runId, kind, state: 'success', stage, board: lastBoard, reason: 0 } };
  }
  const p = ph[i];
  if (p.slot != null) {
    const b = bs[p.slot];
    const frac = p.pct ? (p.pct[0] + (p.pct[1] - p.pct[0]) * (t / p.ms)) / 100 : 0;
    const sending = p.step === 'transfer';
    Object.assign(b, { mode: p.step === 'to_app' || p.step === 'read_hw' ? b.mode : 'boot', step: p.step, retry: p.retry ?? 0,
      progress: { sent: sending ? Math.round(SIZE(p.slot) * frac) : 0, total: sending ? SIZE(p.slot) : 0 } });
  }
  return { ...before, ...now(), busy: 'update', boards: bs.map(bootPresence), run: { id: runId, kind, state: 'running', stage: p.stage, board: p.slot ?? (all ? -1 : lastBoard), reason: 0 } };
}
