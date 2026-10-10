import { t } from './i18n';

export type FwMode = 'app' | 'boot' | 'boot_no_app' | 'standby' | 'unknown';
export type FwHwState = 'unchecked' | 'ok' | 'blank' | 'no_eeprom' | 'unsupported' | 'old_firmware' | 'unknown_d';
export type FwVerdict = 'unchecked' | 'latest' | 'update' | 'downgrade' | 'same' | 'no_file' | 'hw_unknown' | 'no_response' | 'unmanaged';
export type FwStep = 'idle' | 'to_app' | 'read_hw' | 'enter_boot' | 'transfer' | 'verify' | 'reboot';
export type FwResult = 'none' | 'success' | 'failed' | 'skipped';
export type FwRunKind = 'none' | 'board' | 'all' | 'force' | 'force_all';
export type FwRunState = 'none' | 'checking' | 'running' | 'success' | 'failed' | 'rejected';
export type FwRunStage = 'none' | 'precheck' | 'power_off' | 'boards' | 'leg_on' | 'motors' | 'leg_off';
export type FwBusy = 'none' | 'check' | 'update';

const MODES: readonly FwMode[] = ['app', 'boot', 'boot_no_app', 'standby', 'unknown'];
const HW_STATES: readonly FwHwState[] = ['unchecked', 'ok', 'blank', 'no_eeprom', 'unsupported', 'old_firmware', 'unknown_d'];
const VERDICTS: readonly FwVerdict[] = ['unchecked', 'latest', 'update', 'downgrade', 'same', 'no_file', 'hw_unknown', 'no_response', 'unmanaged'];
const STEPS: readonly FwStep[] = ['idle', 'to_app', 'read_hw', 'enter_boot', 'transfer', 'verify', 'reboot'];
const RESULTS: readonly FwResult[] = ['none', 'success', 'failed', 'skipped'];
const KINDS: readonly FwRunKind[] = ['none', 'board', 'all', 'force', 'force_all'];
const RUN_STATES: readonly FwRunState[] = ['none', 'checking', 'running', 'success', 'failed', 'rejected'];
const STAGES: readonly FwRunStage[] = ['none', 'precheck', 'power_off', 'boards', 'leg_on', 'motors', 'leg_off'];
const BUSY: readonly FwBusy[] = ['none', 'check', 'update'];

export type FwBoard = {
  slot: number;
  name: string;
  present: boolean;
  mode: FwMode;
  hw: { state: FwHwState; ver: string | null; remembered: boolean };
  fwTargetA: number;
  appVersion: number | null;
  bootVersion: number | null;
  file: { name: string | null; version: number | null; a: number | null };
  verdict: FwVerdict;
  step: FwStep;
  result: FwResult;
  reason: number;
  retry: number;
  progress: { sent: number; total: number };
};

export type FwStatus = {
  supported: boolean;
  busy: FwBusy;
  checkSeq: number;
  run: { id: number; kind: FwRunKind; state: FwRunState; stage: FwRunStage; board: number; reason: number };
  boards: FwBoard[];
};

export const FW_SLOT_NAMES = [
  'HRR', 'HRP', 'HRK', 'HRW', 'HLR', 'HLP', 'HLK', 'HLW', 'FRR', 'FRP', 'FRK', 'FRW', 'FLR', 'FLP', 'FLK', 'FLW',
  'PDU', 'IF', 'SIDE_F', 'SIDE_H', 'TOP',
] as const;
const SLOT_PDU = 16;
const SLOT_TOP = 20;
export const FW_BODY_SLOTS = [16, 17, 18, 19, 20] as const;
export const isMotorSlot = (slot: number) => slot >= 0 && slot < 16;


const pick = <T extends string>(v: unknown, allowed: readonly T[], dflt: T): T =>
  (typeof v === 'string' && (allowed as readonly string[]).includes(v) ? (v as T) : dflt);
const num = (v: unknown): number | null => (typeof v === 'number' && Number.isFinite(v) ? v : null);
const int = (v: unknown, dflt = 0): number => num(v) ?? dflt;
const str = (v: unknown): string | null => (typeof v === 'string' && v !== '' ? v : null);

export function parseFwStatus(raw: unknown): FwStatus | null {
  const r = raw as Record<string, any> | null;
  if (!r || typeof r !== 'object' || !Array.isArray(r.boards)) return null;
  const run = (r.run ?? {}) as Record<string, unknown>;
  const boards: FwBoard[] = (r.boards as unknown[]).flatMap((x) => {
    const b = x as Record<string, any> | null;
    if (!b || typeof b !== 'object') return [];
    const slot = int(b.slot, -1);
    if (slot < 0) return [];
    const hw = (b.hw ?? {}) as Record<string, unknown>;
    const file = (b.file ?? {}) as Record<string, unknown>;
    const prog = (b.progress ?? {}) as Record<string, unknown>;
    return [{
      slot,
      name: str(b.name) ?? FW_SLOT_NAMES[slot] ?? `#${slot}`,
      present: b.present === true,
      mode: pick(b.mode, MODES, 'unknown'),
      hw: { state: pick(hw.state, HW_STATES, 'unchecked'), ver: str(hw.ver), remembered: hw.remembered === true },
      fwTargetA: int(b.fw_target_a),
      appVersion: num(b.app_version),
      bootVersion: num(b.boot_version),
      file: { name: str(file.name), version: num(file.version), a: num(file.a) },
      verdict: pick(b.verdict, VERDICTS, 'unchecked'),
      step: pick(b.step, STEPS, 'idle'),
      result: pick(b.result, RESULTS, 'none'),
      reason: int(b.reason),
      retry: int(b.retry),
      progress: { sent: int(prog.sent), total: int(prog.total) },
    }];
  });
  boards.sort((a, b) => a.slot - b.slot);
  return {
    supported: r.supported === true,
    busy: r.busy == null ? 'none' : pick(r.busy, BUSY, 'update'),
    checkSeq: int(r.check_seq),
    run: {
      id: int(run.id),
      kind: pick(run.kind, KINDS, 'none'),
      state: pick(run.state, RUN_STATES, 'none'),
      stage: pick(run.stage, STAGES, 'none'),
      board: int(run.board, -1),
      reason: int(run.reason),
    },
    boards,
  };
}


const inBoot = (b: FwBoard) => b.mode === 'boot' || b.mode === 'boot_no_app';

export const reachable = (b: FwBoard) => b.present || inBoot(b);

export const bootRecover = (b: FwBoard) => inBoot(b) && b.verdict === 'hw_unknown';

export const boardUpdatable = (b: FwBoard) => !isMotorSlot(b.slot) && reachable(b)
  && (b.verdict === 'update' || b.verdict === 'downgrade' || bootRecover(b));

const hwA = (b: FwBoard): number | null => {
  if (!['ok', 'unsupported', 'unknown_d'].includes(b.hw.state)) return null;
  const a = Number(b.hw.ver?.split('.')[0]);
  return Number.isFinite(a) && a > 0 ? a : null;
};

export function variantFix(b: FwBoard): number | null {
  const a = b.file.a ?? hwA(b);
  return a && b.fwTargetA > 0 && a !== b.fwTargetA ? a : null;
}


export type FwTarget = 'all' | 'motors' | number;

export type FwMotorVerdict = 'latest' | 'update' | 'downgrade' | 'unmanaged' | 'legs_off' | 'unchecked' | 'check';
export type FwMotorGroup = {
  verdict: FwMotorVerdict;
  mixed: boolean;
  motors: FwBoard[];
  live: FwBoard[];
  todo: FwBoard[];
  issue: FwBoard | null;
};

const motorTodo = (m: FwBoard) => reachable(m)
  && (m.verdict === 'update' || m.verdict === 'same' || m.verdict === 'downgrade' || bootRecover(m));

export function motorGroup(s: FwStatus): FwMotorGroup {
  const motors = s.boards.filter((b) => isMotorSlot(b.slot));
  const live = motors.filter((m) => reachable(m) && m.verdict !== 'no_response');
  const todo = motors.filter(motorTodo);
  const issue = live.find((m) => m.verdict === 'no_file' || (m.verdict === 'hw_unknown' && !inBoot(m))) ?? null;
  const differ = (f: (m: FwBoard) => unknown) => new Set(live.map(f)).size > 1;
  const verdict: FwMotorVerdict =
    !motors.length || motors.some((m) => m.verdict === 'unmanaged') ? 'unmanaged'
    : !live.length ? 'legs_off'
    : issue ? 'check'
    : live.some((m) => m.verdict === 'unchecked') ? 'unchecked'
    : todo.some((m) => m.verdict === 'downgrade') ? 'downgrade'
    : todo.length ? 'update'
    : 'latest';
  return { verdict, mixed: differ((m) => m.verdict) || differ((m) => m.appVersion), motors, live, todo, issue };
}

export const motorsUpdatable = (g: FwMotorGroup) => g.verdict === 'update' || g.verdict === 'downgrade' || g.verdict === 'legs_off';

export function motorsCommon<T>(g: FwMotorGroup, f: (m: FwBoard) => T): T | null {
  const vs = new Set(g.live.map(f));
  return vs.size === 1 ? (g.live.map(f)[0] ?? null) : null;
}


export type FwSummary = {
  targets: FwBoard[];
  motors: FwMotorGroup;
  motorsGo: boolean;
  motorsLater: boolean;
  updatable: number;
  latest: number;
  needCheck: { name: string; board: FwBoard }[];
  noResponse: number;
  unmanaged: number;
};

export function summarize(s: FwStatus): FwSummary {
  const bodies = s.boards.filter((b) => !isMotorSlot(b.slot));
  const v = (...ks: FwVerdict[]) => bodies.filter((b) => ks.includes(b.verdict)).length;
  const g = motorGroup(s);
  const targets = bodies.filter(boardUpdatable);
  const motorsGo = g.todo.length > 0 && g.verdict !== 'check' && g.verdict !== 'unchecked';
  return {
    targets, motors: g, motorsGo, motorsLater: g.verdict === 'legs_off',
    updatable: targets.length + (motorsGo ? 1 : 0),
    latest: v('latest', 'same') + (g.verdict === 'latest' ? 1 : 0),
    needCheck: [
      ...bodies.filter((b) => b.verdict === 'no_file' || (b.verdict === 'hw_unknown' && !inBoot(b))).map((b) => ({ name: b.name, board: b })),
      ...(g.issue ? [{ name: t('모터'), board: g.issue }] : []),
    ],
    noResponse: v('no_response'),
    unmanaged: v('unmanaged') + (g.verdict === 'unmanaged' ? 1 : 0),
  };
}

export const neverChecked = (s: FwStatus) => s.checkSeq === 0 || s.boards.every((b) => b.verdict === 'unchecked');

export const fwChecking = (s: FwStatus) => s.run.state === 'checking' || s.busy === 'check';


const GAIT_FALL = -2, GAIT_CONTROL_OFF = -1, GAIT_SITTING = 0;

export const fwStopped = (gaitId: number, controlOn: boolean) =>
  gaitId === GAIT_SITTING || gaitId === GAIT_CONTROL_OFF || (gaitId === GAIT_FALL && !controlOn);

export type FwGuardInput = {
  connected: boolean;
  featureOn: boolean;
  status: FwStatus | null;
  gaitId: number | null;
  controlOn: boolean;
  otherOwner: boolean;
};
export type FwGuard = {
  blocked: boolean;
  reason: string;
  running: boolean;
  needPowerDown: boolean;
};

const ok = (needPowerDown: boolean): FwGuard => ({ blocked: false, reason: '', running: false, needPowerDown });
const no = (reason: string, running = false): FwGuard => ({ blocked: true, reason, running, needPowerDown: false });
const MOVING = () => t('구동 중입니다 — 로봇을 앉힌 뒤 다시 시도하세요');

export function fwCommonGuard(i: FwGuardInput): FwGuard {
  if (!i.connected) return no(t('로봇에 연결되어 있지 않습니다'));
  if (!i.featureOn) return no(t('로봇 소프트웨어가 이 기능을 지원하지 않습니다 — 소프트웨어 업데이트를 먼저 하세요'));
  if (!i.status) return no(t('보드 펌웨어 상태를 받는 중입니다'));
  if (!i.status.supported) return no(t('CAN-FD 로봇이 아니라 보드 펌웨어를 올릴 수 없습니다'));
  if (i.status.run.state === 'running') return no(t('업데이트가 진행 중입니다'), true);
  if (i.status.busy === 'update') return no(t('로봇이 펌웨어 작업을 하는 중입니다 — 끝난 뒤 다시 시도하세요'));
  if (fwChecking(i.status)) return no(t('버전을 확인하는 중입니다'));
  if (i.otherOwner) return no(t('다른 기기가 조종 중입니다'));
  if (i.gaitId == null) return no(t('로봇 상태를 받고 있지 않습니다'));
  if (!fwStopped(i.gaitId, i.controlOn)) return no(MOVING());
  if (i.controlOn && i.gaitId !== GAIT_SITTING) return no(MOVING());
  return ok(i.controlOn);
}

export function fwAllGuard(i: FwGuardInput): FwGuard {
  const g = fwCommonGuard(i);
  if (g.blocked || !i.status) return g;
  if (neverChecked(i.status)) return no(t('아직 버전을 확인하지 않았습니다 — [버전 확인]을 먼저 누르세요'));
  const sm = summarize(i.status);
  if (sm.needCheck.length) {
    const list = sm.needCheck.map((n) => `${n.name} — ${boardIssue(n.board)}`).join(' · ');
    return no(`${t('확인이 필요한 보드가 있습니다')}: ${list}`);
  }
  if (!sm.updatable && !sm.motorsLater) return no(t('업데이트할 보드가 없습니다'));
  return g;
}

export function fwTargets(target: FwTarget, s: FwStatus): FwBoard[] {
  if (target === 'motors') return motorsUpdatable(motorGroup(s)) ? motorGroup(s).todo : [];
  if (target !== 'all') return s.boards.filter((b) => b.slot === target && boardUpdatable(b));
  const sm = summarize(s);
  return [...sm.targets, ...(sm.motorsGo ? sm.motors.todo : [])];
}

function boardIssue(b: FwBoard): string {
  if (b.verdict === 'no_file') {
    const a = b.file.a ?? hwA(b);
    return a ? t('로봇에 이 보드(HW{a})에 맞는 파일이 없습니다').replace('{a}', String(a)) : t('로봇에 이 보드에 맞는 파일이 없습니다');
  }
  if (b.hw.state === 'blank') return t('hw 기록이 없어 맞는 파일을 정할 수 없습니다');
  if (b.hw.state === 'no_eeprom') return t('EEPROM이 응답하지 않아 hw를 모릅니다');
  if (b.hw.state === 'old_firmware') return t('옛 펌웨어라 hw를 읽지 못해 파일을 정할 수 없습니다');
  return t('hw를 몰라 맞는 파일을 정할 수 없습니다');
}

export function fwReadyText(sm: FwSummary): string {
  const parts: string[] = [];
  const go = [...sm.targets, ...(sm.motorsGo ? sm.motors.todo : [])];
  if (go.length) parts.push(`${t('올릴 보드')} — ${boardListText(go)}`);
  const boot = go.filter(bootRecover);
  if (boot.length) parts.push(t('{board}는 부트에 남아 있어 앱으로 보내 확인한 뒤 올립니다.').replace('{board}', shortNames(boot)));
  if (sm.motorsLater) parts.push(t('다리 전원이 꺼져 있어 모터는 업데이트하면서 다리 전원을 켜고 확인합니다'));
  return parts.join(' · ');
}


export type FwBanners = {
  update: FwBoard[];
  mixedMotors: boolean;
  hwRecord: FwBoard[];
  noEeprom: FwBoard[];
};

export function fwBanners(s: FwStatus | null): FwBanners {
  if (!s || s.run.state === 'running' || fwChecking(s)) return { update: [], mixedMotors: false, hwRecord: [], noEeprom: [] };
  const live = s.boards.filter(reachable);
  const standby = (st: FwHwState) => live.filter((b) => b.mode === 'standby' && b.hw.state === st);
  const g = motorGroup(s);
  const mixedMotors = g.mixed && g.verdict !== 'unmanaged';
  return {
    update: [...live.filter((b) => inBoot(b) && !(mixedMotors && isMotorSlot(b.slot))), ...standby('unsupported')].sort((a, b) => a.slot - b.slot),
    mixedMotors,
    hwRecord: standby('blank'),
    noEeprom: standby('no_eeprom'),
  };
}

export function shortNames(bs: FwBoard[], max = 3): string {
  const names = bs.map((b) => b.name);
  return names.length <= max ? names.join(', ')
    : `${names.slice(0, max).join(', ')} ${t('외 {n}').replace('{n}', String(names.length - max))}`;
}


export function boardListText(bs: FwBoard[]): string {
  const body = bs.filter((b) => !isMotorSlot(b.slot)).map((b) => b.name);
  const n = bs.filter((b) => isMotorSlot(b.slot)).length;
  return [...body, ...(n ? [t('모터({n}개)').replace('{n}', String(n))] : [])].join(', ');
}

export type FwAsk = {
  target: FwTarget;
  boards: FwBoard[];
  powerDown: boolean;
  motorsLater: boolean;
  checkSeq: number;
  changed?: boolean;
};

export function fwAsk(target: FwTarget, s: FwStatus, powerDown: boolean): FwAsk | null {
  const boards = fwTargets(target, s);
  const motorsLater = (target === 'all' || target === 'motors') && motorGroup(s).verdict === 'legs_off';
  if (!boards.length && !motorsLater) return null;
  return { target, boards, powerDown, motorsLater, checkSeq: s.checkSeq };
}

export function sameAsk(x: FwAsk, y: FwAsk): boolean {
  const key = (a: FwAsk) => `${a.target}|${a.powerDown}|${a.motorsLater}|${a.boards.map((b) => `${b.slot}:${b.verdict}:${b.file.version}:${b.file.a}:${b.appVersion}`).join(',')}`;
  return key(x) === key(y);
}

export function fwConfirmLines(a: FwAsk): string[] {
  const lines = [a.boards.length
    ? t('다음 보드를 업데이트합니다: {list}').replace('{list}', boardListText(a.boards))
    : t('모터를 확인해 새 버전이 있으면 올립니다.')];
  if (a.changed) lines.push(t('확인한 뒤 보드 상태가 바뀌었습니다 — 바뀐 내용을 다시 확인하세요.'));
  const boot = a.boards.filter(bootRecover);
  if (boot.length) lines.push(t('{board}는 부트에 남아 있어 앱으로 보내 확인한 뒤 올립니다.').replace('{board}', shortNames(boot)));
  for (const b of a.boards) {
    const va = variantFix(b);
    if (va) lines.push(t('{board}를 맞는 변형(HW{a})으로 바꿉니다.').replace('{board}', b.name).replace('{a}', String(va)));
  }
  if (a.motorsLater) lines.push(t('다리 전원이 꺼져 있어 모터는 업데이트하면서 다리 전원을 켜고 확인합니다.'));
  if (a.powerDown) lines.push(t('다리 또는 팔에 제어가 들어가 있습니다. 전원을 내리고 진행합니다.'));
  for (const b of a.boards.filter((x) => x.verdict === 'downgrade' && !isMotorSlot(x.slot))) {
    lines.push(t('{board}는 보드({cur})보다 낮은 버전({file})으로 내려갑니다.')
      .replace('{board}', b.name).replace('{cur}', fmtVer(b.appVersion)).replace('{file}', fmtVer(b.file.version)));
  }
  const motors = a.boards.filter((b) => isMotorSlot(b.slot));
  const down = motors.filter((m) => m.verdict === 'downgrade');
  if (down.length) {
    const cur = new Set(down.map((m) => m.appVersion)).size === 1 ? fmtVer(down[0].appVersion) : t('여러 버전');
    lines.push(t('모터 {n}개가 보드({cur})보다 낮은 버전({file})으로 내려갑니다.')
      .replace('{n}', String(down.length)).replace('{cur}', cur).replace('{file}', fmtVer(down[0].file.version)));
  }
  const same = motors.filter((m) => m.verdict === 'same').length;
  if (same) lines.push(t('모터 {n}개는 버전이 같아도 파일과 내용이 달라 다시 굽습니다 — 모터는 모두 같은 펌웨어여야 합니다.').replace('{n}', String(same)));
  return [...lines, ...powerLines(a)];
}

function powerLines(a: FwAsk): string[] {
  const slots = a.target === 'all' || a.target === 'motors' ? a.boards.map((b) => b.slot) : [a.target];
  const cut = a.target === 'all' || a.target === SLOT_PDU || a.powerDown;
  const motor = a.target === 'motors' || slots.some(isMotorSlot) || a.motorsLater;
  const out: string[] = [];
  if (cut) out.push(t('다리·팔 전원을 끄고 진행합니다. 끝나도 꺼진 채로 남으니 다시 기동하세요.'));
  if (motor) {
    out.push(cut ? t('모터를 올리는 동안에는 다리 전원을 켰다가 끝나면 다시 끕니다.')
      : t('모터를 올리는 동안 다리 전원을 켰다가 끝나면 끕니다. 끝나도 다리 전원은 꺼진 채로 남으니 다시 기동하세요.'));
  }
  if (!cut && !motor) out.push(t('다리·팔 전원은 건드리지 않습니다.'));
  if (slots.includes(SLOT_PDU)) out.push(t('PDU를 올리는 동안 UPC 전원이 잠시 꺼집니다.'));
  if (slots.includes(SLOT_TOP)) out.push(t('TOP을 올리는 동안 12V 포트가 잠시 꺼집니다.'));
  return out;
}

export const fmtVer = (v: number | null) => (v == null ? '—' : String(v));


export type FwRowState = 'done' | 'active' | 'pending' | 'failed' | 'skipped';
export type FwRunRow = {
  key: string;
  label: string;
  state: FwRowState;
  note?: string;
  pct?: number;
  retry?: number;
};

const STAGE_IDX: Record<FwRunStage, number> = { none: 0, precheck: 1, power_off: 2, boards: 3, leg_on: 4, motors: 5, leg_off: 6 };
export const FW_RETRY_MAX = 3;

export function stepLabel(st: FwStep): string {
  return {
    idle: t('대기'), to_app: t('앱으로 보내는 중'), read_hw: t('hw 확인 중'), enter_boot: t('부트 진입 중'),
    transfer: t('전송 중'), verify: t('끝 확인 중'), reboot: t('재부팅 확인 중'),
  }[st];
}

const pctOf = (b: FwBoard) => (b.progress.total > 0 ? Math.max(0, Math.min(100, Math.round((b.progress.sent / b.progress.total) * 100))) : 0);
const stepNote = (b: FwBoard) => (b.step === 'transfer' && b.progress.total > 0 ? `${stepLabel(b.step)} ${pctOf(b)}%` : stepLabel(b.step));

const runBoard = (s: FwStatus) => s.boards.find((b) => b.slot === s.run.board) ?? s.boards.find((b) => b.result !== 'none');

function skipNote(b: FwBoard): string {
  if (b.verdict === 'unmanaged') return t('대상 아님');
  if (b.verdict === 'same') return t('같은 버전이라 건너뜀');
  if (b.verdict === 'latest') return t('최신이라 건너뜀');
  if (!reachable(b) || b.verdict === 'no_response') return t('응답 없음 — 빼고 진행');
  return `${verdictLabel(b.verdict)} — ${t('건너뜀')}`;
}

export function fwRunRows(s: FwStatus, powerOffHint = false): FwRunRow[] {
  const run = s.run;
  if (run.kind === 'none' || !['running', 'success', 'failed'].includes(run.state)) return [];
  const running = run.state === 'running';
  const failed = run.state === 'failed';
  const cur = STAGE_IDX[run.stage];
  const by = (slot: number) => s.boards.find((b) => b.slot === slot);
  const motors = s.boards.filter((b) => isMotorSlot(b.slot));

  const stage = (key: FwRunStage, label: string): FwRunRow => {
    const idx = STAGE_IDX[key];
    let state: FwRowState;
    if (run.state === 'success') state = 'done';
    else if (running) state = cur > idx ? 'done' : cur === idx ? 'active' : 'pending';
    else if (key === 'leg_off') state = 'done';
    else state = idx < cur ? 'done' : idx === cur ? 'failed' : 'pending';
    return { key, label, state };
  };
  const board = (b: FwBoard): FwRunRow => {
    const row = { key: `b${b.slot}`, label: b.name };
    if (running && run.board === b.slot) {
      if (b.step === 'idle' && run.stage !== 'boards' && run.stage !== 'motors') return { ...row, state: 'pending' };
      return { ...row, state: 'active', note: stepNote(b), pct: b.step === 'transfer' && b.progress.total > 0 ? pctOf(b) : undefined, retry: b.retry || undefined };
    }
    if (b.result === 'success') return { ...row, state: 'done' };
    if (b.result === 'failed' || (failed && run.board === b.slot)) return { ...row, state: 'failed', retry: b.retry || undefined };
    if (b.result === 'skipped' || !reachable(b) || b.verdict === 'no_response' || b.verdict === 'unmanaged') return { ...row, state: 'skipped', note: skipNote(b) };
    return { ...row, state: run.state === 'success' ? 'skipped' : 'pending' };
  };
  const motorRow = (): FwRunRow => {
    const label = t('모터');
    const live = motors.filter((m) => reachable(m) && m.verdict !== 'no_response');
    const active = running ? motors.find((m) => m.slot === run.board) : undefined;
    const failedM = motors.find((m) => m.result === 'failed') ?? (failed ? motors.find((m) => m.slot === run.board) : undefined);
    const todo = motors.filter((m) => m.result === 'success' || m.result === 'failed' || m === active || (m.result === 'none' && motorTodo(m)));
    const done = motors.filter((m) => m.result === 'success').length;
    if (failedM) return { key: 'motors', label, state: 'failed', note: failedM.name, retry: failedM.retry || undefined };
    if (active) {
      const of = t('{n}개 중 {d}개 끝').replace('{n}', String(todo.length)).replace('{d}', String(done));
      return { key: 'motors', label, state: 'active', note: [active.name, stepNote(active), ...(todo.length > 1 ? [of] : [])].join(' · '),
        pct: active.step === 'transfer' && active.progress.total > 0 ? pctOf(active) : undefined, retry: active.retry || undefined };
    }
    if (run.state === 'success' && done > 0) return { key: 'motors', label, state: 'done', note: t('{n}개 올림').replace('{n}', String(done)) };
    if (!live.length) return { key: 'motors', label, state: 'pending', note: running ? t('다리 전원을 켠 뒤 확인합니다') : t('응답 없음') };
    if (!todo.length) return { key: 'motors', label, state: 'skipped', note: t('최신이라 건너뜀') };
    if (run.state === 'success' || (running && cur > STAGE_IDX.motors)) return { key: 'motors', label, state: 'done', note: t('{n}개 올림').replace('{n}', String(done)) };
    return { key: 'motors', label, state: 'pending', note: running ? t('{n}개 올릴 예정').replace('{n}', String(todo.length)) : undefined };
  };

  if (run.kind === 'all' || run.kind === 'force_all') {
    const bodies = FW_BODY_SLOTS.map((slot) => by(slot)).filter((b): b is FwBoard => !!b).map(board);
    const motorPart: FwRunRow[] = motors.length && motors.every((m) => m.verdict === 'unmanaged')
      ? [{ key: 'motors', label: t('모터'), state: 'skipped', note: t('대상 아님') }]
      : [stage('leg_on', t('다리 켜기')), motorRow()];
    return [stage('power_off', t('48V 끄기')), ...bodies, ...motorPart, stage('leg_off', t('다리 끄기'))];
  }
  const one = runBoard(s);
  if (!one) return [];
  const powerOff = one.slot === SLOT_PDU || powerOffHint || run.stage === 'power_off' ? [stage('power_off', t('48V 끄기'))] : [];
  if (isMotorSlot(one.slot)) return [...powerOff, stage('leg_on', t('다리 켜기')), run.kind === 'board' ? motorRow() : board(one), stage('leg_off', t('다리 끄기'))];
  return [...powerOff, board(one)];
}

export function runKindLabel(s: FwStatus): string {
  const one = runBoard(s);
  switch (s.run.kind) {
    case 'board': return one && isMotorSlot(one.slot) ? t('모터') : one?.name ?? '';
    case 'all': return t('전체');
    case 'force_all': return t('전체 강제');
    case 'force': return `${t('강제')} ${one?.name ?? ''}`.trim();
    default: return one?.name ?? '';
  }
}


export function fwReasonText(code: number, board: string, rejected = false): string {
  const b = (s: string) => s.replace('{board}', board || t('보드'));
  switch (code) {
    case 1: return t('CAN-FD 로봇이 아니라 보드 펌웨어를 올릴 수 없습니다');
    case 2: return t('다른 업데이트가 진행 중입니다. 끝난 뒤 다시 시도하세요.');
    case 3: return MOVING();
    case 4: return t('다리 또는 팔에 제어가 들어가 있어 전원을 내려야 합니다. 다시 시도하면 확인창에서 동의를 묻습니다.');
    case 5: return b(t('{board}의 hw를 몰라 맞는 파일을 정할 수 없습니다.'));
    case 6: return b(t('로봇에 {board}에 맞는 펌웨어 파일이 없습니다.'));
    case 7: return b(t('{board}는 이미 같은 버전입니다.'));
    case 8: return b(t('{board}는 낮은 버전으로 내려가므로 동의가 필요합니다. 확인창에서 동의한 뒤 다시 시도하세요.'));
    case 9: return b(t('{board}를 부트에서 앱으로 보냈지만 앱이 뜨지 않았고, hw를 몰라 맞는 파일도 정하지 못했습니다. 관리자에게 문의하세요.'));
    case 10: return b(t('{board} 전송이 시간 초과로 재시도 3번까지 모두 실패했습니다. 보드는 부트 상태로 두었습니다. 잠시 뒤 다시 업데이트하세요.'));
    case 11: return b(t('{board}에 쓴 내용이 파일과 달라(해시 불일치) 재시도 3번까지 모두 실패했습니다. 보드는 부트 상태로 두었습니다. 잠시 뒤 다시 업데이트하세요.'));
    case 12: return t('다리 전원을 켰지만 모터 보드가 하나도 응답하지 않습니다. 전원과 배선을 확인해 주세요.');
    case 13: return rejected
      ? b(t('{board}가 응답하지 않아 업데이트를 시작하지 못했습니다. 보드 전원과 배선을 확인한 뒤 [버전 확인]을 다시 누르세요.'))
      : b(t('{board}가 업데이트 중에 응답하지 않아 재시도 3번까지 모두 실패했습니다. 보드는 부트 상태로 두었습니다. 잠시 뒤 다시 업데이트하세요.'));
    case 14: return t('구운 펌웨어가 이 보드의 hw를 지원하지 않아 대기 상태입니다. 맞는 펌웨어로 다시 업데이트하세요.');
    default: return t('업데이트하지 못했습니다 (사유 {code})').replace('{code}', String(code));
  }
}

export function fwPowerLeft(s: FwStatus, powerOffHint = false): 'legs_arms' | 'legs' | null {
  const run = s.run;
  if (run.kind === 'none' || !['success', 'failed', 'rejected'].includes(run.state)) return null;
  const idx = STAGE_IDX[run.stage];
  if (run.state === 'rejected') return idx >= STAGE_IDX.leg_on ? (powerOffHint ? 'legs_arms' : 'legs') : null;
  const cutsFirst = run.kind === 'all' || run.kind === 'force_all' || runBoard(s)?.slot === SLOT_PDU;
  if (powerOffHint || (cutsFirst && (run.state === 'success' || idx >= STAGE_IDX.power_off))) return 'legs_arms';
  if (run.state === 'failed' || idx >= STAGE_IDX.leg_on) return 'legs';
  return null;
}

export type FwRunResultView = { ok: boolean; title: string; text: string; power: string };

export function fwRunResult(s: FwStatus, powerOffHint = false): FwRunResultView | null {
  const run = s.run;
  if (run.kind === 'none') return null;
  const left = fwPowerLeft(s, powerOffHint);
  const power = left === 'legs_arms' ? t('다리·팔 전원은 꺼진 채입니다 — 다시 기동하세요.')
    : left === 'legs' ? t('다리 전원은 꺼진 채입니다 — 다시 기동하세요.') : '';
  if (run.state === 'success') {
    const rows = fwRunRows(s, powerOffHint);
    const motorsDone = rows.some((r) => r.key === 'motors') ? s.boards.filter((b) => isMotorSlot(b.slot) && b.result === 'success').length : 0;
    const n = rows.filter((r) => /^b\d+$/.test(r.key) && r.state === 'done').length + motorsDone;
    return { ok: true, title: n ? t('업데이트 완료 — 보드 {n}개').replace('{n}', String(n)) : t('확인 완료 — 올릴 보드가 없었습니다'), text: '', power };
  }
  if (run.state !== 'failed' && run.state !== 'rejected') return null;
  const rejected = run.state === 'rejected';
  const title = rejected ? t('업데이트를 시작하지 못했습니다') : t('업데이트 실패');
  const all = run.kind === 'all' || run.kind === 'force_all';
  if (rejected && all && run.reason === 7) return { ok: false, title, text: t('올릴 보드가 없습니다 — 모든 보드가 이미 최신입니다.'), power };
  if (run.reason === 0) return { ok: false, title, text: t('업데이트 중 로봇에서 오류가 나 멈췄습니다. 로봇 로그를 확인한 뒤 다시 시도하세요.'), power };
  if (rejected && isMotorSlot(run.board) && run.kind !== 'force') {
    const text = run.reason === 8
      ? t('모터 중에 낮은 버전으로 내려가는 모터가 있어 동의가 필요합니다. 다리 전원을 켠 채 [버전 확인]을 누른 뒤 확인창에서 동의하세요.')
      : run.reason === 7 ? t('모터가 모두 이미 최신이라 굽지 않았습니다.')
      : fwReasonText(run.reason, t('모터'), true);
    return { ok: false, title: `${title} — ${t('모터')}`, text, power };
  }
  const pointed = s.boards.find((b) => b.slot === run.board);
  const same = s.boards.filter((b) => b.reason === run.reason);
  const who = same.length > 1 ? same : pointed ? [pointed] : same.length ? same : s.boards.filter((b) => b.result === 'failed');
  const name = shortNames(who);
  return { ok: false, title: name ? `${title} — ${name}` : title, text: fwReasonText(run.reason, name, rejected), power };
}


export function fwPostError(e: unknown): { text: string; needPowerDown: boolean; changed: boolean } {
  const status = (e as { status?: unknown } | null)?.status;
  const msg = e instanceof Error ? e.message : String(e ?? '');
  const say = (text: string, needPowerDown = false, changed = false) => ({ text, needPowerDown, changed });
  if (/firmware status changed/.test(msg)) return say(t('확인한 뒤 보드 상태가 바뀌었습니다 — 바뀐 내용을 다시 확인하세요.'), false, true);
  if (/power_down consent required/.test(msg)) return say(t('다리 또는 팔에 제어가 들어가 있습니다 — 전원 내림에 동의해야 업데이트할 수 있습니다'), true);
  if (/not owner/.test(msg) || status === 403) return say(t('다른 기기가 조종 중입니다'));
  if (/robot is moving/.test(msg)) return say(MOVING());
  if (/firmware update in progress/.test(msg)) return say(t('다른 업데이트가 진행 중입니다. 끝난 뒤 다시 시도하세요.'));
  if (/not a CAN FD robot/.test(msg)) return say(t('CAN-FD 로봇이 아니라 보드 펌웨어를 올릴 수 없습니다'));
  if (/motors are updated together/.test(msg)) return say(t('모터는 묶음으로만 업데이트합니다 — 모터 줄의 [업데이트]를 쓰세요'));
  if (status === 0) return say(t('로봇이 응답하지 않습니다 — 연결을 확인하세요'));
  return say(`${t('요청이 거절됐습니다')} — ${msg}`);
}


export function modeLabel(b: FwBoard): string {
  if (!reachable(b)) return t('없음');
  return { app: t('앱'), boot: t('부트'), boot_no_app: t('부트(앱 없음)'), standby: t('대기(STANDBY)'), unknown: t('모름') }[b.mode];
}

export function hwLabel(b: FwBoard): string {
  const ver = b.hw.ver ?? '—';
  switch (b.hw.state) {
    case 'ok': return ver;
    case 'unknown_d': return `${ver} · ${t('새 수정판(D)')}`;
    case 'unsupported': return `${ver} · ${t('지원 안 함')}`;
    case 'blank': return t('기록 없음');
    case 'no_eeprom': return t('EEPROM 무응답');
    case 'old_firmware': return t('옛 펌웨어');
    default: return '—';
  }
}

export function hwStateText(b: FwBoard): string {
  return {
    unchecked: t('확인 전'), ok: t('정상'), blank: t('기록 없음(모름)'), no_eeprom: t('EEPROM 무응답'),
    unsupported: t('지원 안 함 — 펌웨어가 이 hw를 모릅니다'), old_firmware: t('옛 펌웨어(hw를 묻지 못함)'),
    unknown_d: t('정상 — 펌웨어가 아는 것보다 새 수정판(D)'),
  }[b.hw.state];
}

export function verdictLabel(v: FwVerdict): string {
  return {
    unchecked: t('확인 전'), latest: t('최신'), update: t('업데이트 가능'), downgrade: t('다운그레이드'), same: t('같은 버전'),
    no_file: t('파일 없음'), hw_unknown: t('hw 모름'), no_response: t('응답 없음'), unmanaged: t('대상 아님'),
  }[v];
}

export function verdictTone(b: FwBoard): 'green' | 'blue' | 'orange' | 'red' | 'gray' {
  if (bootRecover(b)) return 'orange';
  return ({ latest: 'green', same: 'gray', update: 'blue', downgrade: 'orange', no_file: 'red', hw_unknown: 'red', no_response: 'gray', unchecked: 'gray', unmanaged: 'gray' } as const)[b.verdict];
}

export function modeTone(b: FwBoard): 'green' | 'orange' | 'red' | 'gray' {
  if (!reachable(b)) return 'gray';
  return ({ app: 'gray', boot: 'orange', boot_no_app: 'red', standby: 'orange', unknown: 'gray' } as const)[b.mode];
}

export function motorVerdictLabel(g: FwMotorGroup): string {
  if (g.verdict === 'check') return g.issue ? verdictLabel(g.issue.verdict) : t('확인 필요');
  return {
    latest: t('최신'), update: t('업데이트 가능'), downgrade: t('다운그레이드'), unmanaged: t('대상 아님'),
    legs_off: t('다리 꺼짐'), unchecked: t('확인 전'),
  }[g.verdict];
}

export function motorVerdictTone(g: FwMotorGroup): 'green' | 'blue' | 'orange' | 'red' | 'gray' {
  return ({ latest: 'green', update: 'blue', downgrade: 'orange', unmanaged: 'gray', legs_off: 'gray', unchecked: 'gray', check: 'red' } as const)[g.verdict];
}

export function motorModeLabel(g: FwMotorGroup): string {
  if (!g.live.length) return t('없음');
  const boot = g.live.filter(inBoot).length;
  if (boot) return t('부트 {n}개').replace('{n}', String(boot));
  const m = motorsCommon(g, (b) => b.mode);
  return m ? modeLabel(g.live[0]) : t('섞임');
}
