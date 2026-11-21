import { t } from '@/lib/i18n';


export type Bi = { ko: string; en: string | null } | null | undefined;
export const pick = (b: Bi, en: boolean): string => (!b ? '' : en && b.en ? b.en : b.ko);

export type Level = 1 | 2 | 3;
export type Kind = 'basic' | 'aging' | 'rerun';
export const SOAK_CHOICES = [30, 120, 720] as const;
export type SoakMin = (typeof SOAK_CHOICES)[number];

export type LiveStep = { n: number; name: string; verdict: string; measured: string; moving?: boolean };

export type SoakLive = {
  state?: string; elapsedS?: number; plannedS?: number; verdict?: string; reasons?: string[]; brief?: string; baselineReady?: boolean;
  fb?: { min?: (number | null)[]; limit?: number; lowS?: number[] };
  track?: { max?: (number | null)[]; limit?: (number | null)[]; hardS?: number[]; runMaxS?: number };
  rtt?: { maxMs?: number | null; p99Ms?: number | null; limitMs?: number; latencyS?: number; lost?: number; linkDownS?: number };
  fps?: { measured?: boolean[]; min?: (number | null)[]; limit?: (number | null)[]; badS?: number[] };
  cur?: { maxA?: (number | null)[]; pitchCapA?: number; capState?: string; atCapS?: number; minuteMax?: (number | null)[][] };
};

export type EngineStatus = {
  state?: 'running' | 'done'; kind?: Kind; level?: number; presets?: boolean; startedAt?: string; updatedAt?: string;
  stage?: string; run?: string; serial?: string; uid?: string; stem?: string; stem2?: string | null;
  qc?: string | null; recheck?: string | null; stopping?: boolean;
  base?: string; picked?: number[]; ran?: number[];
  steps?: LiveStep[];
  soak?: SoakLive;
  gate?: { kind: 'ask' | 'interim'; remainS: number; fails?: { n: number; name: string; measured: string }[]; report?: string };
  ptz?: { yaw?: number; pitch?: number; drive?: 'on' | 'off' | 'fault'; homed?: boolean; rttP99?: number };
  robot?: { seen?: boolean; gait?: string; moving?: boolean; battery?: number | null; batteryMin?: number | null };
  done?: { result?: string; verdict?: string; reason?: string; stem?: string; category?: string };
};

export type DaemonInfo = {
  phase?: 'idle' | 'yielding' | 'running' | 'serial'; yielded?: boolean; lastCmdMs?: number; engineInstalled?: boolean;
  lan2can?: boolean; gatewayOpen?: boolean; lastError?: string;
  lastSerial?: { ok?: boolean; serial?: string; asked?: string; why?: string; at?: string };
};

export type QcStatus = {
  now?: number; daemonAlive?: boolean; daemon?: DaemonInfo; running?: boolean; phase?: string; engineFresh?: boolean;
  recentUseMs?: number | null; recentUseLimitMs?: number; status?: EngineStatus;
};

export type HistoryRow = {
  stem: string; startedAt: string; endedAt: string; serial: string; uid: string; kind: string; level?: number; soakMin?: number;
  verdict: string; why: string; whyBi?: Bi; versions?: { gw?: string; yaw?: string; pitch?: string };
  flagged?: { no: number; name: Bi; verdict: string; effective?: string; attempts?: number }[];
  stage: string; reruns: number; running: boolean;
};


export const STEP_NAMES = ['', '통신 응답', '식별 · 버전', '펌웨어 최신화', '드라이버 상태', '설정값 (EEPROM)', '전류 영점', '홈잉',
  '정지 유지', '지점 이동', '소프트 리밋', '프리셋 · EEPROM', '카메라 영상', '스피커 → 마이크', '카메라 경유 구동', '원위치',
  '게이트웨이 통신 부하', '진단 카운터', 'ONVIF 제어 · home 프리셋', '공회전(에이징)', '재검사'];
export const stepName = (n: number) => t(STEP_NAMES[n] ?? `${n}`);

const L1_SKIP = new Set([11, 18]);

export type PlanRow = { no: number; name: string; tag?: string; off?: boolean };
export function planRows(level: Level): PlanRow[] {
  const l1 = level === 1;
  return STEP_NAMES.slice(1).map((name, i) => {
    const no = i + 1;
    const row: PlanRow = { no, name: t(name) };
    if (no === 3) row.tag = t('기준 펌웨어가 로봇에 없어 건너뜀');
    if (no === 6) row.tag = l1 ? t('읽고 범위만') : t('보정');
    if (L1_SKIP.has(no)) { row.tag = l1 ? t('프리셋 — 안 함') : t('프리셋 동의 뒤'); row.off = l1; }
    if (no >= 19) { row.tag = l1 ? t('에이징 — 안 함') : t('에이징 검사만'); row.off = l1; }
    return row;
  });
}

export function plannedCount(level: Level, kind: Kind, picked?: number[]): number {
  if (kind === 'rerun') return withPrereqs(picked ?? [], level).length;
  if (level === 1) return 16;
  return kind === 'aging' ? 20 : 18;
}

export function runSteps(s: EngineStatus | undefined): LiveStep[] {
  const steps = s?.steps ?? [];
  if (s?.kind === 'rerun' && s.ran?.length) { const ran = new Set(s.ran); return steps.filter((x) => ran.has(x.n)); }
  if (s?.level === 1) return steps.filter((x) => !L1_SKIP.has(x.n));
  return steps;
}
const isDecided = (x: LiveStep) => !!x.verdict && x.verdict !== 'WAIT' && x.verdict !== 'RUN';
export function stepCount(s: EngineStatus | undefined): { done: number; total: number } {
  const run = runSteps(s);
  return { done: run.filter(isDecided).length, total: run.length };
}

export function withPrereqs(picked: number[], level: Level): number[] {
  const set = new Set<number>(picked.filter((n) => n >= 1 && n <= 18 && !(level === 1 && L1_SKIP.has(n))));
  if (!set.size) return [];
  if (set.has(18)) set.add(15);
  if (set.has(9) || set.has(10)) set.add(5);
  if (set.has(6)) { set.add(4); if (level === 1) set.add(5); }
  if ([8, 9, 10, 11, 14, 15, 18].some((n) => set.has(n))) set.add(7);
  set.add(1); set.add(2); set.add(17);
  return [...set].sort((a, b) => a - b);
}

export function prepNote(picked: number[], level: Level): string {
  const all = withPrereqs(picked, level);
  if (!all.length) return '';
  const extra = all.filter((n) => !picked.includes(n));
  const front = extra.filter((n) => n !== 7 && n !== 17);
  const parts = [`${t('앞에')} ${front.map((n) => `${n} ${stepName(n)}`).join(' · ')}`];
  if (extra.includes(7)) parts.push(t('홈이 안 돼 있으면 7 홈잉'));
  parts.push(`${t('끝에')} 17 ${stepName(17)}`);
  return `${t('같이 도는 준비 단계')} — ${parts.join(', ')}`;
}


export type Phase =
  | 'offline'
  | 'noDaemon'
  | 'idle'
  | 'starting'
  | 'serial'
  | 'qc'
  | 'ask'
  | 'interim'
  | 'soak'
  | 'recheck'
  | 'finishing';

export function phaseOf(s: QcStatus | null, sentAt = 0, now = Date.now()): Phase {
  if (!s) return 'offline';
  const st = s.status;
  const engineRun = st?.state === 'running' && (s.engineFresh || s.daemon?.phase === 'running');
  if (!s.daemonAlive && !engineRun) return 'noDaemon';
  const dp = s.daemon?.phase;
  if (dp === 'serial') return 'serial';
  if (dp === 'yielding') return 'starting';
  if (engineRun) {
    switch (st?.stage) {
      case 'connect': case 'prep': case 'home': case undefined: case '': return 'starting';
      case 'ask': return 'ask';
      case 'interim': return 'interim';
      case 'soak': return 'soak';
      case 'recheck': return 'recheck';
      case 'report': case 'final': case 'done': return 'finishing';
      default: return 'qc';
    }
  }
  if (dp === 'running' && st?.state === 'done') return 'finishing';
  if (dp === 'running') return 'starting';
  if (sentAt && now - sentAt < 4000) return 'starting';
  return 'idle';
}
export const isBusy = (p: Phase) => p !== 'offline' && p !== 'noDaemon' && p !== 'idle';

export function progressOf(s: EngineStatus | undefined): number {
  if (!s) return 0;
  const { done, total } = stepCount(s);
  const qc = total ? Math.min(1, done / total) : 0;
  if (s.kind !== 'aging') return qc;
  const soak = s.soak?.plannedS ? Math.min(1, (s.soak.elapsedS ?? 0) / s.soak.plannedS) : 0;
  if (s.stage === 'soak') return 0.25 + 0.7 * soak;
  if (s.stage === 'recheck') return 0.95 + 0.05 * qc;
  if (s.stage === 'final' || s.stage === 'done') return 1;
  return 0.25 * qc;
}

export function elapsedOf(s: EngineStatus | undefined): number | null {
  const a = parseTime(s?.startedAt), b = parseTime(s?.updatedAt);
  return a != null && b != null ? Math.max(0, (b - a) / 1000) : null;
}

export function currentStep(s: EngineStatus | undefined): LiveStep | null {
  const steps = s?.steps ?? [];
  return steps.find((x) => x.verdict === 'RUN') ?? null;
}

export function kindLabel(kind: string | undefined, soakMin?: number): string {
  if (kind === 'aging') return soakMin ? `${t('에이징')} ${durLabel(soakMin * 60)}` : t('에이징 검사');
  if (kind === 'rerun') return t('단계 다시');
  return t('기본 검사');
}

export function durLabel(sec: number | undefined | null): string {
  if (sec == null || !isFinite(sec)) return '—';
  const s = Math.max(0, Math.round(sec));
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), x = s % 60;
  if (h) return m ? `${h}${t('시간')} ${m}${t('분')}` : `${h}${t('시간')}`;
  if (m) return x && m < 10 ? `${m}${t('분')} ${x}${t('초')}` : `${m}${t('분')}`;
  return `${x}${t('초')}`;
}

export function clockLabel(sec: number | undefined | null): string {
  if (sec == null || !isFinite(sec)) return '—';
  const s = Math.max(0, Math.round(sec));
  const h = Math.floor(s / 3600), m = Math.floor((s % 3600) / 60), x = s % 60;
  const mm = String(m).padStart(h ? 2 : 1, '0'), ss = String(x).padStart(2, '0');
  return h ? `${h}:${mm}:${ss}` : `${mm}:${ss}`;
}

export function shortTime(at: string | undefined): string {
  const m = /^(\d{4})-(\d\d)-(\d\d)[ T](\d\d):(\d\d)/.exec(at ?? '');
  return m ? `${m[2]}-${m[3]} ${m[4]}:${m[5]}` : at || '—';
}
export function parseTime(at: string | undefined): number | null {
  const m = /^(\d{4})-(\d\d)-(\d\d)[ T](\d\d):(\d\d):(\d\d)/.exec(at ?? '');
  return m ? new Date(+m[1], +m[2] - 1, +m[3], +m[4], +m[5], +m[6]).getTime() : null;
}

export function recentUse(s: QcStatus | null): { text: string; warn: boolean } {
  const ms = s?.recentUseMs;
  if (ms == null) return { text: t('없음'), warn: false };
  return { text: `${durLabel(ms / 1000)} ${t('전')}`, warn: ms < (s?.recentUseLimitMs ?? 10000) };
}


export type Tone = 'green' | 'orange' | 'red' | 'gray' | 'blue';
export function verdictTone(v: string | undefined | null): Tone {
  switch (v) {
    case 'PASS': return 'green';
    case 'WARN': return 'orange';
    case 'FAIL': return 'red';
    case 'RUN': return 'blue';
    default: return 'gray';
  }
}
export function verdictWord(v: string | undefined | null): string {
  switch (v) {
    case 'PASS': case 'WARN': case 'FAIL': case 'SKIP': return v;
    case 'RUN': return t('진행 중');
    case 'WAIT': case '': case undefined: case null: return t('대기');
    case '무효': return t('무효');
    case '중지': return t('중지');
    default: return v;
  }
}

export const rerunNeedsPresets = (steps: number[] | undefined) => !!steps?.some((n) => L1_SKIP.has(n));

const MOVING = [7, 8, 9, 10, 11, 14, 15, 18];

export function startLines(level: Level, kind: Kind, steps?: number[]): string[] {
  const full = level >= 2;
  const all = kind === 'rerun' ? withPrereqs(steps ?? [], level) : null;
  const runs = (n: number) => !all || all.includes(n);
  const out: string[] = [];
  if (!all || MOVING.some((n) => all.includes(n))) out.push(t('축이 끝에서 끝까지 움직입니다 — YAW ±168.5°, PITCH −20~86°'));
  if (runs(13)) out.push(t('카메라 스피커에서 1 kHz · 2 kHz 삐 소리가 2초씩 납니다. 주변이 시끄러우면 13단계가 FAIL 날 수 있습니다'));
  if (full && runs(6)) out.push(t('전류 영점을 새로 재서 저장합니다 — 약 1.5초 PITCH 가 처질 수 있습니다'));
  if (kind === 'aging') out.push(t('공회전 동안 PITCH 전류를 3 A 로 잠시 낮춥니다 — 끝나면 풀립니다. 앱을 닫아도 로봇에서 계속 돕니다'));
  out.push(t('검사 동안 PTZ 는 조작할 수 없습니다'));
  return out;
}

export const PRESET_LINES = [
  '카메라 home — 있으면 지금 자리가 지워지고 0° 자리로 바뀝니다',
  '모터 254번 칸 — 검사에 잠깐 쓰고 지웁니다',
];

const REFUSE: Record<string, string> = {
  busy: '이미 검사가 도는 중입니다',
  no_daemon: 'PTZ 프로그램이 돌고 있지 않습니다 — PTZ 가 연결됐는지 확인해 주세요',
  not_lan2can: '이 PTZ 는 점검을 지원하지 않습니다(LAN2CAN 게이트웨이가 아님)',
  no_engine: '검사 프로그램이 이 로봇에 설치되지 않았습니다 — 로봇 소프트웨어를 업데이트해 주세요',
  no_aging_l1: '레벨 1 에는 에이징 검사가 없습니다',
  no_base: '붙일 보고서가 없습니다 — 결과 목록을 새로 고쳐 주세요',
  no_run: '도는 검사가 없습니다',
  serial_running: '시리얼을 쓰는 중이라 멈출 수 없습니다(30초 안에 끝납니다)',
  no_report: '보고서가 없습니다 — 아직 만드는 중일 수 있습니다',
  no_file: '파일이 없습니다',
  bad_serial: '시리얼은 영문 대문자 · 숫자 · 하이픈으로 1~16자입니다',
  spectator: '다른 사람이 조종하는 중이라 바꿀 수 없습니다(관전 중)',
};
export function refusalText(e: unknown): string {
  const err = e as { body?: { code?: string; error?: string }; message?: string };
  const code = err?.body?.code ?? (err?.body?.error === 'spectator' ? 'spectator' : undefined);
  if (code && REFUSE[code]) return t(REFUSE[code]);
  if (err?.body?.error) return err.body.error;
  return err?.message ?? String(e);
}

const DAEMON_ERR: [string, string][] = [
  ['이미 검사가 도는 중', REFUSE.busy],
  ['LAN2CAN', REFUSE.not_lan2can],
  ['설치되지 않음', REFUSE.no_engine],
  ['띄우지 못함', '검사 프로그램을 띄우지 못했습니다 — 로봇의 engine.log 를 확인해 주세요'],
  ['응답이 없어', '검사 프로그램이 응답하지 않아 로봇이 끊었습니다 — 로봇의 engine.log 를 확인해 주세요'],
  ['바로 끝남', '검사 프로그램이 시작하자마자 끝났습니다 — 로봇의 engine.log 를 확인해 주세요'],
];
export function daemonErrorText(e: string | undefined): string | null {
  if (!e || e.includes('시작하기 전에 멈춤')) return null;
  const hit = DAEMON_ERR.find(([k]) => e.includes(k));
  return hit ? t(hit[1]) : e;
}

export const serialOk = (s: string) => /^[A-Z0-9-]{1,16}$/.test(s);

export function flaggedText(r: HistoryRow, en: boolean): string {
  const f = r.flagged ?? [];
  if (!f.length) return '—';
  return f.slice(0, 3).map((x) => {
    const name = pick(x.name, en) || stepName(x.no);
    const tail = x.attempts && x.effective && x.effective !== x.verdict ? `${x.verdict}→${x.effective}` : x.verdict;
    return `${x.no} ${name} ${tail}`;
  }).join(' · ') + (f.length > 3 ? ' …' : '');
}

export function unseenFinished(rows: HistoryRow[], seen: string | null): HistoryRow | null {
  const r = rows.find((x) => !x.running);
  return r && r.stem !== seen ? r : null;
}


export function niceTicks(maxV: number, n = 5): number[] {
  if (!(maxV > 0)) return [0, 1];
  const raw = maxV / n;
  const p = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 5, 10].map((k) => k * p).find((s) => raw <= s) ?? 10 * p;
  const out: number[] = [];
  for (let v = 0; v <= maxV + step * 0.5; v += step) out.push(Math.round(v / step) * step);
  if (out[out.length - 1] < maxV) out.push(out[out.length - 1] + step);
  return out;
}

export function timeTicks(totalS: number): { s: number; label: string }[] {
  const steps = [600, 1800, 3600, 7200, 10800];
  const step = steps.find((x) => totalS / x <= 7) ?? 10800;
  const out: { s: number; label: string }[] = [];
  for (let s = 0; s <= totalS + 1; s += step) out.push({ s, label: s % 3600 === 0 ? `${s / 3600}${t('시간')}` : durLabel(s) });
  return out;
}
