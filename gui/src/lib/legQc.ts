import { create } from 'zustand';
import { rest, actions } from './rest';
import { PROGRAM, PDU_PORT } from './robotState';
import { useRobot } from '@/store/robot';
import type { LegResultInfo, LegJointResult } from '@/types/robot';

const CMD = {
  INIT_CHECK_DEVICE: 100,
  INIT_AUTOSTART: 107,
  SETTING_SET_FOC_GAIN: 202,
  SET_CURRENT_LIMIT: 226,
  WALKREADY_LEG_QC: 200,
  WALKREADY_LEG_AGING: 201,
  WALKREADY_LEG_CHECK: 202,
} as const;

export const QC_JOINTS = ['Roll', 'Pitch', 'Knee'] as const;
export const BODY_TILT_LIMIT_DEG = 0.3;

export type QcStage = 1 | 2 | 3;
export const JUDGED_STAGES: QcStage[] = [1, 3];

export const STAGE_METRICS: Record<QcStage, { label: string; unit: string; info?: boolean }[]> = {
  1: [
    { label: '마찰 (빠른 구간)', unit: 'A' },
    { label: '비정규 리플 (느린 구간)', unit: 'A' },
  ],
  2: [],
  3: [
    { label: '부하 전류 계수', unit: 'A/m' },
    { label: 'I_rms', unit: 'A' },
    { label: 'E_rms', unit: 'deg' },
  ],
};

export const METRIC_HELP: Record<string, { see: string; judge: string; bad: string[] }> = {
  's1-0': {
    see: '추 없이 20 °/s 로 등속 왕복할 때, 같은 각도에서 올라갈 때와 내려갈 때 전류 차이의 절반. 피팅 없이 직접 잰 값입니다.',
    judge: '클 때만 봅니다. 기준 대비 약 +17.5% 경고, +30% 불량.',
    bad: ['조립할 때 너무 세게 조임', '부품끼리 간섭, 케이블 끌림', '베어링 손상, 그리스 부족', '작아지는 쪽은 판정하지 않습니다'],
  },
  's1-1': {
    see: '추 없이 5 °/s 로 돌 때, 피팅 잔차를 모터 1회전당 출렁임 횟수(차수)로 나눠서, 늘 있는 슬롯(12·24·36·48차)과 극(14·28차) 성분을 뺀 나머지의 크기.',
    judge: '클 때만 봅니다. 기준 대비 약 +70% 경고, +120% 불량. 정상 기어의 맞물림 성분도 들어 있어 원래 흔들림이 커서 문턱을 넓게 잡았습니다.',
    bad: ['기어 이빨 손상, 기어 사이 이물', '베어링 결함', '엔코더 이상', '슬롯·극 리플(모듈 json 참고값)만 크게 변했다면 모터 자체(자석·권선)'],
  },
  's3-0': {
    see: '발끝 추를 달고 코사인으로 움직일 때 I = c0 + k·h + fc·sign(속도) 로 맞춘 기울기 k. h 는 관절 축에서 발끝까지 수평 거리. 이상적으로는 추 무게 ÷ (토크 상수 × 기어비).',
    judge: '양쪽을 봅니다. 기준 대비 약 ±8.75% 경고, ±15% 불량. I_rms 보다 두 배 예민하게 잡았습니다.',
    bad: ['높음: 토크 상수 저하(자석 감자, 과열) — 같은 하중에 전류가 더 듦', '낮음: 추가 가볍거나 빠짐', 'I_rms 는 정상인데 k 만 벗어남: 고정한 다른 관절의 각도·홈 어긋남, 링크 길이 설정', '롤은 고정 자세가 약 11° 달라도 경고에 닿음', '가르는 법: 같은 모듈 1단계 grav_A 도 같이 올랐으면 모터 문제'],
  },
  's3-1': {
    see: '첫 주기를 뺀 4주기 동안 측정 전류를 그대로 제곱 평균한 뒤 제곱근. 평균을 빼지 않으므로 중력·마찰·치우침·리플이 모두 들어 있습니다.',
    judge: '양쪽을 봅니다. 기준 대비 약 ±17.5% 경고, ±30% 불량.',
    bad: ['높음 + 부하 전류 계수도 높음: 토크 상수 저하', '높음 + 부하 전류 계수 정상: 마찰·치우침 증가', '낮음: 추가 가볍거나 빠짐'],
  },
  's3-2': {
    see: '명령 각도와 실제 각도 차이의 RMS (관절 좌표, deg).',
    judge: '클 때만 봅니다. 기준 대비 약 +17.5% 경고, +30% 불량.',
    bad: ['마찰 증가로 같은 게인으로 못 따라감', '백래시', '엔코더·커플링 이상', '전류 제한에 걸려 토크가 모자람'],
  },
  verdict: {
    see: '그 다리의 항목 판정 중 가장 나쁜 것.',
    judge: 'z = (값 − 기준 중앙값) / 흩어짐, |z| 3.5 이상 WARN, 6 이상 FAIL. 에러 비트나 위치 튐이 있으면 항목과 상관없이 FAIL. 항목이 모두 PASS 여도 온도 이상이면 WARN. 기준 표본이 8줄 미만이면 기준 부족(측정 모드 2회 이상 필요).',
    bad: ['같은 관절 네 다리가 함께 틀어짐: 조건 문제(추 무게, 고정 자세, 기준 셋)', '한 다리만: 그 모듈', '같은 다리 세 관절이 함께: 각도·h 계산 쪽'],
  },
  temp: {
    see: '검사가 끝난 순간의 모터(고정자) / 보드 온도.',
    judge: '경고만 냅니다. 5 °C 미만 또는 110 °C 초과(센서 이상), 80 °C 초과, 네 다리 모터 온도 중앙값보다 15 °C 넘게 높으면 WARN.',
    bad: ['한 다리만 뜨거움: 마찰이 크거나 토크 상수 저하로 전류를 더 씀', '2 °C 처럼 말이 안 되는 값: 온도 센서·펌웨어 문제'],
  },
  err: {
    see: '검사 동안 보드가 한 번이라도 띄운 에러 비트와, 위치가 한 틱(2 ms)에 2° 넘게 튄 횟수.',
    judge: '1·3단계는 하나라도 있으면 그 다리가 FAIL 입니다. 2단계(에이징)는 판정 없이 표시만 합니다.',
    bad: ['JAM 모터 걸림', 'CUR 과전류', 'BIG 위치 오차 큼', 'INP 입력 과대', 'FLT FET 드라이버 고장', 'TMP 과열', 'PS1 / PS2 위치 한계 아래 / 위', '위치 튐: 엔코더 신호 불량, 통신 끊김'],
  },
  can: {
    see: '이 단계 동안 그 모듈이 붙은 CAN 채널에 쌓인 에러 프레임 수. 채널 단위라 같은 채널 모듈들이 같은 값을 공유합니다.',
    judge: '판정하지 않고 숫자만 보입니다. 0 이면 초록.',
    bad: ['움직일 때만 늘어남: 케이블·커넥터 접촉 불량(관절이 꺾일 때 끊김)', '종단 저항 문제', '노이즈'],
  },
};

export const REC_STATE = { NONE: 0, READING: 1, READ_OK: 2, NO_REPLY: 3, WRITING: 4, WRITE_OK: 5, WRITE_FAIL: 6 } as const;
export const recIndex = (stage: QcStage, k: number) => (stage === 1 ? k : 2 + k);
export function recDate(sec: number): string {
  const d = new Date(sec * 1000), p = (x: number) => String(x).padStart(2, '0');
  return `${p(d.getMonth() + 1)}-${p(d.getDate())} ${p(d.getHours())}:${p(d.getMinutes())}`;
}

export type StageResult = Omit<LegJointResult, 'joint'>;

type LegQcStore = {
  results: Record<QcStage, (StageResult | null)[]>;
  stage: QcStage;
  tab: number;
  seq: number;
  canTouched: boolean;
  rom: boolean;
  home: (boolean | null)[];
  homeBusy: number;
  homeErr: string;
  setTab: (i: number) => void;
  setStage: (s: QcStage) => void;
  ingest: (r: LegResultInfo) => void;
  reset: () => void;
};

const empty = () => ({
  results: { 1: [null, null, null], 2: [null, null, null], 3: [null, null, null] } as Record<QcStage, (StageResult | null)[]>,
  stage: 3 as QcStage, tab: 0,
  canTouched: false, rom: false, home: [null, null, null, null], homeErr: '',
});

export const useLegQc = create<LegQcStore>()((set, get) => ({
  ...empty(),
  seq: 0,
  homeBusy: -1,
  setTab: (tab) => set({ tab }),
  setStage: (stage) => set({ stage }),
  ingest: (r) => {
    const s = get();
    if (r.seq === s.seq || (r.stage !== 1 && r.stage !== 2 && r.stage !== 3) || !r.items?.length) return;
    const stage = r.stage as QcStage;
    const list = [...s.results[stage]];
    for (const { joint, ...rest } of r.items) if (joint >= 0 && joint <= 2) list[joint] = rest;
    set({ results: { ...s.results, [stage]: list }, stage, tab: r.items.length === 1 ? r.items[0].joint : s.tab, seq: r.seq });
  },
  reset: () => set(empty()),
}));

const ip = () => useRobot.getState().ip;
const motion = (cmd: number, para?: { char?: number[]; int?: number[] }) => rest.commandStruct(ip(), PROGRAM.Motion, cmd, para);
const sleep = (ms: number) => new Promise((r) => setTimeout(r, ms));

export const legQc = {
  start: (mode: 0 | 1) => rest.commandStruct(ip(), PROGRAM.WalkReady, CMD.WALKREADY_LEG_QC, { char: [0, mode] }),
  stop: () => rest.commandStruct(ip(), PROGRAM.WalkReady, CMD.WALKREADY_LEG_QC, { char: [4] }),
  saveRecord: (mask: number) => rest.commandStruct(ip(), PROGRAM.WalkReady, CMD.WALKREADY_LEG_QC, { char: [5], int: [mask] }),
  checkStart: (mode: 0 | 1) => rest.commandStruct(ip(), PROGRAM.WalkReady, CMD.WALKREADY_LEG_CHECK, { char: [0, mode] }),
  checkStop: () => rest.commandStruct(ip(), PROGRAM.WalkReady, CMD.WALKREADY_LEG_CHECK, { char: [4] }),
  agingStart: () => rest.commandStruct(ip(), PROGRAM.WalkReady, CMD.WALKREADY_LEG_AGING, { char: [0] }),
  agingStop: () => rest.commandStruct(ip(), PROGRAM.WalkReady, CMD.WALKREADY_LEG_AGING, { char: [4] }),
  init: () => motion(CMD.INIT_AUTOSTART, { char: [0, 3] }),
  legPower: (on: boolean) => actions.pduPower(ip(), PDU_PORT.LEG_48V, on),
  canCheck: () => motion(CMD.INIT_CHECK_DEVICE, { char: [-1] }),
  romSetting: async () => {
    await motion(CMD.SETTING_SET_FOC_GAIN, { char: [-1], int: [7000, 80] });
    await sleep(3000);
    await motion(CMD.SET_CURRENT_LIMIT, { char: [-1, 0], int: [20000, 40000, 5, 2] });
    await sleep(3000);
    await motion(CMD.SET_CURRENT_LIMIT, { char: [-1, 2], int: [40000, 40000] });
    await sleep(500);
  },
  legHome: async (leg: number): Promise<boolean> => {
    const waitIdle = async () => {
      for (let i = 0; i < 100; i++) {
        await sleep(300);
        const st = await rest.legHomeSet(ip());
        if (!st.running) return st;
      }
      throw new Error('timeout');
    };
    await actions.legHomeSetStart(ip(), leg, 'roll');
    await waitIdle();
    await actions.legHomeSetStart(ip(), leg, 'pitch_knee');
    const st = await waitIdle();
    return st.legs.find((l) => l.leg === leg)?.ok === true;
  },
};
