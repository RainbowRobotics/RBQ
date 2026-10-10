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
