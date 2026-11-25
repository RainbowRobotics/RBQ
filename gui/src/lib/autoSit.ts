import { dcCommand } from './commandBus';

export const AUTO_SIT_PATH = '/api/protection/auto_sit';

export type AutoSitJoint = {
  index: number;
  name: string;
  motor_ignored: boolean;
  board_ignored: boolean;
  motor_c: number | null;
  board_c: number | null;
};

export type AutoSitPolicy = {
  battery: { enabled: boolean; soc_percent: number; voltage: number };
  temperature: { enabled: boolean; motor_c: number; board_c: number; joints: AutoSitJoint[] };
};

export type AutoSitResp = {
  auto_sit: AutoSitPolicy;
  defaults: { soc_percent: number; voltage: number; motor_c: number; board_c: number };
  limits: { soc_percent: [number, number]; voltage: [number, number]; temp_c: [number, number] };
  pending?: boolean;
};

export type AutoSitPatch = {
  reset?: boolean;
  battery?: { enabled?: boolean; soc_percent?: number | null; voltage?: number | null };
  temperature?: {
    enabled?: boolean; motor_c?: number | null; board_c?: number | null;
    joints?: ({ index: number } & Partial<Omit<AutoSitJoint, 'index' | 'name'>>)[];
  };
};

export const autoSit = {
  get: () => dcCommand('GET', AUTO_SIT_PATH) as Promise<AutoSitResp>,
  put: (patch: AutoSitPatch) => dcCommand('PUT', AUTO_SIT_PATH, patch) as Promise<AutoSitResp>,
};


export function fromSlider(pos: number, [lo, hi]: [number, number], step: number): number {
  const v = lo + ((hi - lo) * Math.min(100, Math.max(0, pos))) / 100;
  const snapped = Math.round(v / step) * step;
  return Math.min(hi, Math.max(lo, Number(snapped.toFixed(3))));
}

export function toSlider(v: number, [lo, hi]: [number, number]): number {
  return hi > lo ? Math.min(100, Math.max(0, ((v - lo) * 100) / (hi - lo))) : 0;
}

export const JOINT_INHERIT = 'inherit';
export const JOINT_IGNORE = 'ignore';
export function jointLimitKeys([lo, hi]: [number, number]): string[] {
  const out: string[] = [];
  for (let v = Math.floor(hi / 5) * 5; v >= lo; v -= 5) out.push(String(v));
  return out;
}
export function jointKey(ignored: boolean, c: number | null): string {
  return ignored ? JOINT_IGNORE : c == null ? JOINT_INHERIT : String(c);
}
export function jointPatch(kind: 'motor' | 'board', key: string): Partial<Omit<AutoSitJoint, 'index' | 'name'>> {
  if (key === JOINT_IGNORE) return { [`${kind}_ignored`]: true };
  return { [`${kind}_ignored`]: false, [`${kind}_c`]: key === JOINT_INHERIT ? null : Number(key) };
}
