export const WHEEL_GAIT_NAMES: Record<number, { en: string; ko: string }> = {
  45: { en: 'RL Wheel High Speed', ko: 'RL 휠 고속' },
  48: { en: 'RL Wheel Walk', ko: 'RL 휠 보행' },
  49: { en: 'RL Wheel Walk', ko: 'RL 휠 보행' },
};

export function wheelGaitName(id: number | undefined): { en: string; ko: string } | null {
  return id === undefined ? null : WHEEL_GAIT_NAMES[id] ?? null;
}
