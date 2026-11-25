export const EE_SPEED_DEFAULTS = {move:30, rotate:20};
export function eeSpeedPercent(value:number, fallback:number) {
  return Number.isFinite(value)?Math.max(10,Math.min(100,Math.round(value))):fallback;
}
