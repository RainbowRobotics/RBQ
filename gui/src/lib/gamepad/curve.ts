export function applyStickCurve(
  nx: number,
  ny: number,
  deadzone: number,
  sensitivity: number,
): { x: number; y: number } {
  const mag = Math.min(1, Math.hypot(nx, ny));
  const dz = deadzone / 100;
  if (mag <= dz) return { x: 0, y: 0 };
  const t = (mag - dz) / (1 - dz);
  const expo = 1 + (50 - sensitivity) / 60;
  const outMag = Math.pow(t, Math.max(0.2, expo));
  const ang = Math.atan2(ny, nx);
  return { x: Math.cos(ang) * outMag, y: Math.sin(ang) * outMag };
}
