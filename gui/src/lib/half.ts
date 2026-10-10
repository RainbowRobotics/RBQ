
export function decodeHalf(u16: number): number {
  const sign = u16 & 0x8000 ? -1 : 1;
  const exp = (u16 >> 10) & 0x1f;
  const frac = u16 & 0x3ff;
  if (exp === 0) return sign * Math.pow(2, -14) * (frac / 1024);
  if (exp === 0x1f) return frac ? NaN : sign * Infinity;
  return sign * Math.pow(2, exp - 15) * (1 + frac / 1024);
}

export function getHalf(dv: DataView, off: number, le = true): number {
  return decodeHalf(dv.getUint16(off, le));
}

export function getHalves(dv: DataView, off: number, n: number, le = true): number[] {
  const out = new Array<number>(n);
  for (let i = 0; i < n; i++) out[i] = decodeHalf(dv.getUint16(off + i * 2, le));
  return out;
}
