
export function fmtRate(bytesPerSec: number): string {
  const kb = bytesPerSec / 1024;
  if (Math.round(kb * 10) / 10 >= 1024) return `${(kb / 1024).toFixed(1)} MB/s`;
  if (bytesPerSec >= 1024) return `${kb.toFixed(1)} KB/s`;
  return `${Math.round(bytesPerSec)} B/s`;
}
