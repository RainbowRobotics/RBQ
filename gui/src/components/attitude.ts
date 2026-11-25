export function horizonShift(pitchDeg: number, size: number): number {
  return -pitchDeg * (size / 60);
}
