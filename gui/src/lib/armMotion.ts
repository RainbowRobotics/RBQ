export function armMotionReady(stat: { conStart: boolean; lockPosition: boolean } | undefined) {
  return !!stat?.conStart && !stat.lockPosition;
}
