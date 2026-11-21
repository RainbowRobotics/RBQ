export function pickInputMode(hasGamepad: boolean): 'gamepad' | 'touch' {
  return hasGamepad ? 'gamepad' : 'touch';
}

export function keyboardDrive(hasGamepad: boolean, isPc: boolean): boolean {
  return !hasGamepad && isPc;
}

export function isPcEnvironment(steamos = false): boolean {
  if (steamos) return false;
  if (typeof window === 'undefined' || typeof window.matchMedia !== 'function') return false;
  return window.matchMedia('(hover: hover) and (pointer: fine)').matches;
}

export function resolveHasPhysicalGamepad(
  devicesCount: number,
  gpUiMode: 'auto' | 'virtual' | 'gamepad',
  assumePad = false,
): boolean {
  return gpUiMode === 'auto' ? devicesCount > 0 || assumePad : gpUiMode === 'gamepad';
}
