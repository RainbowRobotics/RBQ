import type { ButtonRole } from '@/lib/gamepad/profiles';

let handler: ((role: ButtonRole) => void) | null = null;

export function oskCapturing(): boolean {
  return handler != null;
}

export function setOskButtonHandler(fn: ((role: ButtonRole) => void) | null) {
  handler = fn;
}

export function oskButton(role: ButtonRole) {
  handler?.(role);
}
