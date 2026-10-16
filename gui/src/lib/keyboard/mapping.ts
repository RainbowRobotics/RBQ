
function keysToStick(pressed: Set<string>, up: string, down: string, left: string, right: string) {
  let nx = (pressed.has(right) ? 1 : 0) - (pressed.has(left) ? 1 : 0);
  let ny = (pressed.has(up) ? 1 : 0) - (pressed.has(down) ? 1 : 0);
  const mag = Math.hypot(nx, ny);
  if (mag > 1) {
    nx /= mag;
    ny /= mag;
  }
  return { nx, ny };
}

export function keysToLeftStick(pressed: Set<string>): { nx: number; ny: number } {
  return keysToStick(pressed, 'KeyW', 'KeyS', 'KeyA', 'KeyD');
}

export function keysToRightStick(pressed: Set<string>): { nx: number; ny: number } {
  const p = new Set(pressed);
  if (pressed.has('KeyQ')) p.add('ArrowLeft');
  if (pressed.has('KeyE')) p.add('ArrowRight');
  return keysToStick(p, 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight');
}

const ZERO = { nx: 0, ny: 0 };

export function resolveFrame(
  armed: boolean,
  left: { nx: number; ny: number },
  right: { nx: number; ny: number },
): { l: { nx: number; ny: number }; r: { nx: number; ny: number } } {
  if (!armed) return { l: { ...ZERO }, r: { ...ZERO } };
  return { l: left, r: right };
}
