import { Platform } from 'react-native';
import { keysToLeftStick, keysToRightStick, resolveFrame } from './mapping';
import { startKeyboardListeners, getRawInput } from './input';
import { applyStickCurve } from '@/lib/gamepad/curve';
import { connection } from '@/lib/connection';
import { getKeyboardDriveNow } from '@/store/inputMode';

const DEADZONE = 0;
const SENSITIVITY = 50;

let started = false;
let ownedAxes = false;

function frame() {
  const raw = getRawInput();
  if (!getKeyboardDriveNow()) {
    if (raw.armed && ownedAxes) {
      connection.setAxes('L', 0, 0);
      connection.setAxes('R', 0, 0);
      ownedAxes = false;
    }
    requestAnimationFrame(frame);
    return;
  }

  const left = keysToLeftStick(raw.pressed);
  const right = keysToRightStick(raw.pressed);
  const resolved = resolveFrame(raw.armed, left, right);
  if (raw.armed && raw.pressed.size === 0) {
    if (ownedAxes) { connection.setAxes('L', 0, 0); connection.setAxes('R', 0, 0); ownedAxes = false; }
  } else if (raw.armed) {
    const l = applyStickCurve(resolved.l.nx, resolved.l.ny, DEADZONE, SENSITIVITY);
    const r = applyStickCurve(resolved.r.nx, resolved.r.ny, DEADZONE, SENSITIVITY);
    connection.setAxes('L', l.x, l.y);
    connection.setAxes('R', r.x, r.y);
    ownedAxes = true;
  }

  requestAnimationFrame(frame);
}

export function startKeyboardManager(): void {
  if (started) return;
  if (typeof window === 'undefined' || typeof requestAnimationFrame === 'undefined') return;
  if (Platform.OS !== 'web') return;
  started = true;
  startKeyboardListeners((armed) => {
    if (!armed) {
      connection.setAxes('L', 0, 0);
      connection.setAxes('R', 0, 0);
    }
  });
  requestAnimationFrame(frame);
}
