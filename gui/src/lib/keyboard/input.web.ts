const pressed = new Set<string>();
let armed = false;
let armCb: (armed: boolean) => void = () => {};

const CONTROL_KEYS = new Set(['KeyW', 'KeyA', 'KeyS', 'KeyD', 'KeyQ', 'KeyE', 'ArrowUp', 'ArrowDown', 'ArrowLeft', 'ArrowRight']);

function onKeyDown(e: KeyboardEvent) {
  const el = document.activeElement as HTMLElement | null;
  const editable = !!el && (el.tagName === 'INPUT' || el.tagName === 'TEXTAREA' || el.isContentEditable);
  if (editable) return;
  if (CONTROL_KEYS.has(e.code)) {
    e.preventDefault();
    pressed.add(e.code);
  }
}
function onKeyUp(e: KeyboardEvent) {
  pressed.delete(e.code);
}
function setArmed(next: boolean) {
  pressed.clear();
  if (armed === next) return;
  armed = next;
  armCb(armed);
}
function onFocus() { setArmed(true); }
function onBlur() { setArmed(false); }

export type RawInput = { pressed: Set<string>; armed: boolean };

export function startKeyboardListeners(onArmChange: (armed: boolean) => void): () => void {
  armCb = onArmChange;
  window.addEventListener('keydown', onKeyDown);
  window.addEventListener('keyup', onKeyUp);
  window.addEventListener('focus', onFocus);
  window.addEventListener('blur', onBlur);
  setArmed(document.hasFocus());
  return () => {
    window.removeEventListener('keydown', onKeyDown);
    window.removeEventListener('keyup', onKeyUp);
    window.removeEventListener('focus', onFocus);
    window.removeEventListener('blur', onBlur);
  };
}

export function getRawInput(): RawInput {
  return { pressed, armed };
}
