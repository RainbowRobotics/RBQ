import { gamepadInput, type GamepadDeviceInfo } from '@/lib/gamepad/input';
import { applyStickCurve } from '@/lib/gamepad/curve';
import { matchProfile, FRAME_INDEX, SAFE_SLOTS, type ButtonRole, type GamepadProfile } from '@/lib/gamepad/profiles';
import { connection } from '@/lib/connection';
import { maybeCruise } from '@/lib/cruise';
import { oskCapturing, oskButton } from '@/lib/osk/capture';
import { activeRobotKind, gamepadCombos } from '@/modules/registry';
import { useGamepad } from '@/store/gamepad';
import { useGamepadBindings } from '@/store/gamepadBindings';
import { useSettings } from '@/store/settings';

let started = false;
let active: { dev: GamepadDeviceInfo; profile: GamepadProfile } | null = null;
const roleDown = new Map<ButtonRole, boolean>();
const rawDown = new Set<number>();
const live = { lx: 0, ly: 0, rx: 0, ry: 0, hatX: 0, hatY: 0 };
let lastTriggerL = 0, lastTriggerR = 0;
let rawTriggerL = 0, rawTriggerR = 0;

export function getLiveAxes() {
  return { ...live };
}
export function clearGamepadOutputs() {
  clearOutputs();
  hatPrev.x = hatPrev.y = 0;
}

const swallowedKeys = new Set<number>();
const swallowedRoles = new Set<ButtonRole>();

export function getPressedRoles(): ButtonRole[] {
  return [...roleDown].filter(([, d]) => d).map(([r]) => r);
}
export function refreshProfile() {
  active = null;
  const devices = useGamepad.getState().devices;
  pickActive(devices);
}
export function getPressedKeys(): number[] {
  return [...rawDown];
}


function clearOutputs() {
  roleDown.clear();
  rawDown.clear();
  live.lx = live.ly = live.rx = live.ry = live.hatX = live.hatY = 0;
  connection.setAxes('L', 0, 0);
  connection.setAxes('R', 0, 0);
  connection.setButtons(new Uint8Array(16));
  lastTriggerL = lastTriggerR = 0;
  rawTriggerL = rawTriggerR = 0;
  connection.setTriggers(0, 0);
}

function pickActive(devices: GamepadDeviceInfo[]) {
  const dev = devices[0] ?? null;
  if (!dev) {
    if (active) clearOutputs();
    active = null;
    useGamepad.getState().setActive(null);
    return;
  }
  if (active && active.dev.id === dev.id) { active.dev = dev; return; }
  if (active) clearOutputs();
  const profile = matchProfile(dev);
  active = { dev, profile };
  roleDown.clear();
  useGamepad.getState().setActive({ name: dev.name, profileLabel: profile.label });
}

function setRole(role: ButtonRole, down: boolean) {
  if ((roleDown.get(role) ?? false) === down) return false;
  roleDown.set(role, down);
  return true;
}

function runBinding(keyCode: number) {
  if (activeRobotKind()) return;
  const bind = useGamepadBindings.getState().bindings.find((b) => b.keyCode === keyCode);
  if (bind?.action.type === 'motion') connection.sendMotion(bind.action.motion);
}

function setHat(role: ButtonRole, vKey: number, down: boolean): boolean {
  if (swallowedRoles.has(role)) {
    roleDown.set(role, down);
    if (!down) swallowedRoles.delete(role);
    return false;
  }
  const changed = setRole(role, down);
  if (changed) {
    if (down) { rawDown.add(vKey); runBinding(vKey); }
    else { rawDown.delete(vKey); maybeCruise(role); }
  }
  return changed;
}

const hatPrev = { x: 0, y: 0 };
function hatToOsk(axes: Record<string, number>) {
  const p = active?.profile.axes;
  if (!p) return;
  const g = (code?: number) => (code == null ? 0 : axes[String(code)] ?? 0);
  const x = g(p.hatX), y = g(p.hatY);
  const edge = (role: ButtonRole) => { swallowedRoles.add(role); oskButton(role); };
  if (x < -0.5 && hatPrev.x >= -0.5) edge('DPAD_L');
  if (x > 0.5 && hatPrev.x <= 0.5) edge('DPAD_R');
  if (y < -0.5 && hatPrev.y >= -0.5) edge('DPAD_U');
  if (y > 0.5 && hatPrev.y <= 0.5) edge('DPAD_D');
  if (x >= -0.5) swallowedRoles.delete('DPAD_L');
  if (x <= 0.5) swallowedRoles.delete('DPAD_R');
  if (y >= -0.5) swallowedRoles.delete('DPAD_U');
  if (y <= 0.5) swallowedRoles.delete('DPAD_D');
  hatPrev.x = x; hatPrev.y = y;
}

function pushButtons() {
  const out = new Uint8Array(16);
  const { gpAllButtons } = useSettings.getState();
  const kind = activeRobotKind();
  for (const [role, down] of roleDown) {
    if (!down) continue;
    const idx = kind ? kind.frameIndex[role] : gpAllButtons ? FRAME_INDEX[role] : SAFE_SLOTS[role];
    if (idx != null) out[idx] = 1;
  }
  connection.setButtons(out);
}

export function startGamepadManager() {
  if (started || !gamepadInput.available) return;
  started = true;

  const sync = (devices: GamepadDeviceInfo[]) => {
    useGamepad.getState().setDevices(devices);
    pickActive(devices);
  };
  sync(gamepadInput.getDevices());
  gamepadInput.onDevices((e) => sync(e.devices));

  let prevGpAllButtons = useSettings.getState().gpAllButtons;
  useSettings.subscribe((state) => {
    if (state.gpAllButtons === prevGpAllButtons) return;
    prevGpAllButtons = state.gpAllButtons;
    if (state.gpAllButtons) {
      lastTriggerL = rawTriggerL; lastTriggerR = rawTriggerR;
      connection.setTriggers(rawTriggerL, rawTriggerR);
    } else {
      lastTriggerL = 0; lastTriggerR = 0;
      connection.setTriggers(0, 0);
    }
  });

  gamepadInput.onAxes((e) => {
    if (!active || e.deviceId !== active.dev.id) return;
    if (oskCapturing()) { hatToOsk(e.axes); return; }
    const p = active.profile.axes;
    const g = (code?: number) => (code == null ? 0 : e.axes[String(code)] ?? 0);
    const { gpSensitivity, gpDeadzone, gpOneStick, gpAllButtons } = useSettings.getState();
    const inv = active.profile.invert;
    let lxRaw = inv?.lx ? -g(p.lx) : g(p.lx);
    let rxRaw = inv?.rx ? -g(p.rx) : g(p.rx);
    let lyRaw = inv?.ly ? g(p.ly) : -g(p.ly);
    let ryRaw = inv?.ry ? g(p.ry) : -g(p.ry);
    if (gpOneStick) { const t = lxRaw; lxRaw = rxRaw; rxRaw = t; }
    const L = applyStickCurve(lxRaw, lyRaw, gpDeadzone, gpSensitivity);
    const R = applyStickCurve(rxRaw, ryRaw, gpDeadzone, gpSensitivity);
    live.lx = L.x; live.ly = L.y; live.rx = R.x; live.ry = R.y;
    connection.setAxes('L', L.x, L.y);
    connection.setAxes('R', R.x, R.y);
    const clamp01 = (v: number) => Math.min(1, Math.max(0, v));
    rawTriggerL = clamp01(g(p.lt));
    rawTriggerR = clamp01(g(p.rt));
    const fullPad = gpAllButtons || !!activeRobotKind();
    const trigL = fullPad ? rawTriggerL : 0;
    const trigR = fullPad ? rawTriggerR : 0;
    if (trigL !== lastTriggerL || trigR !== lastTriggerR) {
      lastTriggerL = trigL; lastTriggerR = trigR;
      connection.setTriggers(trigL, trigR);
    }
    let changed = false;
    if (p.hatX != null) {
      const v = g(p.hatX);
      live.hatX = v;
      changed = setHat('DPAD_L', 21, v < -0.5) || changed;
      changed = setHat('DPAD_R', 22, v > 0.5) || changed;
    }
    if (p.hatY != null) {
      const v = g(p.hatY);
      live.hatY = v;
      changed = setHat('DPAD_U', 19, v < -0.5) || changed;
      changed = setHat('DPAD_D', 20, v > 0.5) || changed;
    }
    if (changed) pushButtons();
  });

  gamepadInput.onButton((e) => {
    if (!active || e.deviceId !== active.dev.id) return;
    if (oskCapturing()) {
      if (e.down) {
        swallowedKeys.add(e.keyCode);
        const r = active.profile.buttons[e.keyCode];
        if (r) oskButton(r);
      } else swallowedKeys.delete(e.keyCode);
      return;
    }
    if (!e.down && swallowedKeys.delete(e.keyCode)) return;
    if (e.down) rawDown.add(e.keyCode); else rawDown.delete(e.keyCode);
    if (e.down && e.repeat === 0) runBinding(e.keyCode);
    const role = active.profile.buttons[e.keyCode];
    if (!e.down) maybeCruise(role);
    if (!role) return;
    if (setRole(role, e.down)) {
      pushButtons();
      if (e.down) {
        if ((role === 'L1' || role === 'L3') && roleDown.get('L1') && roleDown.get('L3')) gamepadCombos.forEach((f) => f('L'));
        if ((role === 'R1' || role === 'R3') && roleDown.get('R1') && roleDown.get('R3')) gamepadCombos.forEach((f) => f('R'));
      }
    }
  });
}
