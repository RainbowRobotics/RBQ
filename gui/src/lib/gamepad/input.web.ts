import type {
  GamepadAxesEvent,
  GamepadButtonEvent,
  GamepadDeviceInfo,
  GamepadDevicesEvent,
} from '../../../modules/gamepad-input/src/GamepadInput.types';

export type {
  GamepadAxesEvent,
  GamepadButtonEvent,
  GamepadDeviceInfo,
  GamepadDevicesEvent,
} from '../../../modules/gamepad-input/src/GamepadInput.types';

export type Unsub = () => void;

const hasApi = typeof window !== 'undefined' && typeof navigator !== 'undefined' && !!navigator.getGamepads;

const devCbs = new Set<(e: GamepadDevicesEvent) => void>();
const axesCbs = new Set<(e: GamepadAxesEvent) => void>();
const btnCbs = new Set<(e: GamepadButtonEvent) => void>();

function toInfo(gp: Gamepad): GamepadDeviceInfo {
  const m = /Vendor:\s*([0-9a-fA-F]{4}).*Product:\s*([0-9a-fA-F]{4})/.exec(gp.id);
  return {
    id: gp.index,
    name: gp.id,
    vendorId: m ? parseInt(m[1], 16) : 0,
    productId: m ? parseInt(m[2], 16) : 0,
    descriptor: gp.id,
    sources: 0,
    keys: gp.buttons.map((_, i) => i),
    axes: gp.axes.map((_, i) => ({ axis: i, label: `AXIS_${i}`, min: -1, max: 1, flat: 0 })),
  };
}

function pads(): Gamepad[] {
  return hasApi ? ([...navigator.getGamepads()].filter(Boolean) as Gamepad[]) : [];
}

function emitDevices() {
  const e = { devices: pads().map(toInfo) };
  devCbs.forEach((cb) => cb(e));
}

let rafId: number | null = null;
const prev = new Map<number, { axes: number[]; buttons: boolean[]; lt: number; rt: number }>();
const EPS = 0.002;

const TRIGGER_BUTTONS = { lt: 6, rt: 7 } as const;
const TRIGGER_AXIS_CODE = { lt: 106, rt: 107 } as const;

function loop() {
  for (const gp of pads()) {
    const p = prev.get(gp.index);
    const axes = [...gp.axes];
    const buttons = gp.buttons.map((b) => b.pressed || b.value > 0.5);
    const lt = gp.buttons[TRIGGER_BUTTONS.lt]?.value ?? 0;
    const rt = gp.buttons[TRIGGER_BUTTONS.rt]?.value ?? 0;
    const trigChanged = !p || Math.abs(lt - p.lt) > EPS || Math.abs(rt - p.rt) > EPS;
    if (trigChanged || !p || axes.some((v, i) => Math.abs(v - (p.axes[i] ?? 0)) > EPS)) {
      const rec: Record<string, number> = {};
      axes.forEach((v, i) => { rec[String(i)] = v; });
      rec[String(TRIGGER_AXIS_CODE.lt)] = lt;
      rec[String(TRIGGER_AXIS_CODE.rt)] = rt;
      const e = { deviceId: gp.index, axes: rec };
      axesCbs.forEach((cb) => cb(e));
    }
    buttons.forEach((down, i) => {
      if (down !== (p?.buttons[i] ?? false)) {
        const e = { deviceId: gp.index, keyCode: i, label: `BUTTON_${i}`, down, repeat: 0 };
        btnCbs.forEach((cb) => cb(e));
      }
    });
    prev.set(gp.index, { axes, buttons, lt, rt });
  }
  rafId = axesCbs.size + btnCbs.size > 0 ? requestAnimationFrame(loop) : null;
}

function ensureLoop() {
  if (hasApi && rafId == null && axesCbs.size + btnCbs.size > 0) rafId = requestAnimationFrame(loop);
}

if (hasApi) {
  window.addEventListener('gamepadconnected', emitDevices);
  window.addEventListener('gamepaddisconnected', (e: GamepadEvent) => {
    prev.delete(e.gamepad.index);
    emitDevices();
  });
}

function sub<T>(set: Set<T>, cb: T): Unsub {
  set.add(cb);
  ensureLoop();
  return () => { set.delete(cb); };
}

export const gamepadInput = {
  available: hasApi,
  getDevices(): GamepadDeviceInfo[] { return pads().map(toInfo); },
  onDevices(cb: (e: GamepadDevicesEvent) => void): Unsub { return sub(devCbs, cb); },
  onAxes(cb: (e: GamepadAxesEvent) => void): Unsub { return sub(axesCbs, cb); },
  onButton(cb: (e: GamepadButtonEvent) => void): Unsub { return sub(btnCbs, cb); },
};
