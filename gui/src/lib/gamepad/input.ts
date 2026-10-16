import GamepadInput from '../../../modules/gamepad-input/src/GamepadInputModule';
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

export const gamepadInput = {
  available: GamepadInput != null,

  getDevices(): GamepadDeviceInfo[] {
    try { return GamepadInput?.getDevices() ?? []; } catch { return []; }
  },

  onDevices(cb: (e: GamepadDevicesEvent) => void): Unsub {
    const sub = GamepadInput?.addListener('onGamepadDevices', cb);
    return () => sub?.remove();
  },

  onAxes(cb: (e: GamepadAxesEvent) => void): Unsub {
    const sub = GamepadInput?.addListener('onGamepadAxes', cb);
    return () => sub?.remove();
  },

  onButton(cb: (e: GamepadButtonEvent) => void): Unsub {
    const sub = GamepadInput?.addListener('onGamepadButton', cb);
    return () => sub?.remove();
  },
};
