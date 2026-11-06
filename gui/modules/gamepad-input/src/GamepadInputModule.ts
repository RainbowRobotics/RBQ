import { NativeModule, requireOptionalNativeModule } from 'expo';

import type { GamepadDeviceInfo, GamepadInputEvents } from './GamepadInput.types';

declare class GamepadInputModule extends NativeModule<GamepadInputEvents> {
  getDevices(): GamepadDeviceInfo[];
}

export default requireOptionalNativeModule<GamepadInputModule>('GamepadInput');
