
export type GamepadAxisRange = {
  axis: number;
  label: string;
  min: number;
  max: number;
  flat: number;
};

export type GamepadDeviceInfo = {
  id: number;
  name: string;
  vendorId: number;
  productId: number;
  descriptor: string;
  sources: number;
  keys: number[];
  axes: GamepadAxisRange[];
};

export type GamepadDevicesEvent = { devices: GamepadDeviceInfo[] };
export type GamepadAxesEvent = { deviceId: number; axes: Record<string, number> };
export type GamepadButtonEvent = {
  deviceId: number;
  keyCode: number;
  label: string;
  down: boolean;
  repeat: number;
};

export type GamepadInputEvents = {
  onGamepadDevices: (e: GamepadDevicesEvent) => void;
  onGamepadAxes: (e: GamepadAxesEvent) => void;
  onGamepadButton: (e: GamepadButtonEvent) => void;
};
