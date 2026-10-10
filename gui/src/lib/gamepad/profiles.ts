import type { GamepadDeviceInfo } from '@/lib/gamepad/input';

export type ButtonRole =
  | 'A' | 'B' | 'X' | 'Y' | 'L1' | 'R1' | 'BACK' | 'START' | 'GUIDE'
  | 'L3' | 'R3' | 'DPAD_U' | 'DPAD_D' | 'DPAD_L' | 'DPAD_R' | 'SELECT';

export type GamepadProfile = {
  id: string;
  label: string;
  axes: { lx: number; ly: number; rx: number; ry: number; hatX?: number; hatY?: number; lt?: number; rt?: number };
  invert?: Partial<Record<'lx' | 'ly' | 'rx' | 'ry', boolean>>;
  buttons: Record<number, ButtonRole>;
  presentKeys?: number[];
};

export const FRAME_INDEX: Record<ButtonRole, number> = {
  A: 0, B: 1, X: 2, Y: 3, L1: 4, R1: 5, BACK: 6, START: 7, GUIDE: 8,
  L3: 9, R3: 10, DPAD_U: 11, DPAD_D: 12, DPAD_L: 14, DPAD_R: 13, SELECT: 15,
};

export const SAFE_SLOTS: Partial<Record<ButtonRole, number>> = { L1: 8, R1: 10 };

export const KEY_LABEL: Record<number, string> = {
  96: 'A', 97: 'B', 98: 'C', 99: 'X', 100: 'Y', 101: 'Z',
  102: 'L1', 103: 'R1', 104: 'L2', 105: 'R2', 106: 'L3', 107: 'R3',
  108: 'ST', 109: 'SE', 110: 'MO', 19: '↑', 20: '↓', 21: '←', 22: '→',
};
export const keyLabel = (k: number) => KEY_LABEL[k] ?? `B${k}`;

export const ROLE_TO_KEYCODE: Partial<Record<ButtonRole, number>> = {
  A: 96, B: 97, X: 99, Y: 100, L1: 102, R1: 103, L3: 106, R3: 107,
  START: 108, SELECT: 109, GUIDE: 110, DPAD_U: 19, DPAD_D: 20, DPAD_L: 21, DPAD_R: 22,
};

export function toDiagramKey(dev: GamepadDeviceInfo, profile: GamepadProfile, raw: number): number {
  if (dev.sources !== 0) return raw;
  const role = profile.buttons[raw];
  return role ? ROLE_TO_KEYCODE[role] ?? -1 : -1;
}

export function fromDiagramKey(dev: GamepadDeviceInfo, profile: GamepadProfile, code: number): number {
  if (dev.sources !== 0) return code;
  const role = (Object.keys(ROLE_TO_KEYCODE) as ButtonRole[]).find((r) => ROLE_TO_KEYCODE[r] === code);
  const raw = role && Object.entries(profile.buttons).find(([, r]) => r === role)?.[0];
  return raw != null ? Number(raw) : code;
}

const ANDROID_STANDARD: GamepadProfile = {
  id: 'android-standard',
  label: 'Android 표준',
  axes: { lx: 0, ly: 1, rx: 11, ry: 14, hatX: 15, hatY: 16, lt: 17, rt: 18 },
  buttons: {
    96: 'A', 97: 'B', 99: 'X', 100: 'Y',
    102: 'L1', 103: 'R1',
    106: 'L3', 107: 'R3',
    108: 'START', 109: 'SELECT', 110: 'GUIDE',
    19: 'DPAD_U', 20: 'DPAD_D', 21: 'DPAD_L', 22: 'DPAD_R',
  },
};

const WEB_STANDARD: GamepadProfile = {
  id: 'web-standard',
  label: '웹 표준',
  axes: { lx: 0, ly: 1, rx: 2, ry: 3, lt: 106, rt: 107 },
  buttons: {
    0: 'A', 1: 'B', 2: 'X', 3: 'Y', 4: 'L1', 5: 'R1',
    8: 'SELECT', 9: 'START', 10: 'L3', 11: 'R3',
    12: 'DPAD_U', 13: 'DPAD_D', 14: 'DPAD_L', 15: 'DPAD_R', 16: 'GUIDE',
  },
};

export function matchProfile(dev: GamepadDeviceInfo): GamepadProfile {
  const { useGamepadProfiles } = require('@/store/gamepadProfiles') as typeof import('@/store/gamepadProfiles');
  const custom = useGamepadProfiles.getState().profiles[dev.descriptor];
  if (custom) return custom;
  const web = dev.sources === 0;
  if (dev.vendorId === 0x0483 && dev.productId === 0xa335) {
    return {
      ...ANDROID_STANDARD,
      id: 'uxv-navigator-tab3',
      label: 'UXV Navigator Tab 3',
      axes: { ...ANDROID_STANDARD.axes, lt: undefined, rt: undefined },
      presentKeys: [96, 97, 99, 100, 102, 103, 106, 107],
    };
  }
  return web ? WEB_STANDARD : ANDROID_STANDARD;
}
