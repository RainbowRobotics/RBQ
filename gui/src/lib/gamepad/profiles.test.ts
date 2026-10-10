import { describe, it, expect } from 'vitest';
import { FRAME_INDEX, toDiagramKey, fromDiagramKey, type ButtonRole, type GamepadProfile } from './profiles';
import type { GamepadDeviceInfo } from './input';

const JOY_BTN = [
  'J_A', 'J_B', 'J_X', 'J_Y', 'J_LB', 'J_RB', 'J_BACK', 'J_START',
  'J_LOGI', 'J_LJOY', 'J_RJOY', 'J_AR_U', 'J_AR_D', 'J_AR_R', 'J_AR_L', 'J_SELECT',
] as const;

const ROLE_TO_JOY_BTN: Record<ButtonRole, (typeof JOY_BTN)[number]> = {
  A: 'J_A', B: 'J_B', X: 'J_X', Y: 'J_Y',
  L1: 'J_LB', R1: 'J_RB', BACK: 'J_BACK', START: 'J_START', GUIDE: 'J_LOGI',
  L3: 'J_LJOY', R3: 'J_RJOY',
  DPAD_U: 'J_AR_U', DPAD_D: 'J_AR_D', DPAD_L: 'J_AR_L', DPAD_R: 'J_AR_R',
  SELECT: 'J_SELECT',
};

describe('FRAME_INDEX', () => {
  it('모든 역할이 이름이 같은 JoyBTN 슬롯에 실린다', () => {
    for (const [role, symbol] of Object.entries(ROLE_TO_JOY_BTN)) {
      expect(FRAME_INDEX[role as ButtonRole], role).toBe(JOY_BTN.indexOf(symbol));
    }
  });
});

describe('toDiagramKey / fromDiagramKey', () => {
  const webProfile: GamepadProfile = {
    id: 'web-standard', label: '웹 표준', axes: { lx: 0, ly: 1, rx: 2, ry: 3 },
    buttons: { 0: 'A', 1: 'B', 4: 'L1', 12: 'DPAD_U', 16: 'GUIDE' },
  };
  const androidProfile: GamepadProfile = {
    id: 'android-standard', label: 'Android 표준', axes: { lx: 0, ly: 1, rx: 11, ry: 14 },
    buttons: { 96: 'A', 97: 'B', 19: 'DPAD_U' },
  };
  const webDev: GamepadDeviceInfo = {
    id: 0, name: 'web pad', vendorId: 0, productId: 0, descriptor: 'web', sources: 0,
    keys: Array.from({ length: 17 }, (_, i) => i), axes: [],
  };
  const androidDev: GamepadDeviceInfo = { ...webDev, descriptor: 'android', sources: 0x01000411, keys: [96, 97, 19] };

  it('웹은 인덱스→역할→키코드로 번역하고 역번역은 원시 인덱스로 돌아온다', () => {
    expect(toDiagramKey(webDev, webProfile, 0)).toBe(96);
    expect(toDiagramKey(webDev, webProfile, 12)).toBe(19);
    expect(toDiagramKey(webDev, webProfile, 6)).toBe(-1);
    expect(fromDiagramKey(webDev, webProfile, 96)).toBe(0);
    expect(fromDiagramKey(webDev, webProfile, 19)).toBe(12);
  });

  it('네이티브(Android 코드)는 그대로 통과한다', () => {
    expect(toDiagramKey(androidDev, androidProfile, 96)).toBe(96);
    expect(fromDiagramKey(androidDev, androidProfile, 19)).toBe(19);
  });
});
