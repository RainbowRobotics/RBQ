import { beforeAll, beforeEach, describe, expect, it, vi } from 'vitest';

const h = vi.hoisted(() => ({
  drive: false,
  axes: null as null | ((e: { deviceId: number; axes: Record<string, number> }) => void),
  btn: null as null | ((e: { deviceId: number; keyCode: number; label: string; down: boolean; repeat: number }) => void),
  driveCb: null as null | ((on: boolean) => void),
}));
const dev = {
  id: 0, name: 'pad', vendorId: 0, productId: 0, descriptor: 'pad', sources: 0,
  keys: Array.from({ length: 17 }, (_, i) => i), axes: [],
};

vi.mock('@/lib/gamepad/input', () => ({
  gamepadInput: {
    available: true,
    getDevices: () => [dev],
    onDevices: () => () => {},
    onAxes: (cb: typeof h.axes) => { h.axes = cb; return () => {}; },
    onButton: (cb: typeof h.btn) => { h.btn = cb; return () => {}; },
  },
}));
vi.mock('@/lib/gamepad/profiles', async (orig) => {
  const m = await orig<typeof import('./profiles')>();
  return { ...m, matchProfile: m.baseProfile };
});
vi.mock('@/lib/connection', () => ({
  connection: { setAxes: vi.fn(), setTriggers: vi.fn(), setButtons: vi.fn() },
}));
vi.mock('@/lib/cruise', () => ({ maybeCruise: vi.fn() }));
vi.mock('@/lib/osk/capture', () => ({ oskCapturing: () => false, oskButton: vi.fn() }));
vi.mock('@/modules/registry', () => ({ activeRobotKind: () => null, gamepadCombos: [], gamepadGrab: { button: () => false, axes: () => false } }));
vi.mock('@/store/gamepad', () => ({
  useGamepad: { getState: () => ({ devices: [dev], setDevices: () => {}, setActive: () => {} }) },
}));
vi.mock('@/store/settings', () => ({
  useSettings: {
    getState: () => ({ gpSensitivity: 50, gpDeadzone: 0, gpOneStick: false, gpAllButtons: true }),
    subscribe: () => () => {},
  },
}));
vi.mock('@/store/inputMode', () => ({
  getPadDriveNow: () => h.drive,
  onPadDriveChange: (cb: (on: boolean) => void) => { h.driveCb = cb; return () => {}; },
}));

import { connection } from '@/lib/connection';
import { startGamepadManager, setPadBack, setPadNav } from './manager';

const back = vi.fn();
const nav = vi.fn();
const press = (keyCode: number, down: boolean) => h.btn!({ deviceId: 0, keyCode, label: '', down, repeat: 0 });
const stick = (x: number) => h.axes!({ deviceId: 0, axes: { 0: x, 1: 0, 2: 0, 3: 0 } });
const nonZero = (fn: unknown) =>
  (fn as ReturnType<typeof vi.fn>).mock.calls.some((c) => c.some((v) => (typeof v === 'number' && v !== 0) || (v instanceof Uint8Array && v.some(Boolean))));

beforeAll(() => { startGamepadManager(); setPadBack(back); setPadNav(nav); });
beforeEach(() => { vi.clearAllMocks(); stick(0); press(1, false); vi.clearAllMocks(); });

describe('게임패드 → 로봇 게이트', () => {
  it('조종 화면 밖: 스틱·버튼이 로봇에 안 나가고, B 는 뒤로다', () => {
    h.drive = false;
    stick(1);
    press(1, true);
    expect(nonZero(connection.setAxes)).toBe(false);
    expect(nonZero(connection.setButtons)).toBe(false);
    expect(back).toHaveBeenCalledTimes(1);
  });

  it('조종 화면: 스틱·버튼이 나간다', () => {
    h.drive = true;
    stick(1);
    press(1, true);
    expect(nonZero(connection.setAxes)).toBe(true);
    expect(nonZero(connection.setButtons)).toBe(true);
  });

  it('조종 화면 밖: 십자키·왼쪽 스틱은 포커스 이동, A 는 누르기 — 조종 화면에선 안 한다', () => {
    h.drive = false;
    press(13, true); press(13, false);
    press(0, true); press(0, false);
    stick(1); stick(0);
    expect(nav.mock.calls.map((c) => c[0])).toEqual(['down', 'press', 'right']);
    nav.mockClear();
    h.drive = true;
    press(13, true); press(13, false);
    stick(1); stick(0);
    expect(nav).not.toHaveBeenCalled();
  });

  it('조종 화면을 떠나는 순간 축·버튼을 0 으로 놓는다', () => {
    h.drive = true;
    stick(1);
    vi.clearAllMocks();
    h.drive = false;
    h.driveCb!(false);
    expect(connection.setAxes).toHaveBeenCalledWith('L', 0, 0);
    expect(connection.setButtons).toHaveBeenCalledWith(new Uint8Array(16));
  });
});
