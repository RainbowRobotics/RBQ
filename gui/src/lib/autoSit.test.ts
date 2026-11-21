import { describe, expect, it, vi } from 'vitest';

vi.mock('./commandBus', () => ({ dcCommand: vi.fn() }));
import { fromSlider, toSlider, jointLimitKeys, jointKey, jointPatch, JOINT_INHERIT, JOINT_IGNORE } from './autoSit';

describe('autoSit slider mapping', () => {
  it('maps 0..100 onto the robot limits and snaps to the step', () => {
    expect(fromSlider(0, [0.5, 50], 0.5)).toBe(0.5);
    expect(fromSlider(100, [0.5, 50], 0.5)).toBe(50);
    expect(fromSlider(19.2, [0.5, 50], 0.5)).toBe(10);
    expect(fromSlider(150, [40, 100], 1)).toBe(100);
    expect(fromSlider(-5, [40, 100], 1)).toBe(40);
  });
  it('round-trips a value through the slider', () => {
    for (const v of [0.5, 10, 25.5, 50]) expect(fromSlider(toSlider(v, [0.5, 50]), [0.5, 50], 0.5)).toBe(v);
    for (const v of [40, 73, 100]) expect(fromSlider(toSlider(v, [40, 100]), [40, 100], 1)).toBe(v);
  });
});

describe('autoSit per-joint select', () => {
  it('offers 5 °C steps from the cap down to the floor', () => {
    expect(jointLimitKeys([40, 100])).toEqual(['100', '95', '90', '85', '80', '75', '70', '65', '60', '55', '50', '45', '40']);
  });
  it('maps state to a key and a key back to a patch', () => {
    expect(jointKey(false, null)).toBe(JOINT_INHERIT);
    expect(jointKey(false, 80)).toBe('80');
    expect(jointKey(true, 80)).toBe(JOINT_IGNORE);
    expect(jointPatch('motor', JOINT_IGNORE)).toEqual({ motor_ignored: true });
    expect(jointPatch('board', JOINT_INHERIT)).toEqual({ board_ignored: false, board_c: null });
    expect(jointPatch('motor', '70')).toEqual({ motor_ignored: false, motor_c: 70 });
  });
});
