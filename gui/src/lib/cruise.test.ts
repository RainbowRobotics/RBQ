import { describe, it, expect, vi, beforeEach } from 'vitest';

const { sent, st, fake } = vi.hoisted(() => {
  const st = { settings: { gpCruise: true }, robot: { robot: { gait_id: 3 } as { gait_id: number } | null } };
  const fake = <T,>(get: () => T, set: (v: Partial<T>) => void) => ({ getState: get, setState: set });
  return { sent: [] as number[][], st, fake };
});
vi.mock('./userCommand', () => ({
  sendUserCommand: (_t: number, _c: number, _pc: number[], pi: number[]) => { sent.push(pi); },
}));
vi.mock('@/store/settings', () => ({ useSettings: fake(() => st.settings, (v) => Object.assign(st.settings, v)) }));
vi.mock('@/store/robot', () => ({ useRobot: fake(() => st.robot, (v) => Object.assign(st.robot, v)) }));

import { maybeCruise } from './cruise';
import { CRUISE } from './robotState';
import type { ButtonRole } from './gamepad/profiles';
import { useSettings } from '@/store/settings';
import { useRobot } from '@/store/robot';

const WEB: Record<number, ButtonRole> = { 11: 'R3', 12: 'DPAD_U', 13: 'DPAD_D' };
const ANDROID: Record<number, ButtonRole> = { 107: 'R3', 19: 'DPAD_U', 20: 'DPAD_D' };
const TROTTING = 3;

beforeEach(() => {
  sent.length = 0;
  useSettings.setState({ gpCruise: true });
  useRobot.setState({ robot: { gait_id: TROTTING } } as never);
});

describe('크루즈 단축키 — 플랫폼 무관', () => {
  const cases = [
    { name: '웹(스팀덱·데스크탑)', map: WEB, up: 12, down: 13, r3: 11 },
    { name: '안드로이드', map: ANDROID, up: 19, down: 20, r3: 107 },
  ];
  for (const c of cases) {
    it(`${c.name}: 십자키↑=증가, ↓=감소, R3=시작`, () => {
      expect(maybeCruise(c.map[c.up])).toBe(true);
      expect(maybeCruise(c.map[c.down])).toBe(true);
      expect(maybeCruise(c.map[c.r3])).toBe(true);
      expect(sent).toEqual([[CRUISE.INCREASE], [CRUISE.DECREASE], [CRUISE.START]]);
    });
  }

  it('크루즈 설정이 꺼져 있거나, 트롯 계열이 아니거나, 매핑 안 된 버튼이면 안 보낸다', () => {
    useSettings.setState({ gpCruise: false });
    expect(maybeCruise('DPAD_U')).toBe(false);
    useSettings.setState({ gpCruise: true });
    useRobot.setState({ robot: { gait_id: 1 } } as never);
    expect(maybeCruise('DPAD_U')).toBe(false);
    useRobot.setState({ robot: { gait_id: TROTTING } } as never);
    expect(maybeCruise(undefined)).toBe(false);
    expect(maybeCruise('A')).toBe(false);
    expect(sent).toEqual([]);
  });
});
