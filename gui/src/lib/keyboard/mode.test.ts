import { describe, it, expect } from 'vitest';
import { pickInputMode, keyboardDrive, resolveHasPhysicalGamepad } from './mode';

describe('pickInputMode — 게임패드 > 터치(PC 도 가상 조이스틱 배치)', () => {
  it('게임패드 있으면 gamepad', () => {
    expect(pickInputMode(true)).toBe('gamepad');
  });
  it('게임패드 없으면 PC 든 태블릿이든 touch — 가상 조이스틱은 늘 보인다', () => {
    expect(pickInputMode(false)).toBe('touch');
  });
});

describe('keyboardDrive — PC 에서만 키보드로 조종', () => {
  it('PC + 패드 없음 → 켬', () => { expect(keyboardDrive(false, true)).toBe(true); });
  it('패드가 있으면 끔(패드가 조종한다)', () => { expect(keyboardDrive(true, true)).toBe(false); });
  it('PC 아님(폰·태블릿·덱) → 끔', () => { expect(keyboardDrive(false, false)).toBe(false); });
});

describe('resolveHasPhysicalGamepad — 덱 게이밍 모드 단정', () => {
  it('auto + 열거 0 + 덱 게이밍 → 패드 있음', () => {
    expect(resolveHasPhysicalGamepad(0, 'auto', true)).toBe(true);
  });
  it('auto + 열거 0 + 그 외(덱 데스크톱 모드 포함) → 패드 없음', () => {
    expect(resolveHasPhysicalGamepad(0, 'auto', false)).toBe(false);
  });
  it("'virtual' 강제는 덱 게이밍이어도 이긴다(가상 조이스틱 검증용)", () => {
    expect(resolveHasPhysicalGamepad(0, 'virtual', true)).toBe(false);
  });
  it("'gamepad' 강제는 열거와 무관하게 패드", () => {
    expect(resolveHasPhysicalGamepad(0, 'gamepad', false)).toBe(true);
  });
  it('실제 패드가 잡히면 단정 여부와 무관하게 패드', () => {
    expect(resolveHasPhysicalGamepad(1, 'auto', false)).toBe(true);
  });
});
