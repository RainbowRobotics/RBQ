import { describe, it, expect } from 'vitest';
import { gaitTone } from './gaitTone';

describe('gaitTone — 로봇 enum 구간을 따른다', () => {
  it('음수는 이상 상태 — 넘어짐이 파란색으로 보이던 것이 이 버그였다', () => {
    expect(gaitTone(-3, 'FALL_RECOVERY')).toBe('fault');
    expect(gaitTone(-2, 'FALL_MODE')).toBe('fault');
    expect(gaitTone(-1, 'CONTROL_OFF')).toBe('fault');
  });
  it('0~1 은 정지·대기', () => {
    expect(gaitTone(0, 'SITTING')).toBe('idle');
    expect(gaitTone(1, 'STANDING')).toBe('idle');
  });
  it('2~10 은 동작 중 — 종전에 TROTTING 만 받던 색이 이 구간으로 넓어진다', () => {
    expect(gaitTone(3, 'TROTTING')).toBe('active');
    expect(gaitTone(10, 'DOCKING')).toBe('active');
    expect(gaitTone(8, 'ZMP_INITIALIZING')).toBe('active');
  });
  it('30~49 RL_* 도 동작 중', () => {
    expect(gaitTone(30, 'RL_TROT')).toBe('active');
    expect(gaitTone(47, 'RL_TROT_VISION_SLOW')).toBe('active');
    expect(gaitTone(49, 'RL_WALK')).toBe('active');
  });
  it('구간 밖 번호는 단정하지 않는다', () => {
    expect(gaitTone(20, 'WHATEVER')).toBe('unknown');
    expect(gaitTone(80, 'RL_END')).toBe('unknown');
  });

  it('gait_id 가 없는 구형 데몬은 이름으로 떨어뜨린다', () => {
    expect(gaitTone(undefined, 'FALL_MODE')).toBe('fault');
    expect(gaitTone(null, 'STANDING')).toBe('idle');
    expect(gaitTone(undefined, 'RL_TROT_VISION_SLOW')).toBe('active');
    expect(gaitTone(undefined, 'UNKNOWN')).toBe('unknown');
    expect(gaitTone(undefined, 'SOME_NEW_FAULT')).toBe('unknown');
    expect(gaitTone(undefined, undefined)).toBe('unknown');
  });
});
