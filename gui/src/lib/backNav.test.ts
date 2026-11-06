import { describe, expect, it } from 'vitest';
import { backAction } from './backNav';
import { robotKinds, type RobotKind } from '@/modules/registry';

robotKinds.push({ key: 'kind-x', route: '/kind-x' } as RobotKind);

describe('backAction', () => {
  it('돌아갈 화면이 있으면 pop', () => {
    expect(backAction('hw', '/settings', true, true)).toBe('pop');
    expect(backAction('esc', '/log', true, true)).toBe('pop');
  });
  it('최상위 컨트롤은 허브로 — 앱을 닫지 않는다', () => {
    expect(backAction('hw', '/', false, true)).toBe('hub');
    expect(backAction('esc', '/kind-x', false, true)).toBe('hub');
  });
  it('허브·PIN 은 종료 확인, 종료할 수 없는 브라우저는 아무것도 안 함', () => {
    expect(backAction('hw', '/hub', false, true)).toBe('quit');
    expect(backAction('pad', '/pin', false, true)).toBe('quit');
    expect(backAction('esc', '/hub', false, false)).toBe(null);
  });
  it('패드 B 는 조종·팔·진단 화면에서 안 먹는다', () => {
    for (const p of ['/', '/kind-x', '/gamepad', '/arm-manual']) expect(backAction('pad', p, true, true)).toBe(null);
    expect(backAction('pad', '/settings', true, true)).toBe('pop');
  });
  it('단독 페이지(웹으로 바로 연 모듈 화면)는 돌아갈 곳이 없으면 아무것도 안 함', () => {
    expect(backAction('esc', '/x', false, true, true)).toBe(null);
    expect(backAction('esc', '/x', true, true, true)).toBe('pop');
  });
  it('Esc 는 팔 화면에서 안 먹는다(조그 멈춤), 하드웨어 뒤로는 먹는다(접기 경로)', () => {
    expect(backAction('esc', '/arm-manual', false, true)).toBe(null);
    expect(backAction('hw', '/arm-manual', false, true)).toBe('hub');
  });
});
