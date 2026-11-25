import { describe, expect, it } from 'vitest';
import { LAMP_TEST_MS, lampColorAt, lampErrorText } from './topLamp';

describe('lampColorAt — TOP panel.c 와 같은 3등분', () => {
  it('빨강 → 초록 → 파랑, 1.6초에 끝', () => {
    expect(lampColorAt(0)).toEqual([255, 0, 0]);
    expect(lampColorAt(533)).toEqual([255, 0, 0]);
    expect(lampColorAt(534)).toEqual([0, 255, 0]);
    expect(lampColorAt(1066)).toEqual([0, 255, 0]);
    expect(lampColorAt(1067)).toEqual([0, 0, 255]);
    expect(lampColorAt(LAMP_TEST_MS - 1)).toEqual([0, 0, 255]);
    expect(lampColorAt(LAMP_TEST_MS)).toBeNull();
  });
  it('시작 전 · 숫자가 아니면 꺼짐', () => {
    expect(lampColorAt(-1)).toBeNull();
    expect(lampColorAt(Number.NaN)).toBeNull();
  });
});

describe('lampErrorText', () => {
  it('앱이 아는 상태만 풀어 쓴다', () => {
    expect(lampErrorText(404)).toContain('업데이트');
    expect(lampErrorText(409)).toContain('CAN FD');
    expect(lampErrorText(0)).toContain('연결');
    expect(lampErrorText(500)).toBeNull();
  });
});
