import { describe, expect, it } from 'vitest';
import { askStandGate, standBlockOf, standBlockText, useStandGate } from './standGate';

const err = (status: number, detail = '') => Object.assign(new Error('x'), { status, detail });
const body = (b: object) => err(409, JSON.stringify(b));

describe('standGate', () => {
  it('기립 거절 409 만 알아본다', () => {
    expect(standBlockOf(body({ error: 'equipment_without_payload', ids: ['0x230', '0x231'] })))
      .toEqual({ reason: 'equipment_without_payload', ids: ['0x230', '0x231'], option: undefined });
    expect(standBlockOf(body({ error: 'firmware update in progress' }))).toBeNull();
    expect(standBlockOf(err(403, JSON.stringify({ error: 'option' })))).toBeNull();
    expect(standBlockOf(err(409, 'not json'))).toBeNull();
    expect(standBlockOf(new Error('x'))).toBeNull();
  });
  it('문구에 장치 번호·옵션 이유가 들어간다', () => {
    expect(standBlockText({ reason: 'equipment_without_payload', ids: ['0x230'] })).toContain('0x230');
    expect(standBlockText({ reason: 'option', ids: [], option: { ko: '장비 무게 미반영', en: 'Gear weight not applied' } })).toContain('장비 무게 미반영');
  });
  it('거절한 로봇 주소를 같이 둔다(다른 로봇으로 바꾸면 창이 닫히게)', () => {
    expect(askStandGate(body({ error: 'option', ids: [] }), () => {}, '192.168.0.10')).toBe(true);
    expect(useStandGate.getState().pending?.ip).toBe('192.168.0.10');
    useStandGate.getState().close();
  });
  it('거절이면 확인 창을 올리고, [그래도 세우기]가 force 를 부른다', () => {
    let forced = 0;
    expect(askStandGate(err(500), () => forced++)).toBe(false);
    expect(useStandGate.getState().pending).toBeNull();
    expect(askStandGate(body({ error: 'option', ids: [] }), () => forced++)).toBe(true);
    useStandGate.getState().pending?.force();
    expect(forced).toBe(1);
    useStandGate.getState().close();
    expect(useStandGate.getState().pending).toBeNull();
  });
});
