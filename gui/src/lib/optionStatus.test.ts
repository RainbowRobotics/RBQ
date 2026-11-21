import { describe, it, expect, vi } from 'vitest';

vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
vi.mock('@/store/lang', () => ({ getLang: () => 'ko' }));
vi.mock('@/store/robot', () => ({ useRobot: () => '' }));
vi.mock('@/store/capability', () => ({ useFeatures: () => '' }));
vi.mock('./rest', () => ({ rest: {} }));

import { optionStatusRows, isFresh, itemValue, itemBool, type OptionStatusResp } from './optionStatus';

const resp: OptionStatusResp = {
  items: [
    { key: 'radiation', label: { ko: '선량', en: 'Radiation' }, value: '3.5000e-03', unit: 'mSv', level: 'ok' },
    { key: 'kcmvp', label: { ko: 'KCMVP', en: 'KCMVP' }, value: false, unit: '', level: 'error' },
    { key: 'time_s', label: { ko: '시간', en: 'Time' }, value: 120, unit: 's', level: 'ok' },
    { key: 'pistol', label: { ko: '권총', en: 'Pistol' }, value: null, unit: '', level: 'ok' },
  ],
  age_ms: 800,
  fresh_ms: 5000,
};

describe('옵션 상태 카드', () => {
  it('행은 언어에 맞는 라벨 · 값 + 단위', () => {
    expect(optionStatusRows(resp, 'ko')).toEqual([
      ['선량', '3.5000e-03 mSv'],
      ['KCMVP', '아니오'],
      ['시간', '120 s'],
      ['권총', '—'],
    ]);
    expect(optionStatusRows(resp, 'en')[0]).toEqual(['Radiation', '3.5000e-03 mSv']);
  });
  it('받은 지 5 s 넘거나 받은 적 없으면 신선하지 않다', () => {
    expect(isFresh(resp)).toBe(true);
    expect(isFresh({ ...resp, age_ms: 5001 })).toBe(false);
    expect(isFresh({ items: [], age_ms: null, fresh_ms: 5000 })).toBe(false);
    expect(isFresh(undefined)).toBe(false);
  });
  it('모양이 틀린 항목은 버린다', () => {
    const bad = { items: [{ key: 'a' }, null, 3, { key: 'b', label: { ko: 'B', en: 'B' }, value: 1, unit: '', level: 'ok' }], age_ms: 1, fresh_ms: 5000 };
    expect(optionStatusRows(bad as never, 'ko')).toEqual([['B', '1']]);
  });
  it('모듈이 값 하나를 꺼내 본다 — 신선할 때만', () => {
    expect(itemValue(resp, 'time_s')).toBe(120);
    expect(itemBool(resp, 'kcmvp')).toBe(false);
    expect(itemBool(resp, 'nope')).toBeUndefined();
    expect(itemBool({ ...resp, age_ms: 9000 }, 'kcmvp')).toBeUndefined();
  });
});
