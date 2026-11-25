import { describe, it, expect } from 'vitest';
import { serialKey, libName, parseLibName, toEntries, libSerials, libDates, libSessions, NO_SERIAL } from './bbLibraryCommon';

describe('이름 규칙', () => {
  it('S/N 이 있으면 파일 이름에 들어가고, 되읽으면 날짜·세션이 나온다', () => {
    const n = libName('RBQ10-001', '20260911', '08_32_18');
    expect(n).toBe('blackbox-RBQ10-001-20260911_08_32_18.zip');
    expect(parseLibName(n)).toEqual({ date: '20260911', session: '08_32_18' });
  });

  it('S/N 을 모르면(시뮬·옛 zip) 전용 폴더 — 파일 이름은 옛 zip 과 같다', () => {
    expect(serialKey('')).toBe(NO_SERIAL);
    expect(serialKey('  ')).toBe(NO_SERIAL);
    expect(libName('', '20260921', '17_52_16')).toBe('blackbox-20260921_17_52_16.zip');
    expect(parseLibName('blackbox-20260921_17_52_16.zip')).toEqual({ date: '20260921', session: '17_52_16' });
  });

  it('경로 문자는 폴더 이름이 되지 못한다', () => {
    expect(serialKey('../etc')).toBe('.._etc');
    expect(serialKey('..')).toBe(NO_SERIAL);
    expect(serialKey('RBQ 10/23')).toBe('RBQ_10_23');
  });

  it('보관함 규칙에 안 맞는 파일은 목록에서 빠진다', () => {
    expect(parseLibName('notes.txt')).toBeNull();
    expect(parseLibName('blackbox-RBQ10-001.zip')).toBeNull();
  });
});

describe('S/N 별 묶기', () => {
  const entries = toEntries([
    { serial: 'RBQ10-002', name: 'blackbox-RBQ10-002-20260910_09_00_00.zip', size: 1 },
    { serial: 'RBQ10-001', name: 'blackbox-RBQ10-001-20260911_08_32_18.zip', size: 2 },
    { serial: 'RBQ10-001', name: 'blackbox-RBQ10-001-20260911_10_05_40.zip', size: 3 },
    { serial: 'RBQ10-001', name: 'blackbox-RBQ10-001-20260909_14_00_00.zip', size: 4 },
    { serial: NO_SERIAL, name: 'blackbox-20260921_17_52_16.zip', size: 5 },
    { serial: 'RBQ10-001', name: 'readme.txt', size: 6 },
  ]);

  it('로봇은 이름순, S/N 없음은 맨 뒤', () => {
    expect(libSerials(entries)).toEqual(['RBQ10-001', 'RBQ10-002', NO_SERIAL]);
  });

  it('날짜·세션은 최신 먼저, 다른 로봇 것이 섞이지 않는다', () => {
    expect(libDates(entries, 'RBQ10-001')).toEqual(['20260911', '20260909']);
    expect(libSessions(entries, 'RBQ10-001', '20260911')).toEqual(['10_05_40', '08_32_18']);
    expect(libDates(entries, 'RBQ10-002')).toEqual(['20260910']);
  });
});
