import { describe, it, expect } from 'vitest';
import { resolveAccessLevel, sha256hex } from './accessLevel';
import { accessCodes } from '@/modules/registry';

describe('접근 레벨 암호', () => {
  it('기본 암호가 없다 — 코드를 넣지 않은 빌드는 0000 으로도 열리지 않는다', () => {
    if (process.env.EXPO_PUBLIC_LV2_SHA256) return;
    expect(resolveAccessLevel('0000')).toBeNull();
  });
  it('모듈이 더한 코드는 그 레벨을 연다(앞뒤 공백 무시)', () => {
    accessCodes.push({ level: 3, hash: sha256hex('t-module') });
    try {
      expect(resolveAccessLevel(' t-module ')).toBe(3);
      expect(resolveAccessLevel('t-other')).toBeNull();
    } finally { accessCodes.length = 0; }
  });
  it('sha256hex 는 표준 SHA-256', () => {
    expect(sha256hex('abc')).toBe('ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
  });
});
