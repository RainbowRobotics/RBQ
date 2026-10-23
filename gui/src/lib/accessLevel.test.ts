import { describe, it, expect } from 'vitest';
import { resolveAccessLevel, sha256hex } from './accessLevel';

describe('접근 레벨 기본 암호', () => {
  it('env 미설정 빌드는 0000 이 최고 레벨(3)을 연다 — 2·3단계 기본값이 같다', () => {
    if (process.env.EXPO_PUBLIC_LV2_SHA256 || process.env.EXPO_PUBLIC_LV3_SHA256) return;
    expect(resolveAccessLevel('0000')).toBe(3);
  });
  it('옛 기본 암호(1234·5678)는 기본값으로 열리지 않는다', () => {
    if (process.env.EXPO_PUBLIC_LV2_SHA256 || process.env.EXPO_PUBLIC_LV3_SHA256) return;
    expect(resolveAccessLevel(String(1234))).toBeNull();
    expect(resolveAccessLevel(String(5678))).toBeNull();
  });
  it('sha256hex 는 표준 SHA-256', () => {
    expect(sha256hex('abc')).toBe('ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad');
  });
});
