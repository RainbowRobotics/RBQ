import { describe, it, expect } from 'vitest';
import { restoreAddr } from './restoreAddr';

describe('restoreAddr', () => {
  it('⚠LAN: lanIp 가 비면 로봇 IP 로 떨어진다 — 저장 없이도 붙어야 한다', () => {
    expect(restoreAddr('lan', '', '', '192.168.0.10')).toBe('192.168.0.10');
    expect(restoreAddr('lan', undefined, undefined, '192.168.0.10')).toBe('192.168.0.10');
    expect(restoreAddr('lan', '   ', '', '192.168.0.10')).toBe('192.168.0.10');
  });

  it('LAN: lanIp 가 있으면 그것이 이긴다 — 사용자가 저장한 값이 우선', () => {
    expect(restoreAddr('lan', '10.0.0.5', '', '192.168.0.10')).toBe('10.0.0.5');
  });

  it('Lo 는 언제나 루프백', () => {
    expect(restoreAddr('lo', '10.0.0.5', '1.2.3.4', '192.168.0.10')).toBe('127.0.0.1');
  });

  it('WAN 은 wanIp 만 본다 — 로봇 IP 로 떨어지지 않는다(다른 망에서 붙는 프로파일)', () => {
    expect(restoreAddr('wan', '10.0.0.5', '1.2.3.4', '192.168.0.10')).toBe('1.2.3.4');
    expect(restoreAddr('wan', '10.0.0.5', '', '192.168.0.10')).toBe('');
  });

  it('둘 다 없으면 빈 문자열 — 붙을 대상이 없다', () => {
    expect(restoreAddr('lan', '', '', '')).toBe('');
  });
});
