import { describe, it, expect } from 'vitest';
import { validateDraft, registerValues, type RegisterDraft } from './robotRegister';

const draft = (o: Partial<RegisterDraft> = {}): RegisterDraft => ({
  serial: 'RBQ1000000001',
  name: 'RBQ10 사무실 1호기',
  lanIp: '192.168.0.10',
  rendezvousUrl: 'ws://203.0.113.10:8888/ws',
  ...o,
});

describe('validateDraft', () => {
  it('정상값은 통과', () => {
    expect(validateDraft(draft())).toEqual({ ok: true });
  });

  it('시리얼이 비면 막는다 — 서버에 빈 행이 남는다', () => {
    expect(validateDraft(draft({ serial: '   ' }))).toEqual({ ok: false, reason: 'serial_empty' });
  });

  it('시리얼에 공백·경로문자를 막는다 — 로그 버킷 경로와 같은 문자열이다', () => {
    expect(validateDraft(draft({ serial: 'RBQ 102' }))).toEqual({ ok: false, reason: 'serial_charset' });
    expect(validateDraft(draft({ serial: 'a/b' }))).toEqual({ ok: false, reason: 'serial_charset' });
  });

  it('랑데부 url 은 비워도 된다 — LAN 전용 로봇도 등록한다', () => {
    expect(validateDraft(draft({ rendezvousUrl: '' }))).toEqual({ ok: true });
  });

  it('랑데부 url 이 ws/wss 가 아니면 막는다', () => {
    expect(validateDraft(draft({ rendezvousUrl: 'http://h/ws' }))).toEqual({ ok: false, reason: 'rv_scheme' });
  });
});

describe('registerValues', () => {
  it('랑데부 id 는 시리얼이다 — hostname 은 RBQ10 이 다들 r-pc 라 충돌한다', () => {
    expect(registerValues(draft()).robotId).toBe('RBQ1000000001');
  });

  it('로봇 INI 와 서버에 같은 값이 간다 — 어긋나면 폴백이 못 붙는다', () => {
    const v = registerValues(draft({ serial: '  RBQ1000000001  ' }));
    expect(v.serial).toBe('RBQ1000000001');
    expect(v.robotId).toBe(v.serial);
  });
});

