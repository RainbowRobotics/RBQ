import { describe, it, expect } from 'vitest';
import { applyRendezvousSection, rendezvousPatchFromInputs, type IniSection } from './robotRendezvousConfig';

const base: IniSection[] = [
  { name: 'BASIC', keys: [{ key: 'NO_OF_IMU', value: '1' }, { key: 'NO_OF_MC', value: '12' }] },
  { name: 'CAN_TYPE', keys: [{ key: 'CAN_TYPE_SocketCAN', value: 'true' }] },
];

const sec = (secs: IniSection[], name: string) => secs.find((s) => s.name === name);
const val = (secs: IniSection[], name: string, key: string) =>
  sec(secs, name)?.keys.find((k) => k.key === key)?.value;

describe('applyRendezvousSection', () => {
  it('기존 섹션을 하나도 잃지 않는다 — PUT 은 전체 교체다', () => {
    const out = applyRendezvousSection(base, { enabled: true, url: 'ws://h/ws' });
    expect(out.map((s) => s.name)).toContain('BASIC');
    expect(out.map((s) => s.name)).toContain('CAN_TYPE');
    expect(val(out, 'BASIC', 'NO_OF_MC')).toBe('12');
  });

  it('RENDEZVOUS 섹션이 없으면 새로 만든다', () => {
    const out = applyRendezvousSection(base, { enabled: true, url: 'ws://h/ws' });
    expect(val(out, 'RENDEZVOUS', 'enabled')).toBe('true');
    expect(val(out, 'RENDEZVOUS', 'url')).toBe('ws://h/ws');
  });

  it('있으면 그 안의 키만 바꾼다 — 같은 섹션의 다른 키는 남는다', () => {
    const withRv: IniSection[] = [
      ...base,
      { name: 'RENDEZVOUS', keys: [
        { key: 'enabled', value: 'false' },
        { key: 'url', value: 'ws://old/ws' },
        { key: 'webrtc_token', value: 'KEEP' },
      ] },
    ];
    const out = applyRendezvousSection(withRv, { enabled: true, url: 'ws://new/ws' });
    expect(val(out, 'RENDEZVOUS', 'enabled')).toBe('true');
    expect(val(out, 'RENDEZVOUS', 'url')).toBe('ws://new/ws');
    expect(val(out, 'RENDEZVOUS', 'webrtc_token')).toBe('KEEP');
  });

  it('undefined 인 필드는 쓰지 않는다 — 토큰이 조용히 지워지면 안 된다', () => {
    const withTok: IniSection[] = [
      ...base,
      { name: 'RENDEZVOUS', keys: [{ key: 'webrtc_token', value: 'SECRET' }] },
    ];
    const out = applyRendezvousSection(withTok, { enabled: false });
    expect(val(out, 'RENDEZVOUS', 'webrtc_token')).toBe('SECRET');
  });

  it('빈 문자열은 명시적 지우기다 — undefined 와 구분한다', () => {
    const withTok: IniSection[] = [
      ...base,
      { name: 'RENDEZVOUS', keys: [{ key: 'webrtc_token', value: 'SECRET' }] },
    ];
    const out = applyRendezvousSection(withTok, { enabled: false, webrtcToken: '' });
    expect(val(out, 'RENDEZVOUS', 'webrtc_token')).toBe('');
  });

  it('원본을 변형하지 않는다', () => {
    const snapshot = JSON.stringify(base);
    applyRendezvousSection(base, { enabled: true, url: 'ws://h/ws' });
    expect(JSON.stringify(base)).toBe(snapshot);
  });

  it('enabled=false 로 끄면 url·token 은 그대로 둔다 — 다시 켤 때 재입력이 필요 없다', () => {
    const withRv: IniSection[] = [
      ...base,
      { name: 'RENDEZVOUS', keys: [
        { key: 'enabled', value: 'true' },
        { key: 'url', value: 'ws://h/ws' },
      ] },
    ];
    const out = applyRendezvousSection(withRv, { enabled: false });
    expect(val(out, 'RENDEZVOUS', 'enabled')).toBe('false');
    expect(val(out, 'RENDEZVOUS', 'url')).toBe('ws://h/ws');
  });
});

describe('rendezvousPatchFromInputs', () => {
  it('빈 칸은 undefined 다 — 빈 문자열로 넘기면 로봇 값이 지워진다', () => {
    const p = rendezvousPatchFromInputs(true, '');
    expect(p).toEqual({ enabled: true });
    expect('url' in p).toBe(false);
  });

  it('토큰은 둘 다 만들지 않는다 — 토글의 일은 랑데부를 켜는 것뿐이다', () => {
    const p = rendezvousPatchFromInputs(true, 'ws://h/ws');
    expect(p).toEqual({ enabled: true, url: 'ws://h/ws' });
    expect('token' in p).toBe(false);
    expect('webrtcToken' in p).toBe(false);
  });

  it('끌 때는 enabled 만 — url·token 을 남긴다', () => {
    expect(rendezvousPatchFromInputs(false, 'ws://h/ws')).toEqual({ enabled: false });
  });

  it('켤 때 빈 url 이 기존 값을 지우면 안 된다', () => {
    const withRv: IniSection[] = [
      ...base,
      { name: 'RENDEZVOUS', keys: [
        { key: 'url', value: 'ws://203.0.113.20:8888/ws' },
        { key: 'token', value: 'REGTOKEN' },
      ] },
    ];
    const out = applyRendezvousSection(withRv, rendezvousPatchFromInputs(true, ''));
    expect(val(out, 'RENDEZVOUS', 'url')).toBe('ws://203.0.113.20:8888/ws');
    expect(val(out, 'RENDEZVOUS', 'token')).toBe('REGTOKEN');
    expect(val(out, 'RENDEZVOUS', 'enabled')).toBe('true');
  });

  it('자기잠금 방지: 토글이 로봇에 webrtc_token 을 심지 않는다', () => {
    const clean: IniSection[] = [...base, { name: 'RENDEZVOUS', keys: [{ key: 'enabled', value: 'false' }] }];
    const out = applyRendezvousSection(clean, rendezvousPatchFromInputs(true, 'ws://h/ws'));
    expect(sec(out, 'RENDEZVOUS')?.keys.some((k) => k.key === 'webrtc_token')).toBe(false);
  });
});
