import { describe, it, expect } from 'vitest';
import { directTarget, sameTarget, settingsFor, isPrivateIp, type Target } from './connectTarget';

const ok = <T,>(r: { ok: true; target: T } | { ok: false; reason: string }) => {
  if (!r.ok) throw new Error(`실패: ${r.reason}`);
  return r.target;
};

const remoteT = (id: string): Target => ({ route: 'remote', addr: id, remote: { id }, webrtcToken: '', label: id });

describe('대상 만들기', () => {
  it('직결은 주소가 있어야 한다', () => {
    expect(directTarget('  ')).toEqual({ ok: false, reason: 'no_addr' });
  });

});

describe('sameTarget', () => {
  it('원격 값이 다르면 다른 대상이다(원격 모듈이 채우는 remote)', () => {
    expect(sameTarget(remoteT('RBQ103'), remoteT('RBQ105'))).toBe(false);
    expect(sameTarget(remoteT('RBQ103'), remoteT('RBQ103'))).toBe(true);
  });

  it('경로가 다르면 주소가 같아도 다른 대상이다', () => {
    expect(sameTarget(ok(directTarget('192.168.0.10')), { ...remoteT('x'), addr: '192.168.0.10' })).toBe(false);
  });

  it('전부 같으면 같은 대상', () => {
    expect(sameTarget(ok(directTarget('192.168.0.10')), ok(directTarget(' 192.168.0.10 ')))).toBe(true);
  });

  it('한쪽이 없으면 같지 않다 — 처음 연결은 항상 새로 붙는다', () => {
    expect(sameTarget(null, ok(directTarget('1.2.3.4')))).toBe(false);
  });
});

describe('settingsFor — 안 쓰는 값은 지운다', () => {
  it('원격이면 wan 프로파일 — wanIp·토큰을 비운다(원격 값은 모듈이 쓴다)', () => {
    expect(settingsFor(remoteT('RBQ103'))).toEqual({ connProfile: 'wan', wanIp: '', webrtcToken: '' });
  });

  it('★로봇망 직결은 wanIp 를 비운다 — 남겨 두면 다음 연결이 그걸 주워 쓴다', () => {
    const s = settingsFor(ok(directTarget('192.168.0.10')));
    expect(s).toMatchObject({ connProfile: 'lan', lanIp: '192.168.0.10', wanIp: '' });
  });

  it('공인 주소 직결은 wan 으로 저장한다', () => {
    const s = settingsFor(ok(directTarget('203.0.113.5', { token: 'tok' })));
    expect(s).toMatchObject({ connProfile: 'wan', wanIp: '203.0.113.5', webrtcToken: 'tok' });
  });

  it('루프백은 lo 프로파일 — lanIp·wanIp 를 건드리지 않는다(Lo 탭 저장이 로봇 LAN IP 를 덮던 회귀)', () => {
    const s = settingsFor(ok(directTarget('127.0.0.1')));
    expect(s.connProfile).toBe('lo');
    expect(s.lanIp).toBeUndefined();
    expect(s.wanIp).toBeUndefined();
  });
});

describe('isPrivateIp', () => {
  it.each([['192.168.0.10', true], ['10.1.2.3', true], ['172.16.0.1', true], ['127.0.0.1', true],
           ['172.32.0.1', false], ['203.0.113.5', false], ['robot.example.com', false]])(
    '%s → %s', (addr, want) => expect(isPrivateIp(addr as string)).toBe(want));
});
