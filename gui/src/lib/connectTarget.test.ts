import { describe, it, expect } from 'vitest';
import { directTarget, rendezvousTarget, sameTarget, settingsFor, isPrivateIp } from './connectTarget';

const ok = <T,>(r: { ok: true; target: T } | { ok: false; reason: string }) => {
  if (!r.ok) throw new Error(`실패: ${r.reason}`);
  return r.target;
};

describe('대상 만들기', () => {
  it('직결은 주소가 있어야 한다', () => {
    expect(directTarget('  ')).toEqual({ ok: false, reason: 'no_addr' });
  });

  it('랑데부는 robotId 와 url 둘 다 있어야 한다', () => {
    expect(rendezvousTarget('', 'ws://x/ws')).toEqual({ ok: false, reason: 'no_robot_id' });
    expect(rendezvousTarget('R1', '')).toEqual({ ok: false, reason: 'no_rendezvous_url' });
  });

  it('★랑데부의 주소는 robotId 다 — 계정이 준 lanIp 를 쓰면 다른 로봇이 같은 대상으로 보인다', () => {
    const t = ok(rendezvousTarget('RBQ105', 'ws://x/ws'));
    expect(t.addr).toBe('RBQ105');
  });
});

describe('sameTarget', () => {
  it('로봇이 다르면 다른 대상이다', () => {
    const a = ok(rendezvousTarget('RBQ103', 'ws://x/ws'));
    const b = ok(rendezvousTarget('RBQ105', 'ws://x/ws'));
    expect(sameTarget(a, b)).toBe(false);
  });

  it('경로가 다르면 주소가 같아도 다른 대상이다', () => {
    const a = ok(directTarget('192.168.0.10'));
    const b = ok(rendezvousTarget('192.168.0.10', 'ws://x/ws'));
    expect(sameTarget(a, b)).toBe(false);
  });

  it('전부 같으면 같은 대상', () => {
    expect(sameTarget(ok(directTarget('192.168.0.10')), ok(directTarget(' 192.168.0.10 ')))).toBe(true);
  });

  it('한쪽이 없으면 같지 않다 — 처음 연결은 항상 새로 붙는다', () => {
    expect(sameTarget(null, ok(directTarget('1.2.3.4')))).toBe(false);
  });
});

describe('settingsFor — 안 쓰는 값은 지운다', () => {
  it('랑데부면 wanIp·토큰을 비운다', () => {
    const s = settingsFor(ok(rendezvousTarget('RBQ103', 'ws://x/ws')));
    expect(s).toMatchObject({ connProfile: 'wan', wanIp: '', webrtcToken: '',
                              rendezvousUrl: 'ws://x/ws', robotId: 'RBQ103' });
  });

  it('★직결이면 랑데부 값을 비운다 — 남겨 두면 다음 연결이 그걸 주워 쓴다', () => {
    const s = settingsFor(ok(directTarget('192.168.0.10')));
    expect(s).toMatchObject({ connProfile: 'lan', lanIp: '192.168.0.10',
                              rendezvousUrl: '', robotId: '', wanIp: '' });
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
