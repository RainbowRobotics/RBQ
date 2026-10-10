import { describe, it, expect } from 'vitest';
import { loginWithPin, pickAccount, isCacheValid, isExpired, rowToAccount, CACHE_TTL_MS, type RemoteAccount, type RemoteLoginCache } from './remoteLogin';

const D = { url: 'https://x.supabase.co', anonKey: 'anon' };
const ROW = { account_id: 'a1', account_name: 'demo-user', level: 3, robot_serial: 'RBQ1000000001', robot_name: 'R1', lan_ip: '192.168.0.10' };
const okFetch = (rows: unknown[]) => (async () => ({ ok: true, json: async () => rows })) as unknown as typeof fetch;
const deadFetch = (async () => { throw new Error('network'); }) as unknown as typeof fetch;
const cache = (at: number): RemoteLoginCache => ({ account: rowToAccount(ROW), at });

describe('원격 로그인 PIN (텔레오프 v1)', () => {
  it('정답 PIN → 계정·레벨·매칭 로봇', async () => {
    const r = await loginWithPin('2468', { ...D, fetchFn: okFetch([ROW]) });
    expect(r).toMatchObject({ ok: true, fromCache: false });
    if (r.ok) expect(r.account).toMatchObject({ accountName: 'demo-user', level: 3, lanIp: '192.168.0.10' });
  });

  it('매칭이 여러 대면 전부 준다 — 첫 행만 쓰면 나머지가 없는 것처럼 보인다', async () => {
    const ROW2 = { ...ROW, robot_serial: 'RBQ1000000002', robot_name: 'R2', lan_ip: '192.168.0.11' };
    const r = await loginWithPin('2468', { ...D, fetchFn: okFetch([ROW, ROW2]) });
    expect(r.ok && r.accounts.map((a) => a.robotSerial)).toEqual(['RBQ1000000001', 'RBQ1000000002']);
    expect(r.ok && r.account.robotSerial).toBe('RBQ1000000001');
  });

  it('서버 시도 제한(429) → throttled — 오답(bad_pin)과 구분해야 사용자가 기다린다', async () => {
    const throttled = (async () => new Response('{}', { status: 429 })) as unknown as typeof fetch;
    expect(await loginWithPin('1357', { ...D, fetchFn: throttled })).toEqual({ ok: false, reason: 'throttled' });
  });
  it('오답 PIN(빈 배열) → bad_pin', async () => {
    const r = await loginWithPin('0000', { ...D, fetchFn: okFetch([]) });
    expect(r).toEqual({ ok: false, reason: 'bad_pin' });
  });

  it('형식 위반(4자리 아님)은 요청조차 보내지 않는다', async () => {
    let called = false;
    const spy = (async () => { called = true; return { ok: true, json: async () => [ROW] }; }) as unknown as typeof fetch;
    expect(await loginWithPin('79', { ...D, fetchFn: spy })).toEqual({ ok: false, reason: 'bad_pin' });
    expect(called).toBe(false);
  });

  it('오프라인 + 유효 캐시 → 통과(fromCache)', async () => {
    const now = 1_000_000_000;
    const r = await loginWithPin('2468', { ...D, fetchFn: deadFetch, now: () => now, cache: cache(now - 1000) });
    expect(r).toMatchObject({ ok: true, fromCache: true });
  });

  it('오프라인 + 만료 캐시 → offline', async () => {
    const now = 1_000_000_000;
    const r = await loginWithPin('2468', { ...D, fetchFn: deadFetch, now: () => now, cache: cache(now - CACHE_TTL_MS - 1) });
    expect(r).toEqual({ ok: false, reason: 'offline' });
  });

  it('서버가 오답이라고 답하면 캐시가 있어도 통과시키지 않는다 (폐기 PIN 방어)', async () => {
    const now = 1_000_000_000;
    const r = await loginWithPin('2468', { ...D, fetchFn: okFetch([]), now: () => now, cache: cache(now) });
    expect(r).toEqual({ ok: false, reason: 'bad_pin' });
  });

  it('HTTP 오류 → server', async () => {
    const bad = (async () => ({ ok: false, json: async () => ({}) })) as unknown as typeof fetch;
    expect(await loginWithPin('2468', { ...D, fetchFn: bad })).toEqual({ ok: false, reason: 'server' });
  });
});

describe('캐시·레벨 순수 함수', () => {
  it('isCacheValid — null/미래시각/만료 경계', () => {
    const now = 1_000_000;
    expect(isCacheValid(null, now)).toBe(false);
    expect(isCacheValid(cache(now + 5000), now)).toBe(false);
    expect(isCacheValid(cache(now - CACHE_TTL_MS), now)).toBe(false);
    expect(isCacheValid(cache(now - CACHE_TTL_MS + 1), now)).toBe(true);
  });


  it('rowToAccount — 이상한 level은 1로 접지, 매칭 없는 계정은 null', () => {
    expect(rowToAccount({ ...ROW, level: 9 }).level).toBe(1);
    const solo = rowToAccount({ account_id: 'a2', account_name: 'x', level: 1 });
    expect(solo.robotSerial).toBeNull();
    expect(solo.lanIp).toBeNull();
  });
});

describe('rowToAccount — v2 연결정보', () => {
  it('서버가 주는 5종을 매핑한다', () => {
    const a = rowToAccount({
      ...ROW,
      wan_ip: '1.2.3.4', rendezvous_url: 'ws://h/ws', robot_id: 'r-pc',
      webrtc_token: 'tok', expires_at: '2026-09-01T00:00:00Z',
    });
    expect(a.wanIp).toBe('1.2.3.4');
    expect(a.rendezvousUrl).toBe('ws://h/ws');
    expect(a.robotId).toBe('r-pc');
    expect(a.webrtcToken).toBe('tok');
    expect(a.expiresAt).toBe('2026-09-01T00:00:00Z');
  });

  it('v2 필드가 없으면 null — 구 서버 호환', () => {
    const a = rowToAccount(ROW);
    expect(a.wanIp).toBeNull();
    expect(a.rendezvousUrl).toBeNull();
    expect(a.robotId).toBeNull();
    expect(a.webrtcToken).toBeNull();
    expect(a.expiresAt).toBeNull();
  });

  it('빈 문자열도 null 로 접는다 — 미설정이지 유효한 값이 아니다', () => {
    const a = rowToAccount({ ...ROW, wan_ip: '', rendezvous_url: '', robot_id: '', webrtc_token: '' });
    expect(a.wanIp).toBeNull();
    expect(a.rendezvousUrl).toBeNull();
    expect(a.robotId).toBeNull();
    expect(a.webrtcToken).toBeNull();
  });
});

describe('isExpired', () => {
  const at = (iso: string | null) => ({ expiresAt: iso }) as RemoteAccount;

  it('null 이면 만료 없음', () => {
    expect(isExpired(at(null), Date.now())).toBe(false);
  });

  it('시각이 지나면 만료', () => {
    const a = at('2026-08-25T00:00:00Z');
    expect(isExpired(a, Date.parse('2026-08-24T23:59:59Z'))).toBe(false);
    expect(isExpired(a, Date.parse('2026-08-25T00:00:01Z'))).toBe(true);
  });

  it('정확히 만료 시각이면 아직 유효', () => {
    const a = at('2026-08-25T00:00:00Z');
    expect(isExpired(a, Date.parse('2026-08-25T00:00:00Z'))).toBe(false);
  });

  it('파싱 못 하는 값은 만료로 보지 않는다 — 서버 오류가 권한 박탈이 되면 안 된다', () => {
    expect(isExpired(at('garbage'), Date.now())).toBe(false);
  });
});

describe('pickAccount — 고르던 로봇 유지', () => {
  const a1 = rowToAccount(ROW);
  const a2 = rowToAccount({ ...ROW, robot_serial: 'RBQ1000000002' });

  it('시리얼이 같은 것을 찾는다', () => {
    expect(pickAccount([a1, a2], 'RBQ1000000002')).toBe(a2);
  });

  it('매칭이 끊겨 목록에서 사라졌으면 첫 번째로 떨어진다', () => {
    expect(pickAccount([a1], 'RBQ1000000002')).toBe(a1);
  });

  it('목록이 비면 null — 없는 로봇에 붙이지 않는다', () => {
    expect(pickAccount([], 'RBQ1000000001')).toBeNull();
  });
});
