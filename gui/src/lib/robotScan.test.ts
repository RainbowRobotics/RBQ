import { describe, it, expect, vi, afterEach } from 'vitest';
import type { Probe } from './robotTls';

const platform = vi.hoisted(() => ({ probe: vi.fn(async (_h: string, _p: number): Promise<Probe> => ({ kind: 'plain' })) }));
vi.mock('./robotTlsPlatform', () => ({ robotTlsPlatform: platform }));

import { probeRobotLan, probeLan, probeAddress, retryLanProbe, LAN_RETRY_MS, ROBOT_LAN_IP } from './robotScan';
import { setTlsIssueHandler, setTlsPinSource, type TlsIssue } from './robotTls';
import { setRobotPasswordSource } from './auth';

const fake = (alive: Record<string, string>) => (async (url: string) => {
  const ip = /http:\/\/([\d.]+):8080/.exec(url)![1];
  if (!(ip in alive)) throw new Error('unreachable');
  return { ok: true, json: async () => ({ serial_number: alive[ip] }) } as Response;
}) as unknown as typeof fetch;

describe('robotScan', () => {
  it('고정 IP 가 응답하면 시리얼(공백 제거)', async () => {
    expect(await probeRobotLan({ fetchFn: fake({ [ROBOT_LAN_IP]: ' RBQ10-001 ' }), timeoutMs: 50 })).toBe('RBQ10-001');
  });
  it('안 닿으면 null, 시리얼 미설정 로봇은 빈 문자열 — 로봇이 없는 것과 구분한다(RBQ_EXAMPLE_5G 실측)', async () => {
    expect(await probeRobotLan({ fetchFn: fake({}), timeoutMs: 50 })).toBeNull();
    expect(await probeRobotLan({ fetchFn: fake({ [ROBOT_LAN_IP]: '' }), timeoutMs: 50 })).toBe('');
  });
});

describe('probeLan — 웹·데스크탑(base "")은 프록시 현재 대상이 아니라 물어본 주소를 확인한다', () => {
  it('프록시가 원격 로봇 id 를 대상으로 잡고 있어도 /probe-robot?ip= 로 그 주소를 묻는다(덱 2026-09-15 재현)', async () => {
    const calls: string[] = [];
    const fetchFn = (async (url: string) => {
      calls.push(url);
      if (url.startsWith('/api/')) throw new Error('unreachable via remote target');
      return { ok: true, json: async () => ({ ok: true, serial: 'RBQ1000000001' }) };
    }) as unknown as typeof fetch;
    expect(await probeLan('192.168.0.10', { base: '', fetchFn })).toBe('RBQ1000000001');
    expect(calls).toEqual(['/probe-robot?ip=192.168.0.10']);
  });
});

describe('retryLanProbe — 처음 한 번에 못 찾았으면 조용히 다시 묻는다(혼잡한 와이파이)', () => {
  it('찾을 때까지 간격마다 묻고, 찾으면 onFound 한 번 — 그 뒤엔 canTry 가 막는다', async () => {
    vi.useFakeTimers();
    let n = 0, found = 0, connected = false;
    const stop = retryLanProbe({ probe: async () => (++n >= 3 ? 'RBQ10-001' : null), canTry: () => !connected, onFound: () => { found++; connected = true; } });
    await vi.advanceTimersByTimeAsync(LAN_RETRY_MS * 2);
    expect(found).toBe(0);
    await vi.advanceTimersByTimeAsync(LAN_RETRY_MS);
    expect(found).toBe(1);
    await vi.advanceTimersByTimeAsync(LAN_RETRY_MS * 3);
    expect(n).toBe(3);
    stop();
    vi.useRealTimers();
  });
  it('물음이 간격보다 오래 걸려도 겹쳐 묻지 않고, 멈추면 늦게 온 답을 버린다', async () => {
    vi.useFakeTimers();
    let inflight = 0, maxInflight = 0, found = 0;
    const stop = retryLanProbe({
      probe: () => { inflight++; maxInflight = Math.max(maxInflight, inflight); return new Promise((r) => setTimeout(() => { inflight--; r('x'); }, LAN_RETRY_MS * 2.5)); },
      canTry: () => true, onFound: () => { found++; },
    });
    await vi.advanceTimersByTimeAsync(LAN_RETRY_MS * 2);
    stop();
    await vi.advanceTimersByTimeAsync(LAN_RETRY_MS * 3);
    expect(maxInflight).toBe(1);
    expect(found).toBe(0);
    vi.useRealTimers();
  });
});

describe('robotScan — TLS 로봇은 비번 없이 probe 의 시리얼로, 평문은 종전대로', () => {
  const pinSource = (pins: Record<string, string>) => setTlsPinSource(() => ({ selectedKey: null, selectedSsid: null, pins, keyFor: async (s) => s, currentSsid: async () => null }));
  afterEach(() => { platform.probe.mockReset(); platform.probe.mockImplementation(async () => ({ kind: 'plain' })); pinSource({}); setTlsIssueHandler(() => {}); });

  it('probeLan_tls_returns_probe_serial_without_auth', async () => {
    platform.probe.mockResolvedValueOnce({ kind: 'tls', fp: 'F', serial: 'B' });
    const fetchFn = vi.fn() as unknown as typeof fetch;
    expect(await probeLan(ROBOT_LAN_IP, { fetchFn, timeoutMs: 50 })).toBe('B');
    expect(platform.probe).toHaveBeenCalledWith(ROBOT_LAN_IP, 8080);
    expect(fetchFn).not.toHaveBeenCalled();
  });

  it('probeLan_down_returns_null_without_fetch', async () => {
    platform.probe.mockResolvedValueOnce({ kind: 'down' });
    const fetchFn = vi.fn() as unknown as typeof fetch;
    expect(await probeLan(ROBOT_LAN_IP, { fetchFn, timeoutMs: 50 })).toBeNull();
    expect(fetchFn).not.toHaveBeenCalled();
  });

  it('probeLan_plain_pinned_serial_refuses', async () => {
    pinSource({ 'S@8080': 'G' });
    const issues: TlsIssue[] = [];
    setTlsIssueHandler((i) => issues.push(i));
    expect(await probeLan(ROBOT_LAN_IP, { fetchFn: fake({ [ROBOT_LAN_IP]: 'S' }), timeoutMs: 50 })).toBeNull();
    expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
  });

  it('probeLan_plain_reads_serial_over_http_even_with_https_base', async () => {
    setRobotPasswordSource(() => 'pw');
    const fetchFn = vi.fn(async () => ({ ok: true, status: 200, json: async () => ({ serial_number: 'OLD-1' }) })) as unknown as typeof fetch;
    expect(await probeLan(ROBOT_LAN_IP, { fetchFn, timeoutMs: 50, base: `https://${ROBOT_LAN_IP}:8080` })).toBe('OLD-1');
    setRobotPasswordSource(() => undefined);
    const [url, init] = vi.mocked(fetchFn).mock.calls[0] as unknown as [string, { headers: Record<string, string> }];
    expect(url).toBe(`http://${ROBOT_LAN_IP}:8080/api/robot/serial_number`);
    expect(init.headers).toEqual(expect.objectContaining({ Authorization: expect.stringMatching(/^Basic /) }));
  });

  it('probeLan_tlsOnly_plain_means_not_this_robot', async () => {
    setRobotPasswordSource(() => 'pw');
    const fetchFn = vi.fn(async () => ({ ok: true, status: 200, json: async () => ({ serial_number: 'Y' }) })) as unknown as typeof fetch;
    expect(await probeLan(ROBOT_LAN_IP, { fetchFn, timeoutMs: 50, tlsOnly: true })).toBeNull();
    setRobotPasswordSource(() => undefined);
    expect(fetchFn).not.toHaveBeenCalled();
  });

  it('probeAddress_no_hardcoded_http', async () => {
    platform.probe.mockResolvedValueOnce({ kind: 'tls', fp: 'F', serial: 'B' });
    const fetchFn = vi.fn() as unknown as typeof fetch;
    expect(await probeAddress('10.0.0.7', false, fetchFn)).toEqual({ serial: 'B' });
    expect(platform.probe).toHaveBeenCalledWith('10.0.0.7', 8080);
    expect(fetchFn).not.toHaveBeenCalled();
  });

  it('probeLan_web_passes_tls_only_to_proxy', async () => {
    const calls: string[] = [];
    const fetchFn = (async (url: string) => { calls.push(url); return { ok: true, json: async () => ({ ok: false, tls: null }) }; }) as unknown as typeof fetch;
    expect(await probeLan('192.168.0.10', { base: '', fetchFn, tlsOnly: true })).toBeNull();
    expect(await probeLan('192.168.0.10', { base: '', fetchFn })).toBeNull();
    expect(calls).toEqual(['/probe-robot?ip=192.168.0.10&tlsOnly=1', '/probe-robot?ip=192.168.0.10']);
  });

  it('probeAddress_web_plain_pinned_serial_refuses', async () => {
    pinSource({ 'S@8080': 'G' });
    const issues: TlsIssue[] = [];
    setTlsIssueHandler((i) => issues.push(i));
    const answer = (tls: string | null) => (async () => ({ ok: true, json: async () => ({ ok: true, serial: 'S', tls }) })) as unknown as typeof fetch;
    expect(await probeAddress('10.0.0.7', true, answer(null))).toBeNull();
    expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
    expect(await probeAddress('10.0.0.7', true, answer('G'))).toEqual({ serial: 'S' });
  });

  it('probeAddress_plain_pinned_serial_refuses', async () => {
    pinSource({ 'S@8081': 'G' });
    const issues: TlsIssue[] = [];
    setTlsIssueHandler((i) => issues.push(i));
    expect(await probeAddress('10.0.0.7', false, fake({ '10.0.0.7': 'S' }))).toBeNull();
    expect(issues).toEqual([{ kind: 'refuse-old', entryKey: 'S' }]);
  });
});
