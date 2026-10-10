import { describe, it, expect } from 'vitest';
import { probeRobotLan, probeLan, ROBOT_LAN_IP } from './robotScan';

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
      if (url.startsWith('/api/')) throw new Error('unreachable via rendezvous target');
      return { ok: true, json: async () => ({ ok: true, serial: 'RBQ1000000001' }) };
    }) as unknown as typeof fetch;
    expect(await probeLan('192.168.0.10', { base: '', fetchFn })).toBe('RBQ1000000001');
    expect(calls).toEqual(['/probe-robot?ip=192.168.0.10']);
  });
});
