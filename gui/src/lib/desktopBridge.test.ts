import { describe, it, expect } from 'vitest';
import { requestOnce, sortAndDedup, type WifiNetwork } from './desktopBridge';

function mockTx() {
  let cb: ((p: any) => void) | null = null;
  let emitted: any = null;
  return {
    tx: {
      listen: async (_e: string, c: (p: any) => void) => { cb = c; return () => {}; },
      emit: async (_e: string, p: any) => { emitted = p; },
    },
    fire: (p: any) => cb && cb(p),
    emitted: () => emitted,
  };
}
const flush = () => new Promise((r) => setTimeout(r, 0));

describe('requestOnce', () => {
  it('matching reqId로 resolve, 다른 reqId는 무시', async () => {
    const m = mockTx();
    const p = requestOnce<number[]>(m.tx, 'wifi-scan', 'wifi-scan-result', {}, 1000);
    await flush();
    const reqId = m.emitted().reqId;
    m.fire({ reqId: 'other', ok: true, data: [9] });
    m.fire({ reqId, ok: true, data: [1, 2] });
    await expect(p).resolves.toEqual([1, 2]);
  });

  it('ok:false면 error로 reject', async () => {
    const m = mockTx();
    const p = requestOnce(m.tx, 'wifi-connect', 'wifi-connect-result', {}, 1000);
    await flush();
    m.fire({ reqId: m.emitted().reqId, ok: false, error: '비번 틀림' });
    await expect(p).rejects.toThrow('비번 틀림');
  });

  it('타임아웃이면 reject', async () => {
    const m = mockTx();
    const p = requestOnce(m.tx, 'wifi-scan', 'wifi-scan-result', {}, 5);
    await expect(p).rejects.toThrow('응답 없음');
  });
});

describe('sortAndDedup', () => {
  it('SSID 중복은 최고 신호로 병합, 신호 내림차순 정렬', () => {
    const raw: WifiNetwork[] = [
      { ssid: 'A', signal: 30, secured: true, active: false },
      { ssid: 'B', signal: 80, secured: false, active: false },
      { ssid: 'A', signal: 60, secured: true, active: true },
    ];
    const out = sortAndDedup(raw);
    expect(out.map((n) => n.ssid)).toEqual(['B', 'A']);
    expect(out.find((n) => n.ssid === 'A')!.signal).toBe(60);
    expect(out.find((n) => n.ssid === 'A')!.active).toBe(true);
  });
});
