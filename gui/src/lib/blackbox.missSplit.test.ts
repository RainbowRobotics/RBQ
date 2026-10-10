import { describe, it, expect, vi } from 'vitest';

vi.mock('@/lib/rest', () => ({ rest: {} }));
import { buildSession, chan, chanOpt } from './blackbox';

function missSrc(s: ReturnType<typeof buildSession>, f: number): string {
  const missM = (chanOpt(s, f, 'deadline_miss.motion') ?? 0) > 0.5;
  const missW = (chanOpt(s, f, 'deadline_miss.wbc') ?? 0) > 0.5;
  return missM || missW ? ` ${[missM && 'M', missW && 'W'].filter(Boolean).join('+')}` : '';
}

const mk = (header: string[], rows: number[][]) =>
  buildSession({
    metaTxt: JSON.stringify({ hz: 100 }),
    dataTxt: [header.join('\t'), ...rows.map((r) => r.join('\t'))].join('\n'),
    logTxt: '', videoTxt: '', session: 't', date: '20260824',
  });

describe('blackbox miss 출처 구분', () => {
  it('신 녹화 — motion만 미스면 " M"', () => {
    const s = mk(['deadline_miss', 'deadline_miss.motion', 'deadline_miss.wbc'], [[1, 1, 0]]);
    expect(chan(s, 0, 'deadline_miss')).toBe(1);
    expect(missSrc(s, 0)).toBe(' M');
  });
  it('신 녹화 — 둘 다 미스면 " M+W"', () => {
    const s = mk(['deadline_miss', 'deadline_miss.motion', 'deadline_miss.wbc'], [[1, 1, 1]]);
    expect(missSrc(s, 0)).toBe(' M+W');
  });
  it('신 녹화 — 미스 없으면 접미사 없음', () => {
    const s = mk(['deadline_miss', 'deadline_miss.motion', 'deadline_miss.wbc'], [[0, 0, 0]]);
    expect(missSrc(s, 0)).toBe('');
  });
  it('구 녹화(세부 채널 없음) — 미스여도 접미사 없이 합만 (하위 호환)', () => {
    const s = mk(['deadline_miss', 'process_time_ms'], [[1, 2.1]]);
    expect(chan(s, 0, 'deadline_miss')).toBe(1);
    expect(missSrc(s, 0)).toBe('');
  });
});
