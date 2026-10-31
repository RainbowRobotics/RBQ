import { describe, it, expect } from 'vitest';
import { routeFromStats, routeDetail, routeLabel, linkQualityPct } from './connectionRoute';

const cand = (id: string, candidateType: string) => ({ id, type: 'local-candidate', candidateType });
const pair = (o: Record<string, unknown> = {}) => ({
  type: 'candidate-pair', state: 'succeeded', nominated: true,
  localCandidateId: 'L', remoteCandidateId: 'R', ...o,
});

describe('routeFromStats', () => {
  it('양쪽 다 host/srflx 면 P2P', () => {
    expect(routeFromStats([pair(), cand('L', 'srflx'), cand('R', 'host')])).toBe('p2p');
  });

  it('한쪽만 relay 여도 릴레이다 — 그 구간이 TURN 을 지난다', () => {
    expect(routeFromStats([pair(), cand('L', 'relay'), cand('R', 'srflx')])).toBe('relay');
    expect(routeFromStats([pair(), cand('L', 'srflx'), cand('R', 'relay')])).toBe('relay');
  });

  it('후보쌍이 없으면 unknown — 협상 중이다', () => {
    expect(routeFromStats([cand('L', 'host')])).toBe('unknown');
  });

  it('후보쌍은 있는데 후보를 못 찾으면 unknown — p2p 로 단정하지 않는다', () => {
    expect(routeFromStats([pair()])).toBe('unknown');
  });

  it('selected 플래그를 우선한다', () => {
    const stats = [
      pair({ selected: false, state: 'failed', nominated: false, localCandidateId: 'X' }),
      pair({ selected: true, localCandidateId: 'L', remoteCandidateId: 'R' }),
      cand('L', 'relay'), cand('R', 'relay'), cand('X', 'host'),
    ];
    expect(routeFromStats(stats)).toBe('relay');
  });

  it('nominated 가 없으면 succeeded 만으로도 찾는다', () => {
    const stats = [pair({ nominated: false }), cand('L', 'host'), cand('R', 'host')];
    expect(routeFromStats(stats)).toBe('p2p');
  });
});

describe('routeLabel', () => {
  it('모르면 아무 말도 하지 않는다', () => {
    expect(routeLabel('unknown')).toBe('');
    expect(routeLabel('p2p')).toBe('P2P');
    expect(routeLabel('relay')).toBe('릴레이');
  });
});

describe('routeDetail — 릴레이 수송', () => {
  it('udp 릴레이', () => {
    const r = routeDetail([pair(), { ...cand('L', 'relay'), relayProtocol: 'udp' }, cand('R', 'srflx')]);
    expect(r).toEqual({ route: 'relay', proto: 'udp' });
  });

  it('tls 면 443 폴백 — 그 망이 UDP 를 막는다는 신호', () => {
    const r = routeDetail([pair(), { ...cand('L', 'relay'), relayProtocol: 'tls' }, cand('R', 'srflx')]);
    expect(r.proto).toBe('tls');
    expect(routeLabel(r.route, r.proto)).toBe('릴레이(tls)');
  });

  it('P2P 면 수송은 없다', () => {
    expect(routeDetail([pair(), cand('L', 'host'), cand('R', 'host')])).toEqual({ route: 'p2p', proto: null });
  });

  it('수송을 모르면 그냥 릴레이', () => {
    expect(routeLabel('relay', null)).toBe('릴레이');
  });
});

describe('linkQualityPct', () => {
  it('RTT 로 막대 칸 수가 정해진다', () => {
    const q = (rtt: number) => linkQualityPct([pair({ currentRoundTripTime: rtt })]);
    expect(q(0.05)).toBeGreaterThan(90);
    expect(q(0.2)).toSatisfy((v: number) => v > 80 && v <= 90);
    expect(q(0.4)).toSatisfy((v: number) => v > 55 && v <= 80);
    expect(q(0.8)).toSatisfy((v: number) => v > 30 && v <= 55);
    expect(q(2)).toSatisfy((v: number) => v > 0 && v <= 30);
  });
  it('RTT 가 없으면 null — 모르는 걸 나쁨으로 그리지 않는다', () => {
    expect(linkQualityPct([pair()])).toBeNull();
    expect(linkQualityPct([])).toBeNull();
  });
});
