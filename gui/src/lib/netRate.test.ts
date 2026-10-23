import { describe, it, expect } from 'vitest';
import { wireBytesFromStats } from './connectionRoute';
import { fmtRate } from './netMeter';

const withPair = [
  { type: 'candidate-pair', state: 'succeeded', nominated: true, bytesReceived: 1_000, bytesSent: 40 },
  { type: 'transport', bytesReceived: 9_999, bytesSent: 9_999 },
  { type: 'inbound-rtp', kind: 'video', bytesReceived: 900 },
];

describe('wireBytesFromStats — 이 피어가 회선에 흘린 누적 바이트', () => {
  it('고른 후보쌍이 정본이다(transport 와 겹쳐도 이중계산 없음)', () => {
    expect(wireBytesFromStats(withPair)).toEqual({ rx: 1_000, tx: 40 });
  });

  it('후보쌍에 바이트가 없으면 transport 로 폴백', () => {
    const stats = [
      { type: 'candidate-pair', state: 'succeeded', nominated: true },
      { type: 'transport', bytesReceived: 500, bytesSent: 20 },
    ];
    expect(wireBytesFromStats(stats)).toEqual({ rx: 500, tx: 20 });
  });

  it('둘 다 없으면 null', () => {
    expect(wireBytesFromStats([{ type: 'inbound-rtp', kind: 'video' }])).toBeNull();
  });
});

describe('fmtRate — 합계 하나, 구형과 같은 단위', () => {
  it('자리수에 맞춰 단위를 고른다', () => {
    expect(fmtRate(0)).toBe('0 B/s');
    expect(fmtRate(940)).toBe('940 B/s');
    expect(fmtRate(1024)).toBe('1.0 KB/s');
    expect(fmtRate(48_000)).toBe('46.9 KB/s');
    expect(fmtRate(1_048_576)).toBe('1.0 MB/s');
    expect(fmtRate(2_400_000)).toBe('2.3 MB/s');
  });

  it('1024 경계를 넘을 때만 윗단위로 간다', () => {
    expect(fmtRate(1023)).toBe('1023 B/s');
    expect(fmtRate(1_048_575)).toBe('1.0 MB/s');
  });
});
