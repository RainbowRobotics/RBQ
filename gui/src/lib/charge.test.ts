import { describe, it, expect } from 'vitest';
import { chargeMinutes, chargeView, packInAmps, robotCharging } from './charge';
import type { PduState } from './robotState';

const rail = (v: number, a: number) => ({ v, a });
const pdu = (o: { l?: number; r?: number; swL?: boolean; swR?: boolean; chg?: number } = {}): PduState => ({
  batSwL: o.swL ?? true, batSwR: o.swR ?? true, chgE: false, chgS: true,
  rails: { total: rail(54, 2), batL: rail(54, o.l ?? 0), batR: rail(54, o.r ?? 0), leg: rail(0, 0), add: rail(0, 0), ext: rail(0, 0), chg: rail(58, o.chg ?? 0) },
} as unknown as PduState);
const avg = (a: number) => ({ a, t: 0 });

describe('packInAmps', () => {
  it('두 팩의 충전 전류 합(+ = 충전). BMS 는 + = 방전이라 부호를 뒤집는다', () => { expect(packInAmps(pdu({ l: -6.7, r: -6.8 }))).toBeCloseTo(13.5); });
  it('인식 안 된 팩과 값 없음(327.67)은 뺀다', () => {
    expect(packInAmps(pdu({ l: -9, r: -9, swL: false }))).toBeCloseTo(9);
    expect(packInAmps(pdu({ l: 327.67, r: -8.9 }))).toBeCloseTo(8.9);
  });
});

describe('robotCharging', () => {
  it('FD 는 팩 전류 — 레일이 0 이어도(Bomi 43/54) 팩이 받으면 충전 중', () => { expect(robotCharging(true, avg(8.9), avg(0))).toBe(true); });
  it('FD 완충 뒤 레일 1.4~2.2 A 가 흘러도 팩 0 이면 충전 아님', () => { expect(robotCharging(true, avg(0.1), avg(2.2))).toBe(false); });
  it('클래식은 종전대로 충전 레일 > 1 A', () => { expect(robotCharging(false, null, avg(1.8))).toBe(true); });
});

describe('chargeMinutes — 실측 구간', () => {
  it('Bomi 10-06 대기 충전 8→39%, 팩 13.5 A, 실제 24분', () => { expect(chargeMinutes(8, 39, 2, 13.5)).toBe(25); });
});

describe('chargeView', () => {
  const now = 100_000;
  it('FD 80% 미만 충전 중 — 80%까지 분, 그 뒤는 꼬리 안내', () => {
    const v = chargeView({ fd: true, pdu: pdu({ l: -4.45, r: -4.45 }), pct: 38, packAvg: avg(8.9), now });
    expect(v.to80Min).toBe(51);
    expect(v.tail).toBe(true);
    expect(v.issues).toEqual([]);
  });
  it('80% 넘으면 숫자를 내지 않는다(꼬리만)', () => {
    const v = chargeView({ fd: true, pdu: pdu({ l: -2, r: -2 }), pct: 86, packAvg: avg(4), now });
    expect(v.to80Min).toBeNull();
    expect(v.tail).toBe(true);
  });
  it('팩 하나로 충전 — 용량이 절반이라 시간도 절반 용량으로', () => {
    const v = chargeView({ fd: true, pdu: pdu({ r: -8.9, swL: false }), pct: 38, packAvg: avg(8.9), now });
    expect(v.issues).toEqual(['one-pack']);
    expect(v.to80Min).toBe(25);
  });
  it('단자 연결 20초가 지나도 전류가 없으면 경고, 그 전·완충 직전은 아니다', () => {
    const base = { fd: true, pdu: pdu(), packAvg: avg(0.05) };
    expect(chargeView({ ...base, pct: 68, plugSince: now - 25_000, now }).issues).toEqual(['no-current']);
    expect(chargeView({ ...base, pct: 68, plugSince: now - 10_000, now }).issues).toEqual([]);
    expect(chargeView({ ...base, pct: 97, plugSince: now - 60_000, now }).issues).toEqual([]);
  });
  it('클래식은 시간·예외를 내지 않는다', () => {
    const v = chargeView({ fd: false, pdu: pdu(), pct: 40, railAvg: avg(5), now });
    expect(v).toEqual({ charging: true, amps: null, to80Min: null, tail: false, issues: [] });
  });
});
