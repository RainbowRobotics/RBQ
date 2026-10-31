import { describe, it, expect } from 'vitest';
import { parsePduState, isRobotCharging, nextChargeAvg, type ChargeAvg } from './robotState';

const HEX =
  '0000000000000000e583a9a385a5da41150ce2df9609d11800000000000000010000000000000000' +
  '4f0137201c3100000000000096520000965200009652000069529a3f6d526626003d662e96526d4a' +
  '00000000e1e7b740000000003a360000470000000000280001000000000000000000000000000000' +
  '00000000000000000000000000000000000000000100000000000000000000000000000000000000' +
  '00000000000000000000000000000000000000000000000000000000000000000107807f00000000';

function frame(): ArrayBuffer {
  const hex = HEX.replace(/\s/g, '');
  const b = new Uint8Array(200);
  for (let i = 0; i < 200; i++) b[i] = parseInt(hex.slice(i * 2, i * 2 + 2), 16);
  return b.buffer;
}

describe('parsePduState — 실기 프레임 오프셋', () => {
  const p = parsePduState(frame());

  it('꼬리(typeVersion/typeId/tail1/tail2)가 @192..195 → 레일은 @52 부터', () => {
    const b = new Uint8Array(frame());
    expect([b[192], b[193], b[194], b[195]]).toEqual([0x01, 0x07, 0x80, 0x7f]);
  });

  it('FET 비트와 레일 전압이 서로 맞는다 — 켜진 레일만 전압이 산다', () => {
    expect(p.fetLeg).toBe(true);
    expect(p.fetAdd).toBe(true);
    expect(p.fetExt).toBe(false);
    expect(p.rails.leg.v).toBeGreaterThan(40);
    expect(p.rails.add.v).toBeGreaterThan(40);
    expect(p.rails.ext.v).toBeLessThan(5);
  });

  it('배터리 팩 전압은 total/batL/batR 이 같은 값이다', () => {
    expect(p.rails.total.v).toBeGreaterThan(40);
    expect(p.rails.batL.v).toBeCloseTo(p.rails.total.v, 1);
    expect(p.rails.batR.v).toBeCloseTo(p.rails.total.v, 1);
  });

  it('충전 레일 전압은 음수가 아니다(밀린 오프셋의 전형적 증상)', () => {
    expect(p.rails.chg.v).toBeGreaterThanOrEqual(0);
  });

  it('배터리%·온도가 실제 값으로 읽힌다(예전엔 batPct 가 늘 0 이었다)', () => {
    expect(p.batPct).toBeGreaterThan(0);
    expect(p.batPct).toBeLessThanOrEqual(100);
    expect(p.tempPdu).toBeGreaterThan(0);
    expect(p.tempPs).toBeGreaterThan(0);
  });
});

describe('isRobotCharging — 비트가 아니라 충전 전류의 이동 평균으로 본다', () => {
  const p = parsePduState(frame());
  const run = (amps: number[]) => {
    let avg: ChargeAvg | null = null;
    return amps.map((a, i) => { avg = nextChargeAvg(avg, a, i * 200); return isRobotCharging(avg); });
  };

  it('충전기가 물린 프레임이면 참 (chg 전류가 실제로 흐른다)', () => {
    expect(p.rails.chg.a).toBeGreaterThan(2);
    expect(run([p.rails.chg.a]).at(-1)).toBe(true);
  });

  it('대기 전류(0.45A)는 충전이 아니다 — 임계 0.3A 였을 때 오탐이 났다', () => {
    expect(run(Array(50).fill(0.45)).some(Boolean)).toBe(false);
  });

  it('저전류 충전(0.75~2.44A 요동, 평균 1.78A)에서 깜빡이지 않는다', () => {
    const seq = [0.91, 1.94, 1.26, 2.34, 1.30, 1.45, 1.69, 1.97, 0.94, 1.96, 2.14, 2.39, 2.19, 1.24, 1.97, 2.21, 1.76, 1.31, 1.96, 1.20, 2.32];
    const out = run([...seq, ...seq, ...seq]);
    expect(out.slice(5).every(Boolean)).toBe(true);
  });

  it('충전기를 빼면 몇 초 안에 거짓이 된다', () => {
    const out = run([...Array(30).fill(5), ...Array(50).fill(0.45)]);
    expect(out[29]).toBe(true);
    expect(out.at(-1)).toBe(false);
  });

  it('chgE·chgS 비트로 판정하면 안 된다 — 충전 중인데도 둘 다 false 였다', () => {
    expect(p.chgE).toBe(false);
    expect(p.chgS).toBe(false);
  });

  it('PDU 미수신이면 거짓', () => {
    expect(isRobotCharging(null)).toBe(false);
    expect(isRobotCharging(undefined)).toBe(false);
  });
});
