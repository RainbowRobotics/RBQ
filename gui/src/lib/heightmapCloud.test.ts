import { describe, it, expect } from 'vitest';
import { HmChannel, hmChannelOfSensorId, toBodyFrame, parsePcdBin, pickPcdAt, isHeightmapGait } from './heightmapCloud';

type Frame = { kind: number; stride: 3 | 4; epochMs: number; tf: number[]; pts: number[] };
function buildPcd(frames: Frame[], opt: { version?: number; endEpochMs: number; magic?: string }): ArrayBuffer {
  const bodies = frames.map((f) => 80 + f.pts.length * 4);
  const buf = new ArrayBuffer(32 + bodies.reduce((a, b) => a + b, 0));
  const dv = new DataView(buf);
  const magic = opt.magic ?? 'RPCD';
  for (let i = 0; i < 4; i++) dv.setUint8(i, magic.charCodeAt(i));
  dv.setUint16(4, opt.version ?? 1, true);
  dv.setUint16(6, 32, true);
  dv.setUint32(8, frames.length, true);
  dv.setBigInt64(24, BigInt(opt.endEpochMs), true);
  let pos = 32;
  for (const f of frames) {
    dv.setUint8(pos, f.kind);
    dv.setUint8(pos + 1, f.stride);
    dv.setUint32(pos + 4, f.pts.length / f.stride, true);
    dv.setBigInt64(pos + 8, BigInt(f.epochMs), true);
    f.tf.forEach((v, i) => dv.setFloat32(pos + 16 + i * 4, v, true));
    f.pts.forEach((v, i) => dv.setFloat32(pos + 80 + i * 4, v, true));
    pos += 80 + f.pts.length * 4;
  }
  return buf;
}
const I4 = [1,0,0,0, 0,1,0,0, 0,0,1,0, 0,0,0,1];
const c = Math.cos(Math.PI / 2), s = Math.sin(Math.PI / 2);
const TF_Y90 = [c,-s,0,2,  s,c,0,3,  0,0,1,0.5,  0,0,0,1];

describe('hmChannelOfSensorId', () => {
  it('0x80..0x85 → Map..FootAnswer, 범위 밖·LiDAR 는 null', () => {
    expect(hmChannelOfSensorId(0x80)).toBe(HmChannel.Map);
    expect(hmChannelOfSensorId(0x85)).toBe(HmChannel.FootAnswer);
    expect(hmChannelOfSensorId(0x86)).toBeNull();
    expect(hmChannelOfSensorId(11)).toBeNull();
  });
});

describe('toBodyFrame', () => {
  it('항등 TF 는 좌표 그대로(stride 4 → 3 압축)', () => {
    const out = toBodyFrame(I4, [1, 2, 3, 9, 4, 5, 6, 9], 4, 2);
    expect(Array.from(out)).toEqual([1, 2, 3, 4, 5, 6]);
  });
  it('world 점에 body→world 를 역적용하면 body 점이 나온다', () => {
    const out = toBodyFrame(TF_Y90, [2, 4, 0.5], 3, 1);
    expect(out[0]).toBeCloseTo(1, 5); expect(out[1]).toBeCloseTo(0, 5); expect(out[2]).toBeCloseTo(0, 5);
  });
  it('TF 전부 0 이면 항등으로 간주', () => {
    expect(Array.from(toBodyFrame(new Array(16).fill(0), [7, 8, 9], 3, 1))).toEqual([7, 8, 9]);
  });
});

describe('parsePcdBin', () => {
  const T0 = 1_700_000_000_000;
  it('프레임을 msFromStart 오름차순으로 읽고 body frame 으로 변환한다(meta 기준)', () => {
    const buf = buildPcd([
      { kind: HmChannel.Map,       stride: 3, epochMs: T0 + 300, tf: TF_Y90, pts: [2, 4, 0.5] },
      { kind: HmChannel.FootQuery, stride: 4, epochMs: T0 + 100, tf: I4,     pts: [1, 1, 1, 7] },
    ], { endEpochMs: T0 + 1000 });
    const fr = parsePcdBin(buf, { frameZeroEpochMs: T0, durMs: 1000 });
    expect(fr.map((f) => [f.ch, f.msFromStart])).toEqual([[HmChannel.FootQuery, 100], [HmChannel.Map, 300]]);
    expect(fr[1].frame.positions[0]).toBeCloseTo(1, 5);
    expect(Array.from(fr[0].frame.w!)).toEqual([7]);
    expect(fr[1].frame.w).toBeNull();
  });
  it('meta 없으면 파일 end 를 창의 끝(durMs)에 정렬', () => {
    const buf = buildPcd([{ kind: 0, stride: 3, epochMs: T0 + 800, tf: I4, pts: [0, 0, 0] }], { endEpochMs: T0 + 1000 });
    expect(parsePcdBin(buf, { frameZeroEpochMs: null, durMs: 5000 })[0].msFromStart).toBe(4800);
  });
  it('numPoints 0 프레임은 클리어 신호로 유지, 미지 kind 는 건너뜀', () => {
    const buf = buildPcd([
      { kind: HmChannel.Stair, stride: 3, epochMs: T0, tf: I4, pts: [] },
      { kind: 42,              stride: 3, epochMs: T0, tf: I4, pts: [1, 2, 3] },
    ], { endEpochMs: T0 });
    const fr = parsePcdBin(buf, { frameZeroEpochMs: T0, durMs: 0 });
    expect(fr).toHaveLength(1);
    expect(fr[0].frame.count).toBe(0);
  });
  it('bad magic / 버전 불일치 / 잘린 프레임은 안전하게 멈춘다', () => {
    expect(parsePcdBin(buildPcd([], { endEpochMs: 0, magic: 'XXXX' }), { frameZeroEpochMs: null, durMs: 0 })).toEqual([]);
    expect(parsePcdBin(buildPcd([], { endEpochMs: 0, version: 2 }), { frameZeroEpochMs: null, durMs: 0 })).toEqual([]);
    const ok = buildPcd([{ kind: 0, stride: 3, epochMs: 0, tf: I4, pts: [1, 2, 3] }], { endEpochMs: 0 });
    const cut = ok.slice(0, ok.byteLength - 4);
    expect(parsePcdBin(cut, { frameZeroEpochMs: 0, durMs: 0 })).toEqual([]);
    expect(parsePcdBin(new ArrayBuffer(3), { frameZeroEpochMs: 0, durMs: 0 })).toEqual([]);
  });
});

describe('pickPcdAt', () => {
  it('채널별 nowMs 이하 마지막 프레임 — 미래 프레임 무시, 같은 참조 유지', () => {
    const mk = (ch: HmChannel, ms: number) => ({ ch, msFromStart: ms, frame: { positions: new Float32Array(0), w: null, count: 0, at: ms } });
    const frames = [mk(0, 0), mk(1, 50), mk(0, 100), mk(0, 200)];
    const at120 = pickPcdAt(frames, 120);
    expect(at120[HmChannel.Map]).toBe(frames[2].frame);
    expect(at120[HmChannel.Edge]).toBe(frames[1].frame);
    expect(at120[HmChannel.Stair]).toBeUndefined();
    expect(pickPcdAt(frames, -1)).toEqual({});
  });
});

describe('isHeightmapGait', () => {
  it('TROT_STAIRS(4)·RL_WALK_VISION(48) 에서만 true — 그 외 gait 는 잔상 방지로 숨긴다', () => {
    expect(isHeightmapGait(4)).toBe(true);
    expect(isHeightmapGait(48)).toBe(true);
    for (const g of [-1, 0, 1, 3, 30, 49]) expect(isHeightmapGait(g)).toBe(false);
    expect(isHeightmapGait(undefined)).toBe(false);
  });
});
