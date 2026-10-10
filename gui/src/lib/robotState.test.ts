import { describe, it, expect } from 'vitest';
import { parseRobotState } from './robotState';

const JOINT_BASE = 228;
const JOINT_SIZE = 16;

function halfBits(v: number): number {
  if (v === 0) return 0;
  const f32 = new Float32Array(1);
  const u32 = new Uint32Array(f32.buffer);
  f32[0] = v;
  const x = u32[0];
  const sign = (x >>> 16) & 0x8000;
  const exp = ((x >>> 23) & 0xff) - 127 + 15;
  if (exp < 1 || exp > 0x1e) throw new Error(`half 정상수 범위 밖: ${v}`);
  return sign | (exp << 10) | ((x >>> 13) & 0x3ff);
}

function frame(joint: { temperature: number; current: number; status: number; err: number; stator: number; kp: number; position: number; torque: number }) {
  const buf = new ArrayBuffer(592);
  const dv = new DataView(buf);
  dv.setUint8(224, 13);
  const o = JOINT_BASE;
  dv.setUint8(o + 0, 1);
  dv.setInt8(o + 1, joint.temperature);
  dv.setUint16(o + 2, halfBits(joint.current), true);
  dv.setUint8(o + 4, joint.status);
  dv.setUint8(o + 5, joint.err);
  dv.setUint8(o + 6, joint.stator);
  dv.setUint16(o + 8, joint.kp, true);
  dv.setUint16(o + 12, halfBits(joint.position), true);
  dv.setUint16(o + 14, halfBits(joint.torque), true);
  return buf;
}

const J = {
  temperature: 41,
  current: -9.25,
  status: 0b1000010,
  err: 0b10,
  stator: 57,
  kp: 300,
  position: 0.5,
  torque: 12.5,
};

describe('parseRobotState — 관절 슬롯 16B 배치', () => {
  const j = parseRobotState(frame(J)).joints[0];

  it('전류는 @2(옛 패딩)에서 읽고, 이웃 칸을 건드리지 않는다', () => {
    expect(j.current).toBe(-9.25);
    expect(j.temperature).toBe(41);
    expect(j.run).toBe(true);
    expect(j.calib).toBe(true);
  });

  it('전류 추가로 position·torque·에러·stator 온도 자리가 밀리지 않았다', () => {
    expect(j.position).toBe(0.5);
    expect(j.torque).toBe(12.5);
    expect(j.errors).toEqual(['CUR']);
    expect(j.statorTemp).toBe(57);
    expect(j.locked).toBe(true);
  });

  it('592B 프레임 하나에 관절 13칸이 다 들어간다(스트라이드 16B)', () => {
    const st = parseRobotState(frame(J));
    expect(st.jointCount).toBe(13);
    expect(st.joints).toHaveLength(13);
    expect(JOINT_BASE + 13 * JOINT_SIZE).toBeLessThanOrEqual(592);
  });

  it('구버전 로봇(@2 가 패딩 0)은 전류 0 으로 온다 — 크래시 없음', () => {
    expect(parseRobotState(frame({ ...J, current: 0 })).joints[0].current).toBe(0);
  });

  it('FD 환산이 0으로 나눠 무한대가 와도 숫자로 통과시킨다(표에서 "-" 로 거른다)', () => {
    const buf = frame(J);
    new DataView(buf).setUint16(JOINT_BASE + 2, 0x7c00, true);
    expect(Number.isFinite(parseRobotState(buf).joints[0].current)).toBe(false);
  });
});
