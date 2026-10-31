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

function frame(joint: { temperature: number; current: number; status: number; err: number; stator: number; kp: number; kd: number; position: number; torque: number }) {
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
  dv.setUint16(o + 10, joint.kd, true);
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
  kp: 2997,
  kd: 302,
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

  it('게인은 KP@8 ÷10 · KD@10 ÷100 으로 물리 단위가 된다', () => {
    expect(j.kp).toBeCloseTo(299.7, 5);
    expect(j.kd).toBeCloseTo(3.02, 5);
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


describe('FC display telemetry in the existing 592-byte packet', () => {
  it('decodes signed FC yaw, including zero, without changing velocity or trip fields', () => {
    for (const yaw of [0, -1.2345, 2.1, -3.1416, 3.1416]) {
      const buf = frame(J), dv = new DataView(buf);
      dv.setBigUint64(548, 12345n, true);
      dv.setFloat32(572, 0.5, true);
      dv.setInt16(584, Math.round(yaw * 10000), true);
      dv.setUint16(586, 0xFC02, true);
      const state = parseRobotState(buf);
      expect(Math.atan2(Math.sin(state.fcYaw!-yaw),Math.cos(state.fcYaw!-yaw))).toBeCloseTo(0, 4);
      expect(state.cmdVel).toEqual([0.5, 0, 0]);
      expect(state.tripTotals.distMm).toBe(12345);
      expect(state.joints[0].position).toBe(0.5);
    }
  });
  it('does not mistake legacy zeros, unknown versions or malformed yaw for FC', () => {
    const buf = frame(J), dv = new DataView(buf);
    expect(parseRobotState(buf).fcYaw).toBeUndefined();
    dv.setUint16(586, 0xFC03, true);
    expect(parseRobotState(buf).fcYaw).toBeUndefined();
    dv.setUint16(586, 0xFC02, true);
    dv.setInt16(584, 32767, true);
    expect(parseRobotState(buf).fcYaw).toBeUndefined();
    const prefixed = new Uint8Array(593);
    prefixed.set(new Uint8Array(buf), 1);
    dv.setInt16(584, -3000, true);
    prefixed.set(new Uint8Array(buf), 1);
    expect(parseRobotState(prefixed.buffer, 1).fcYaw).toBeCloseTo(-0.3);
  });
});

describe('EE speed support telemetry',()=>{
 it('requires the explicit reserved arm-status capability bit',()=>{
  const buf=frame(J),dv=new DataView(buf);
  dv.setUint8(51,6);expect(parseRobotState(buf).armStat.eeSpeedControl).toBe(false);
  dv.setUint8(51,14);const state=parseRobotState(buf);
  expect(state.armStat.eeSpeedControl).toBe(true);
  expect(state.armStat.manualControl).toBe(true);expect(state.armStat.lockPosition).toBe(true);
 });
});

describe('FC yaw after moving and re-entering manual control',()=>{
 it('keeps the FC/body relationship when odometry and control yaw have different origins',()=>{
  for(const bodyYaw of [0,1,-2,3,-3])for(const offset of [0,-.25,.4]){
   const buf=frame(J),dv=new DataView(buf);
   dv.setUint16(138,halfBits(bodyYaw),true);dv.setInt16(584,Math.round(offset*10000),true);dv.setUint16(586,0xFC02,true);
   const yaw=parseRobotState(buf).fcYaw!;
   expect(Math.atan2(Math.sin(yaw-bodyYaw),Math.cos(yaw-bodyYaw))).toBeCloseTo(offset,4);
  }
 });
 it('keeps legacy axes visible and rejects nonfinite odometry for FC02',()=>{
  const buf=frame(J),dv=new DataView(buf);dv.setUint16(586,0xFC01,true);
  expect(parseRobotState(buf).fcYaw).toBe(0);
  dv.setUint16(138,halfBits(2),true);dv.setInt16(584,-2500,true);
  expect(parseRobotState(buf).fcYaw).toBe(-.25);
  dv.setUint16(586,0xFC02,true);dv.setUint16(138,0x7e00,true);
  expect(parseRobotState(buf).fcYaw).toBeUndefined();
 });
});

describe('FC telemetry version compatibility',()=>{
 it('distinguishes legacy absolute yaw from corrected body-relative yaw without losing zero axes',()=>{
  const buf=frame(J),dv=new DataView(buf);dv.setUint16(138,halfBits(2),true);
  for(const angle of [0,.3,-1.2]){
   dv.setInt16(584,Math.round(angle*10000),true);
   dv.setUint16(586,0xFC01,true);expect(parseRobotState(buf).fcYaw).toBeCloseTo(angle,4);
   dv.setUint16(586,0xFC02,true);expect(parseRobotState(buf).fcYaw).toBeCloseTo(2+angle,4);
  }
  dv.setUint16(586,0xFC01,true);dv.setInt16(584,32767,true);expect(parseRobotState(buf).fcYaw).toBeUndefined();
 });
});

describe('parseRobotState — 에러 bit 6 이름', () => {
  function bit6(arm: boolean) {
    const buf = frame({ ...J, err: 0b1000000 });
    const dv = new DataView(buf);
    dv.setUint8(JOINT_BASE + 12 * JOINT_SIZE + 5, 0b1000000);
    dv.setUint8(40, arm ? 1 : 0);
    return parseRobotState(buf).joints;
  }

  it('다리 · 바퀴는 CMD(지령 끊김)', () => {
    const js = bit6(false);
    expect(js[0].errors).toEqual(['CMD']);
    expect(js[12].errors).toEqual(['CMD']);
  });

  it('팔이 붙으면 12번부터 팔 관절이라 PS1', () => {
    const js = bit6(true);
    expect(js[0].errors).toEqual(['CMD']);
    expect(js[12].errors).toEqual(['PS1']);
  });
});
