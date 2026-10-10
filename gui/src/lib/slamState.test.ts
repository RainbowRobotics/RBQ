import { describe, it, expect } from 'vitest';
import { parseSlamState, SLAM_STATE_SIZE } from './slamState';

function frame(opts: {
  camel?: Record<number, boolean>;
  lower?: Record<number, boolean>;
  driveGoal?: number; mapColor?: number; pointSize?: number; plotView?: number;
} = {}) {
  const buf = new ArrayBuffer(SLAM_STATE_SIZE);
  const dv = new DataView(buf);
  Object.entries(opts.camel ?? {}).forEach(([i, v]) => dv.setUint8(8 + Number(i), v ? 1 : 0));
  Object.entries(opts.lower ?? {}).forEach(([i, v]) => dv.setUint8(32 + Number(i), v ? 1 : 0));
  dv.setInt32(56, opts.driveGoal ?? 0, true);
  dv.setUint8(68, opts.mapColor ?? 0);
  dv.setUint8(69, opts.pointSize ?? 0);
  dv.setUint8(70, opts.plotView ?? 0);
  return buf;
}

describe('parseSlamState — CamelCase 블록을 읽는다', () => {
  it('전부 0이면 전부 false', () => {
    const s = parseSlamState(frame());
    expect(s.mapBuilding).toBe(false);
    expect(s.driveStarted).toBe(false);
    expect(s.beep).toBe(false);
  });

  it('선언 순서대로 오프셋이 맞는다 (mapBuilding=5, locaStarted=10, driveStarted=11)', () => {
    expect(parseSlamState(frame({ camel: { 5: true } })).mapBuilding).toBe(true);
    expect(parseSlamState(frame({ camel: { 10: true } })).locaStarted).toBe(true);
    expect(parseSlamState(frame({ camel: { 11: true } })).driveStarted).toBe(true);
    const s = parseSlamState(frame({ camel: { 5: true } }));
    expect(s.recLoaded).toBe(false);
    expect(s.mapSaved).toBe(false);
  });

  it('첫/마지막 CamelCase 필드 (lidarConnected=0, driveMode=23)', () => {
    expect(parseSlamState(frame({ camel: { 0: true } })).lidarConnected).toBe(true);
    expect(parseSlamState(frame({ camel: { 23: true } })).driveMode).toBe(true);
  });

  it('★소문자 블록(32..55)을 읽으면 안 된다 — 로봇이 안 채우는 곳이다', () => {
    const s = parseSlamState(frame({ lower: { 5: true } }));
    expect(s.mapBuilding).toBe(false);
  });

  it('beep 만은 소문자 블록에서 읽는다 (offset 55) — 로봇이 쓰는 유일한 소문자 필드', () => {
    expect(parseSlamState(frame({ lower: { 23: true } })).beep).toBe(true);
  });

  it('숫자 필드 오프셋 (driveGoal 56, mapColor/pointSize/plotView 68/69/70)', () => {
    const s = parseSlamState(frame({ driveGoal: 7, mapColor: 2, pointSize: 3, plotView: 1 }));
    expect(s.driveGoal).toBe(7);
    expect(s.mapColor).toBe(2);
    expect(s.pointSize).toBe(3);
    expect(s.plotView).toBe(1);
  });

  it('플래그와 숫자가 서로 침범하지 않는다', () => {
    const s = parseSlamState(frame({ camel: { 23: true }, driveGoal: 1 }));
    expect(s.driveMode).toBe(true);
    expect(s.driveGoal).toBe(1);
    expect(s.beep).toBe(false);
  });
});
