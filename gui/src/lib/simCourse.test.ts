import { describe, it, expect } from 'vitest';
import { FALL_HOLD_MS, LEVELS, stepCourse, respawnPoint, killZ, fmtTime, groundAt, footprintGround, STUCK_MS, type Level, type Progress } from './simCourse';

const L: Level = {
  id: 't', no: 1, name: '시험',
  spawn: { x: 0, y: 0, z: 0, yaw: 0 },
  gates: [{ x: 3, y: 0, z: 0 }, { x: 6, y: 0, z: 0.5, facing: 90, tol: 30 }],
  goal: { x: 9, y: 0, z: 0.5, hx: 0.8, hy: 0.8 },
  boxes: [{ x: 4, y: 0, z: -0.15, hx: 6, hy: 1, hz: 0.15, m: 'c' }],
};
const P0: Progress = { passed: 0, done: false };
const at = (x: number, y: number, z: number, yawDeg = 0) => ({ x, y, z, yaw: (yawDeg * Math.PI) / 180 });

describe('시뮬 코스 판정', () => {
  it('게이트를 건너뛰고 결승에 가도 클리어가 아니다(지름길 차단)', () => {
    const r = stepCourse(L, P0, at(9, 0, 1.0));
    expect(r.ev).toBeNull();
    expect(r.prog.done).toBe(false);
  });

  it('게이트는 순서대로만 — 2번 자리에 먼저 서도 1번이 안 되어 있으면 무시', () => {
    expect(stepCourse(L, P0, at(6, 0, 1.0, 90)).ev).toBeNull();
    const r = stepCourse(L, P0, at(3.2, 0.3, 0.5));
    expect(r.ev).toEqual({ t: 'gate', n: 1, total: 2 });
  });

  it('게이트 높이가 다르면(아래층·공중) 통과가 아니다', () => {
    expect(stepCourse(L, P0, at(3, 0, 2.5)).ev).toBeNull();
    expect(stepCourse(L, P0, at(3, 0, 0.05)).ev).toBeNull();
  });

  it('옆걸음 게이트는 정면이 요구 방향일 때만 통과 — 돌아서 걸어가면 인정 안 됨', () => {
    const p1: Progress = { passed: 1, done: false };
    expect(stepCourse(L, p1, at(6, 0, 1.0, 0)).ev).toEqual({ t: 'wrongFacing', n: 2 });
    expect(stepCourse(L, p1, at(6, 0, 1.0, 110)).ev).toEqual({ t: 'gate', n: 2, total: 2 });
    expect(stepCourse(L, p1, at(6, 0, 1.0, -270)).ev?.t).toBe('gate');
  });

  it('게이트를 다 지난 뒤 결승 발판 위에 서면 클리어, 그 뒤로는 이벤트 없음', () => {
    const p2: Progress = { passed: 2, done: false };
    const r = stepCourse(L, p2, at(9.3, 0.2, 1.0));
    expect(r.ev).toEqual({ t: 'clear' });
    expect(r.prog.done).toBe(true);
    expect(stepCourse(L, r.prog, at(9.3, 0.2, -5)).ev).toBeNull();
  });

  it('떨어지면 마지막 통과 게이트로, 방향은 다음 목표를 향해', () => {
    expect(killZ(L)).toBeCloseTo(-1.2);
    const r = stepCourse(L, { passed: 1, done: false }, at(4, 3, -2));
    expect(r.ev?.t).toBe('fall');
    expect(r.ev && r.ev.t === 'fall' && r.ev.to).toEqual({ x: 3, y: 0, z: 0, yaw: 0 });
    expect(respawnPoint(L, 0)).toEqual(L.spawn);
    expect(respawnPoint(L, 2).yaw).toBe(90);
  });

  it('복귀 직후 큐에 남은 낙하 pose 는 다시 세지 않는다', () => {
    const r1 = stepCourse(L, { passed: 1, done: false }, at(4, 3, -2), 1000);
    expect(r1.ev?.t).toBe('fall');
    expect(stepCourse(L, r1.prog, at(4, 3, -2.1), 1100).ev).toBeNull();
    expect(stepCourse(L, r1.prog, at(4, 3, -2), 1000 + FALL_HOLD_MS + 1).ev?.t).toBe('fall');
  });

  it('레벨 정본 10개 — 상자 80개 이하, 게이트·결승·시작점이 있고 옆걸음 게이트엔 허용각', () => {
    expect(LEVELS).toHaveLength(10);
    for (const l of LEVELS) {
      expect(l.boxes.length).toBeLessThanOrEqual(80);
      expect(l.gates.length).toBeGreaterThan(0);
      for (const g of l.gates) if (g.facing != null) expect(g.tol).toBeGreaterThan(0);
      for (const b of l.boxes) expect([b.pitch, b.roll, b.yaw].filter((v) => v).length).toBeLessThanOrEqual(1);
    }
  });

  it('지면 높이 — 평판·경사판(피치·롤)·yaw 상자, 허공은 null', () => {
    const lv: Level = { ...L, boxes: [
      { x: 0, y: 0, z: -0.15, hx: 1, hy: 1, hz: 0.15, m: 'c' },
      { x: 5, y: 0, z: 0.5 - 0.1 / Math.cos(10 * Math.PI / 180), hx: 1, hy: 1, hz: 0.1, m: 'c', pitch: -10 },
      { x: 0, y: 5, z: 0.4, hx: 1, hy: 1, hz: 0.1, m: 'c', roll: 8 },
      { x: 10, y: 0, z: 0.25, hx: 1, hy: 0.2, hz: 0.25, m: 'c', yaw: 90 },
    ] };
    expect(groundAt(lv, 0.5, 0.5)).toBeCloseTo(0);
    expect(groundAt(lv, 5, 0)).toBeCloseTo(0.5, 2);
    expect(groundAt(lv, 5.5, 0)!).toBeGreaterThan(groundAt(lv, 4.5, 0)!);
    expect(groundAt(lv, 0, 5.5)!).toBeGreaterThan(groundAt(lv, 0, 4.5)!);
    expect(groundAt(lv, 10, 0.8)).toBeCloseTo(0.5);
    expect(groundAt(lv, 20, 20)).toBeNull();
    const st: Level = { ...L, gates: [{ x: 0, y: 0, z: 0 }], boxes: [
      { x: 0, y: 0, z: 0, hx: 0.16, hy: 1, hz: 0.3, m: 's' }, { x: 0.32, y: 0, z: 0.07, hx: 0.16, hy: 1, hz: 0.37, m: 's' }] };
    expect(footprintGround(st, 0, 0, 0)).toBeCloseTo(0.44);
    expect(respawnPoint(st, 1).z).toBeCloseTo(0.44);
  });

  it('보행 중 틈에 걸쳐 몸이 낮게 3초 넘게 끼면 마지막 게이트로 복귀 — 앉기(보행 아님)·평지는 제외', () => {
    const G: Level = { ...L, boxes: [{ x: 1, y: 0, z: -0.15, hx: 2.9, hy: 1, hz: 0.15, m: 'c' }, { x: 7.1, y: 0, z: -0.15, hx: 3, hy: 1, hz: 0.15, m: 'c' }] };
    let p: Progress = { passed: 1, done: false };
    const low = at(4, 0, 0.25);
    p = stepCourse(G, p, low, 1000, true).prog;
    expect(stepCourse(G, p, low, 1000 + STUCK_MS - 10, true).ev).toBeNull();
    const r = stepCourse(G, p, low, 1000 + STUCK_MS + 10, true);
    expect(r.ev?.t).toBe('fall');
    expect(r.ev && r.ev.t === 'fall' && r.ev.stuck).toBe(true);
    let q: Progress = { passed: 1, done: false };
    q = stepCourse(G, q, low, 1000, false).prog;
    expect(stepCourse(G, q, low, 9000, false).ev).toBeNull();
    let w: Progress = stepCourse(G, { passed: 1, done: false }, low, 1, true).prog;
    w = stepCourse(G, w, at(4, 0, 0.5), 1500, true).prog;
    expect(stepCourse(G, w, low, 2000, true).ev).toBeNull();
    let f: Progress = { passed: 1, done: false };
    for (let t = 1; t < STUCK_MS * 2; t += 500) { const r2 = stepCourse(G, f, at(1, 0, 0.25), t, true); f = r2.prog; expect(r2.ev).toBeNull(); }
  });

  it('계단을 내려가는 중(몸 아래 지면 기준 정상 높이)은 끼임이 아니다 · 낮아도 나아가면 아니다', () => {
    const st: Level = { ...L, gates: [{ x: 5, y: 0, z: 0 }], boxes: [
      { x: 0, y: 0, z: 0.2, hx: 0.16, hy: 1, hz: 0.5, m: 's' }, { x: 0.32, y: 0, z: 0.13, hx: 0.16, hy: 1, hz: 0.43, m: 's' }] };
    let p: Progress = { passed: 0, done: false };
    for (let t = 0; t <= STUCK_MS + 500; t += 250) {
      const r = stepCourse(st, p, at(0.32, 0, 1.01), 1 + t, true); p = r.prog;
      expect(r.ev).toBeNull();
    }
    const G2: Level = { ...L, boxes: [{ x: 1, y: 0, z: -0.15, hx: 2.9, hy: 1, hz: 0.15, m: 'c' }, { x: 7.1, y: 0, z: -0.15, hx: 3, hy: 1, hz: 0.15, m: 'c' }] };
    let q: Progress = { passed: 1, done: false };
    for (let t = 0, x = 3.7; t <= STUCK_MS * 2; t += 500, x += 0.4) {
      const r = stepCourse(G2, q, at(x, 0, 0.25), 1 + t, true); q = r.prog;
      expect(r.ev?.t === 'fall').toBe(false);
    }
  });

  it('기록 표기', () => {
    expect(fmtTime(83.44)).toBe('1:23.4');
    expect(fmtTime(5)).toBe('0:05.0');
  });
});
