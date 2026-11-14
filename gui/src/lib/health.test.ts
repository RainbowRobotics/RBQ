import { describe, it, expect, vi } from 'vitest';

vi.mock('@/lib/demoFlag', () => ({ isDemo: () => false, getDemoBaseRobot: () => null, getDemoBaseLevel: () => null }));
vi.mock('@react-native-async-storage/async-storage', () => ({
  default: { getItem: vi.fn(async () => null), setItem: vi.fn(async () => {}), removeItem: vi.fn(async () => {}) },
}));
import { healthText, healthTarget, isBootFinding, healthRows } from './health';
import { useRobot } from '@/store/robot';

describe('healthText — id + 파라미터로 문구를 만든다', () => {
  it('온도는 값·기준·°C 를 채운다', () => {
    expect(healthText({ id: 'motor.over_temp', severity: 'warn', component: 'HRR0', value: 81, threshold: 80, unit: 'C', since: 0 }))
      .toBe('HRR0 모터 온도 81°C (기준 80°C)');
  });
  it('에러 비트 원문은 그대로', () => {
    expect(healthText({ id: 'motor.error', severity: 'fault', component: 'HLK5', detail: 'JAM,CUR', since: 0 }))
      .toBe('HLK5 모터 에러 (JAM,CUR)');
  });
  it('모르는 id 는 id 를 보인다 — 로봇이 앞서 나가도 빈 줄이 되지 않게', () => {
    expect(healthText({ id: 'new.thing', severity: 'warn', component: 'X', since: 0 })).toBe('new.thing (X)');
  });
});

describe('healthTarget', () => {
  it('전원·관절·비전으로 간다, 비전은 개발자 모드에서만', () => {
    expect(healthTarget({ id: 'battery.low', severity: 'warn', since: 0 }, false)).toBe('/maintenance?sec=pwr');
    expect(healthTarget({ id: 'motor.disconnected', severity: 'fault', since: 0 }, false)).toBe('/maintenance?sec=chk');
    expect(healthTarget({ id: 'vision.sensor_failed', severity: 'warn', since: 0 }, false)).toBeUndefined();
    expect(healthTarget({ id: 'vision.sensor_failed', severity: 'warn', since: 0 }, true)).toBe('/maintenance?sec=vision');
    expect(healthTarget({ id: 'robot.fallen', severity: 'fault', since: 0 }, true)).toBeUndefined();
  });
  it('기동 단계는 팝오버 행이 따로 보인다', () => {
    expect(isBootFinding({ id: 'boot.imu', severity: 'warn', since: 0 })).toBe(true);
  });
});

describe('신호등 — 로봇 판정이 있으면 그걸 쓴다', () => {
  const robot = { imu: true, can_bus: true, find_pose: true, control_started: true } as any;
  it('fault → red, 판정이 없으면 종전 규칙', () => {
    useRobot.getState().applyRobot(robot);
    expect(useRobot.getState().agg).toBe('green');
    useRobot.getState().applyHealth({ overall: 'fault', findings: [] });
    expect(useRobot.getState().agg).toBe('red');
    useRobot.getState().applyRobot(robot);
    expect(useRobot.getState().agg).toBe('red');
    useRobot.getState().applyHealth(undefined);
    expect(useRobot.getState().agg).toBe('green');
  });
});

describe('healthRows — 레벨 1 은 큰 묶음, 2 이상은 항목 그대로', () => {
  const F = [
    { id: 'motor.over_temp', severity: 'warn', component: 'HLK5', value: 84, threshold: 80, unit: 'C', since: 0 },
    { id: 'battery.low', severity: 'warn', value: 4, threshold: 5, unit: '%', since: 0 },
    { id: 'motor.disconnected', severity: 'fault', component: 'HRK2', since: 0 },
    { id: 'vision.sensor_failed', severity: 'warn', component: 'RR0', code: 3, since: 0 },
    { id: 'boot.control', severity: 'warn', since: 0 },
  ] as const;
  it('레벨 1 — 묶음당 한 줄, 고장 먼저, 묶음 색은 가장 심한 것', () => {
    const r = healthRows([...F], 1);
    expect(r.map((x) => [x.text, x.severity])).toEqual([['다리 이상', 'fault'], ['배터리 부족', 'warn'], ['카메라 이상', 'warn']]);
    expect(r[0].to).toBe('/maintenance?sec=chk');
    expect(r[2].to).toBeUndefined();
  });
  it('레벨 2 — 항목마다 부품·값', () => {
    const r = healthRows([...F], 2);
    expect(r).toHaveLength(4);
    expect(r[0].text).toBe('HRK2 모터 연결 끊김');
    expect(r.find((x) => x.text.startsWith('카메라'))?.to).toBe('/maintenance?sec=vision');
  });
  it('배터리 보호는 "배터리 이상", 비상정지는 그대로', () => {
    const r = healthRows([{ id: 'battery.protection', severity: 'fault', component: 'left', detail: 'UV', since: 0 },
      { id: 'robot.fallen', severity: 'fault', since: 0 }], 1);
    expect(r.map((x) => x.text)).toEqual(['배터리 이상', '비상정지 상태']);
  });
});
