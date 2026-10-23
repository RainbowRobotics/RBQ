import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
import { useTelemetry, noteMotionLink, noteVisionLink, STALE_CLEAR_MS } from './telemetry';

const robot = { foo: 1 } as never;
const sensors = [{ bar: 2 }] as never;

describe('telemetry 값 비우기', () => {
  beforeEach(() => useTelemetry.getState().reset());

  it('모션 값만 비우고 비전 값은 남긴다', () => {
    useTelemetry.setState({ robot, sensors, lastRobotAt: 1, lastSensorAt: 2 });
    useTelemetry.getState().clearMotionData();
    const s = useTelemetry.getState();
    expect(s.robot).toBeUndefined();
    expect(s.lastRobotAt).toBeUndefined();
    expect(s.sensors).toBe(sensors);
  });

  it('비전 값만 비우고 모션 값은 남긴다', () => {
    useTelemetry.setState({ robot, sensors, lastRobotAt: 1, lastSensorAt: 2 });
    useTelemetry.getState().clearVisionData();
    const s = useTelemetry.getState();
    expect(s.sensors).toBeUndefined();
    expect(s.lastSensorAt).toBeUndefined();
    expect(s.robot).toBe(robot);
  });

  it('연결 상태는 건드리지 않는다 — 재시도 중이면 "연결 중" 표시가 유지돼야 한다', () => {
    useTelemetry.setState({ motionConn: 'connecting', visionConn: 'connecting', robot, sensors });
    useTelemetry.getState().clearMotionData();
    useTelemetry.getState().clearVisionData();
    const s = useTelemetry.getState();
    expect(s.motionConn).toBe('connecting');
    expect(s.visionConn).toBe('connecting');
  });

  it('reset 은 연결 상태까지 끊김으로 되돌린다(사용자가 끊은 경우)', () => {
    useTelemetry.setState({ motionConn: 'connected', robot });
    useTelemetry.getState().reset();
    const s = useTelemetry.getState();
    expect(s.motionConn).toBe('disconnected');
    expect(s.robot).toBeUndefined();
  });
});

describe('링크 보고 + 유예', () => {
  beforeEach(() => { vi.useFakeTimers(); useTelemetry.getState().reset(); });
  afterEach(() => { noteMotionLink('connected'); noteVisionLink('connected'); vi.useRealTimers(); });

  it('끊긴 채 유예가 지나면 모션 값만 비운다', () => {
    noteMotionLink('connected');
    useTelemetry.setState({ robot, sensors });
    noteMotionLink('connecting');
    vi.advanceTimersByTime(STALE_CLEAR_MS - 1);
    expect(useTelemetry.getState().robot).toBe(robot);
    vi.advanceTimersByTime(1);
    const s = useTelemetry.getState();
    expect(s.robot).toBeUndefined();
    expect(s.sensors).toBe(sensors);
    expect(s.motionConn).toBe('connecting');
  });

  it('유예 안에 다시 붙으면 값이 남는다 — 짧은 끊김에 깜빡이지 않는다', () => {
    useTelemetry.setState({ robot });
    noteMotionLink('disconnected');
    vi.advanceTimersByTime(STALE_CLEAR_MS / 2);
    noteMotionLink('connected');
    vi.advanceTimersByTime(STALE_CLEAR_MS);
    expect(useTelemetry.getState().robot).toBe(robot);
    expect(useTelemetry.getState().motionConn).toBe('connected');
  });

  it('비전 링크는 vision-state DC 기준으로 같은 규약', () => {
    useTelemetry.setState({ sensors, robot });
    noteVisionLink('disconnected');
    vi.advanceTimersByTime(STALE_CLEAR_MS);
    const s = useTelemetry.getState();
    expect(s.sensors).toBeUndefined();
    expect(s.robot).toBe(robot);
    expect(s.visionConn).toBe('disconnected');
  });
});
