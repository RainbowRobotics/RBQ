import { describe, it, expect, vi } from 'vitest';

vi.mock('@/lib/rest', () => ({
  actions: {},
  WEBRTC_RES: [{ width: 1280, height: 720 }, { width: 960, height: 540 }, { width: 640, height: 360 }],
}));
vi.mock('@/store/telemetry', () => ({ useTelemetry: { subscribe: () => () => {} } }));
vi.mock('@/store/visionToggles', () => ({ useVisionToggles: { getState: () => ({}), setState: () => {} } }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ ip: '' }) } }));
vi.mock('@/lib/commandBus', () => ({ onStreamerCommandReady: () => () => {} }));
import { projectorOnFrom, resIndexFrom } from './robotSettingsSync';

const s = (projector: boolean, projectorOn: boolean) => ({ projector, projectorOn });

describe('로봇 실제값 해석', () => {
  it('IR 프로젝터 — 프로젝터 달린 센서가 없으면 모름, 하나라도 켜져 있으면 켜짐', () => {
    expect(projectorOnFrom(undefined)).toBeUndefined();
    expect(projectorOnFrom([s(false, false)])).toBeUndefined();
    expect(projectorOnFrom([s(true, false), s(false, true)])).toBe(false);
    expect(projectorOnFrom([s(true, false), s(true, true)])).toBe(true);
  });

  it('해상도 — 사다리에 있는 값만 인덱스로, 모르는 값은 표시를 바꾸지 않는다', () => {
    expect(resIndexFrom({ width: 960, height: 540 })).toBe(1);
    expect(resIndexFrom({ width: 1920, height: 1080 })).toBeUndefined();
    expect(resIndexFrom(undefined)).toBeUndefined();
  });
});
