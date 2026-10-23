import { describe, it, expect, vi, beforeEach } from 'vitest';

const mem = new Map<string, string>();
vi.mock('@react-native-async-storage/async-storage', () => ({
  default: {
    getItem: async (k: string) => mem.get(k) ?? null,
    setItem: async (k: string, v: string) => { mem.set(k, v); },
    removeItem: async (k: string) => { mem.delete(k); },
  },
}));

import { useRobotSettings } from './robotSettings';
import { useLedBottom } from './ledBottom';
import type { LedSetting } from '@/lib/ledBottom';

const green: LedSetting = { mode: 'on', rgb: [0, 127, 0], on_ms: 0, off_ms: 0, count: 0 };
const blink: LedSetting = { mode: 'blink', rgb: [0, 255, 0], on_ms: 803, off_ms: 800, count: 0 };

describe('useLedBottom — 마지막으로 알던 값(로봇 시리얼별)', () => {
  beforeEach(() => {
    useRobotSettings.setState({ serial: '', bySerial: {} });
    useLedBottom.setState({ right: null, left: null });
  });

  it('로봇마다 따로 남고, 다른 로봇으로 가면 비어 있다', () => {
    useRobotSettings.getState().setSerial('RBQ-A');
    useLedBottom.getState().remember('right', green, 'sent', 1000);
    expect(useRobotSettings.getState().bySerial['RBQ-A']['ledBottom.right']).toEqual({ s: green, from: 'sent', at: 1000 });
    useRobotSettings.getState().setSerial('RBQ-B');
    expect(useLedBottom.getState().right).toBeNull();
    useRobotSettings.getState().setSerial('RBQ-A');
    expect(useLedBottom.getState().right).toEqual({ s: green, from: 'sent', at: 1000 });
  });

  it('로봇이 알려 준 값이 보낸 값을 덮는다 — 같은 값이 다시 오면 스토어를 건드리지 않는다', () => {
    useRobotSettings.getState().setSerial('RBQ-A');
    useLedBottom.getState().remember('left', green, 'sent', 1000);
    useLedBottom.getState().remember('left', blink, 'robot', 1300);
    const first = useLedBottom.getState().left;
    expect(first).toEqual({ s: { ...blink, on_ms: 800 }, from: 'robot', at: 1300 });
    useLedBottom.getState().remember('left', { ...blink }, 'robot', 2300);
    expect(useLedBottom.getState().left).toBe(first);
  });

  it('보낸 값을 그보다 먼저 IF 가 알려 준 옛 로봇 값으로 되감지 않는다(PUT 직후의 1초 폴링)', () => {
    useRobotSettings.getState().setSerial('RBQ-A');
    useLedBottom.getState().remember('right', blink, 'sent', 5000);
    useLedBottom.getState().remember('right', green, 'robot', 4990);
    expect(useLedBottom.getState().right).toEqual({ s: { ...blink, on_ms: 800 }, from: 'sent', at: 5000 });
    useLedBottom.getState().remember('right', green, 'robot', 5100);
    expect(useLedBottom.getState().right).toEqual({ s: green, from: 'robot', at: 5100 });
  });
});
