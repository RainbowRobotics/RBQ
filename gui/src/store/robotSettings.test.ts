import { describe, it, expect, vi, beforeEach } from 'vitest';

const mem = new Map<string, string>();
vi.mock('@react-native-async-storage/async-storage', () => ({
  default: {
    getItem: async (k: string) => mem.get(k) ?? null,
    setItem: async (k: string, v: string) => { mem.set(k, v); },
    removeItem: async (k: string) => { mem.delete(k); },
  },
}));

import { create } from 'zustand';
import { useRobotSettings, bindRobotCache } from './robotSettings';

type T = { night: boolean; zoom: number; err: string };

describe('bindRobotCache — 로봇별 설정 캐시', () => {
  beforeEach(() => { useRobotSettings.setState({ serial: '', bySerial: {} }); });

  it('시리얼을 모르면 캐시에 쓰지 않고, 알게 되면 로봇마다 따로 저장·복원한다', () => {
    const store = create<T>(() => ({ night: false, zoom: 1, err: '' }));
    const off = bindRobotCache(store, ['night', 'zoom'], 'vt');

    store.setState({ night: true });
    expect(useRobotSettings.getState().bySerial).toEqual({});

    useRobotSettings.getState().setSerial('RBQ-A');
    expect(store.getState().night).toBe(true);
    expect(useRobotSettings.getState().bySerial['RBQ-A']).toEqual({ 'vt.night': true });
    store.setState({ zoom: 4 });
    expect(useRobotSettings.getState().bySerial['RBQ-A']).toEqual({ 'vt.night': true, 'vt.zoom': 4 });

    useRobotSettings.getState().setSerial('RBQ-B');
    expect(store.getState()).toMatchObject({ night: false, zoom: 1 });
    store.setState({ zoom: 2 });

    useRobotSettings.getState().setSerial('RBQ-A');
    expect(store.getState()).toMatchObject({ night: true, zoom: 4 });
    expect(useRobotSettings.getState().bySerial['RBQ-B']).toEqual({ 'vt.zoom': 2 });
    off();
  });

  it('시리얼 전에 바뀐 필드만 우선한다 — 나머지는 그 로봇 캐시값으로 채운다', () => {
    const store = create<T>(() => ({ night: false, zoom: 1, err: '' }));
    const off = bindRobotCache(store, ['night', 'zoom'], 'vt');
    useRobotSettings.setState({ bySerial: { 'RBQ-D': { 'vt.night': true, 'vt.zoom': 3 } } });
    store.setState({ zoom: 2 });
    useRobotSettings.getState().setSerial('RBQ-D');
    expect(store.getState()).toMatchObject({ night: true, zoom: 2 });
    expect(useRobotSettings.getState().bySerial['RBQ-D']).toEqual({ 'vt.night': true, 'vt.zoom': 2 });
    off();
  });

  it('묶지 않은 필드는 캐시하지 않는다', () => {
    const store = create<T>(() => ({ night: false, zoom: 1, err: '' }));
    const off = bindRobotCache(store, ['night'], 'vt');
    useRobotSettings.getState().setSerial('RBQ-C');
    store.setState({ err: 'boom', night: true });
    expect(useRobotSettings.getState().bySerial['RBQ-C']).toEqual({ 'vt.night': true });
    off();
  });
});
