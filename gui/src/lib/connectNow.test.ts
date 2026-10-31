import { describe, it, expect, vi, beforeEach } from 'vitest';

const connectAll = vi.fn();
vi.mock('./transports', () => ({ connectAll: (...a: unknown[]) => connectAll(...a), disconnectAll: () => {} }));
vi.mock('./demoFlag', () => ({ isDemo: () => false }));
const settings = { connProfile: 'lan', lanIp: '192.168.0.10', wanIp: '', webrtcToken: '',
  setConnProfile: () => {}, setLanIp: () => {}, setWanIp: () => {}, setWebrtcToken: () => {} };
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => settings } }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ ip: '192.168.0.10', visionIp: '', setConnError: () => {}, setVisionIp: () => {}, setIp: () => {} }) } }));

import { connectNow, disconnectAll, reconnectCurrent } from './connectNow';
import { directTarget } from './connectTarget';
import { remote, type RemoteRobots } from '@/modules/registry';

describe('connectNow — 일부러 끊은 뒤 다시 붙기', () => {
  beforeEach(() => { connectAll.mockClear(); });

  it('disconnectAll 뒤 reconnectCurrent 는 설정에 남은 이전 로봇으로 되살리지 않는다', () => {
    const t = directTarget('192.168.0.10'); if (!t.ok) throw new Error('target');
    connectNow(t.target);
    expect(connectAll).toHaveBeenCalledTimes(1);
    disconnectAll();
    expect(reconnectCurrent()).toBe(false);
    expect(connectAll).toHaveBeenCalledTimes(1);
  });

  it('다시 connectNow 하면 reconnectCurrent 가 그 대상으로 붙는다', () => {
    const t = directTarget('192.168.0.10'); if (!t.ok) throw new Error('target');
    connectNow(t.target);
    expect(reconnectCurrent()).toBe(true);
    expect(connectAll).toHaveBeenCalledTimes(2);
  });
});

describe('connectNow — 원격 모듈 값', () => {
  it('★직결 대상에도 원격 모듈 apply 를 부른다 — 남은 원격 값이 다음 연결에 주워지지 않게 모듈이 지운다', () => {
    const apply = vi.fn();
    remote.robots = { apply } as unknown as RemoteRobots;
    try {
      const t = directTarget('192.168.0.10'); if (!t.ok) throw new Error('target');
      connectNow(t.target);
      expect(apply).toHaveBeenCalledWith(t.target);
    } finally { remote.robots = undefined; }
  });
});
