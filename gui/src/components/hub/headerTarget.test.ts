import { describe, it, expect, afterEach } from 'vitest';

import { headerTarget } from './headerTarget';
import { remote, type RemoteRobots } from '@/modules/registry';
import type { RobotProfile } from '@/store/robots';

type P = RobotProfile & { x?: string };
const fake = { lastVia: 'via-x', has: (p: P) => !!p.x, idOf: (p: P) => p.x } as unknown as RemoteRobots;
const lan: P = { serial: 'RBQ1', name: 'RBQ1', lastVia: 'lan', lastSeenAt: 0 };

describe('headerTarget — 허브 헤더의 경로·이름', () => {
  afterEach(() => { remote.robots = undefined; });
  it('직결이면 목록 기록 그대로', () => {
    remote.robots = fake;
    expect(headerTarget('', lan, 'lan')).toEqual({ viaEff: 'lan', otherId: null });
  });
  it('같은 로봇에 PIN(원격)으로 붙으면 "원격", 이름은 목록 것', () => {
    remote.robots = fake;
    expect(headerTarget('RBQ1', lan, 'lan')).toEqual({ viaEff: 'via-x', otherId: null });
  });
  it('다른 로봇에 PIN 으로 붙으면 그 로봇 id 를 이름으로', () => {
    remote.robots = fake;
    expect(headerTarget('RBQ9', lan, 'lan')).toEqual({ viaEff: 'via-x', otherId: 'RBQ9' });
  });
  it('계정 원격 로봇(원격 id 가 시리얼과 다름)은 목록 이름', () => {
    remote.robots = fake;
    expect(headerTarget('rv-7', { serial: 'S7', name: 'S7', lastVia: 'via-x', lastSeenAt: 0, x: 'rv-7' } as P, undefined))
      .toEqual({ viaEff: 'via-x', otherId: null });
  });
  it('원격 모듈이 없으면(공개 빌드) 목록 기록 그대로', () => {
    expect(headerTarget('', lan, undefined)).toEqual({ viaEff: 'lan', otherId: null });
  });
});
