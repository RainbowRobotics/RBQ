import { describe, it, expect } from 'vitest';
import { decideEntry } from './entryGate';

describe('decideEntry', () => {
  it('데모는 게이트를 건너뛰고 컨트롤', () => expect(decideEntry({ hasRobots: false, hasAccount: false, isDemo: true })).toBe('/'));
  it('로봇도 계정도 없으면(첫 실행) PIN — 처음 한 번', () => expect(decideEntry({ hasRobots: false, hasAccount: false, isDemo: false })).toBe('/pin'));
  it('플랫폼 서버 없는 빌드는 첫 실행도 허브 — PIN 을 넣을 곳이 없다', () => expect(decideEntry({ hasRobots: false, hasAccount: false, isDemo: false, cloud: false })).toBe('/hub'));
  it('저장된 로봇이 있으면 허브', () => expect(decideEntry({ hasRobots: true, hasAccount: false, isDemo: false })).toBe('/hub'));
  it('계정(원격 목록)만 있어도 허브 — 코드는 저장돼 있다', () => expect(decideEntry({ hasRobots: false, hasAccount: true, isDemo: false })).toBe('/hub'));
});
