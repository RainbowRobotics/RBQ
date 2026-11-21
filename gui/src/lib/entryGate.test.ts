import { describe, it, expect } from 'vitest';
import { decideEntry } from './entryGate';

describe('decideEntry', () => {
  it('데모는 게이트를 건너뛰고 컨트롤', () => expect(decideEntry({ isDemo: true, route: '/x' })).toBe('/'));
  it('기본은 허브 — 원격 모듈이 없는 빌드(공개)도', () => expect(decideEntry({ isDemo: false })).toBe('/hub'));
  it('모듈이 정한 첫 화면으로', () => expect(decideEntry({ isDemo: false, route: '/x' })).toBe('/x'));
});
