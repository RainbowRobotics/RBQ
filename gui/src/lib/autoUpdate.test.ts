import { describe, expect, it, vi } from 'vitest';

vi.mock('react-native', () => ({ Platform: { OS: 'android' } }));
vi.mock('@/lib/desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('@/lib/rest', () => ({ rest: {} }));
vi.mock('@/lib/firmwareRelease', () => ({}));
vi.mock('@/lib/appSelfUpdate', () => ({}));

import { pickOffers, type OfferInput } from './autoUpdate';

const top = { tag: 'v1.2610.0', version: '1.2610.0', publishedAt: '', latest: true, channel: 'release' as const };
const appRel = { tag: 'v1.2610.0', version: '1.2610.0', channel: 'release' as const };
const base: OfferInput = {
  top, robotSupported: true, robotPkgOnDevice: false, robotVersion: null,
  appRel, appMode: 'apk', skip: {},
};
const kinds = (i: Partial<OfferInput>) => pickOffers({ ...base, ...i }).map((o) => o.kind);

describe('pickOffers', () => {
  it('새 로봇 소프트웨어와 새 앱을 둘 다 묻는다(로봇 먼저)', () => {
    expect(kinds({})).toEqual(['robot', 'app']);
  });

  it('최신 로봇 패키지가 이미 기기에 있으면 로봇은 묻지 않는다', () => {
    expect(kinds({ robotPkgOnDevice: true })).toEqual(['app']);
  });

  it('붙어 있는 로봇이 이미 최신이면 로봇은 묻지 않는다', () => {
    expect(kinds({ robotVersion: '1.2610.0' })).toEqual(['app']);
    expect(kinds({ robotVersion: '1.2609.0' })).toEqual(['robot', 'app']);
  });

  it('패키지를 둘 곳이 없는 실행(브라우저)은 로봇을 묻지 않는다', () => {
    expect(kinds({ robotSupported: false })).toEqual(['app']);
  });

  it('직접 설치한 앱(apk·appimage·exe)만 앱 업데이트를 묻는다', () => {
    expect(kinds({ appMode: 'appimage' })).toEqual(['robot', 'app']);
    expect(kinds({ appMode: 'exe' })).toEqual(['robot', 'app']);
    expect(kinds({ appMode: 'store' })).toEqual(['robot']);
    expect(kinds({ appMode: 'none' })).toEqual(['robot']);
  });

  it('앱이 이미 최신이면(appRel 없음) 앱은 묻지 않는다', () => {
    expect(kinds({ appRel: null })).toEqual(['robot']);
  });

  it('거절한 버전은 다시 묻지 않지만, 더 새 버전은 묻는다', () => {
    expect(kinds({ skip: { robot: '1.2610.0', app: '1.2610.0' } })).toEqual([]);
    expect(kinds({ skip: { robot: '1.2609.0', app: '1.2609.0' } })).toEqual(['robot', 'app']);
  });

  it('채널 목록이 비면(발행 없음) 로봇은 묻지 않는다', () => {
    expect(kinds({ top: null, appRel: null })).toEqual([]);
  });
});
