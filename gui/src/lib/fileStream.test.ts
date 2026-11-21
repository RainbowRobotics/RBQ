import { describe, it, expect, vi, beforeEach } from 'vitest';

const deleted: string[] = [];
vi.mock('expo-file-system', () => ({
  Paths: { cache: { uri: 'file:///data/user/0/com.rainbowrobotics.rbq/cache/' } },
  Directory: class {
    uri: string;
    constructor(parent: { uri: string }, name: string) { this.uri = `${parent.uri}${name}/`; }
    delete() {
      if (deleted.includes('NO_DIR')) throw new Error('not found');
      deleted.push(this.uri);
    }
  },
  File: class {
    constructor(public uri: string) {}
    delete() {
      if (this.uri.includes('gone')) throw new Error('not found');
      deleted.push(this.uri);
    }
  },
}));
vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
import { discardPicked, purgePicked } from './fileStream';

const CACHE = 'file:///data/user/0/com.rainbowrobotics.rbq/cache/';

describe('고른 파일 복사본 지우기 — 네이티브', () => {
  beforeEach(() => { deleted.length = 0; });

  it('선택기가 앱 캐시에 만든 DocumentPicker 복사본만 지운다 — 받아 둔 패키지·USB 원본·프록시 경로는 그대로', () => {
    discardPicked(`${CACHE}DocumentPicker/0b6f2c1e.tar`);
    discardPicked(`${CACHE}core-v1.2609.0-1114-u22-x86_64.rbq`);
    discardPicked('content://com.android.externalstorage.documents/document/primary%3ADownload%2Frbq-usb-v1.2609.0-1114.tar');
    discardPicked('/fw-file?name=core.rbq');
    discardPicked('file:///var/mobile/Containers/Data/Application/X/Library/Caches/DocumentPicker/A1.rbq');
    expect(deleted).toEqual([`${CACHE}DocumentPicker/0b6f2c1e.tar`, 'file:///var/mobile/Containers/Data/Application/X/Library/Caches/DocumentPicker/A1.rbq']);
  });

  it('이미 없는 복사본이면 조용히 넘어간다', () => {
    expect(() => discardPicked(`${CACHE}DocumentPicker/gone.tar`)).not.toThrow();
  });
});

describe('앱을 켤 때 선택기 복사본 비우기 — 네이티브', () => {
  beforeEach(() => { deleted.length = 0; });

  it('앱 캐시의 DocumentPicker 폴더를 통째로 지운다', () => {
    purgePicked();
    expect(deleted).toEqual([`${CACHE}DocumentPicker/`]);
  });

  it('폴더가 없으면 조용히 넘어간다', () => {
    deleted.push('NO_DIR');
    expect(() => purgePicked()).not.toThrow();
  });
});
