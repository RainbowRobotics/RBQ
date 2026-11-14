import { describe, it, expect, beforeEach, vi } from 'vitest';

const DIR = 'file:///cache/';
const cache = new Map<string, number>();
const deleted: string[] = [];
vi.mock('expo-file-system/legacy', () => ({
  cacheDirectory: 'file:///cache/',
  readDirectoryAsync: async () => [...cache.keys()],
  getInfoAsync: async (uri: string) => {
    const name = uri.slice(DIR.length);
    return cache.has(name) ? { exists: true, isDirectory: false, uri, modificationTime: cache.get(name) } : { exists: false };
  },
  deleteAsync: async (uri: string) => { deleted.push(uri); cache.delete(uri.slice(DIR.length)); },
  downloadAsync: async (_url: string, uri: string) => { cache.set(uri.slice(DIR.length), 100); return { uri, status: 200, headers: {} }; },
  writeAsStringAsync: async () => {},
}));
vi.mock('expo-file-system', () => ({ File: class {}, FileMode: {} }));
vi.mock('expo-sharing', () => ({ isAvailableAsync: async () => true, shareAsync: async () => {} }));
vi.mock('./auth', () => ({ robotAuth: () => ({}) }));
vi.mock('./rest', () => ({ restBase: () => 'http://r', MEDIA_ROOT: { clip: '/api/media/clip' }, viaRemote: () => false, RemoteUnsupportedError: class {} }));
vi.mock('./robotTunnel', () => ({ tunnelDownload: async () => {}, tunnelReady: () => false }));
vi.mock('./storage', () => ({ ensureFreeSpace: async () => {}, freeBytes: async () => 0 }));
vi.mock('./bbLibrary', () => ({ libPut: async () => {} }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ ip: '' }) } }));

import { saveMediaFile } from './download';
import { useMediaDl } from '@/store/mediaDownload';
import { useLogSave } from '@/store/logSave';

describe('지난 저장분 정리 — 공유를 기다리는 파일은 지우지 않는다', () => {
  beforeEach(() => {
    cache.clear(); deleted.length = 0;
    useMediaDl.setState({ share: [] });
    useLogSave.setState({ share: null });
  });

  it('화면 밖에서 차례로 받은 미디어·로그는 다음 받기에 지워지지 않고, 기다리지 않는 옛 것만 지운다', async () => {
    cache.set('media-old.jpg', 1);
    cache.set('blackbox-RBQ10_x.zip', 2);
    cache.set('media-front_1.mp4', 3);
    useLogSave.setState({ share: `${DIR}blackbox-RBQ10_x.zip` });
    useMediaDl.setState({ share: [`${DIR}media-front_1.mp4`] });
    await saveMediaFile('192.168.0.10', 'clip', 'rear_1.mp4', 0, () => false);
    expect(deleted).not.toContain(`${DIR}blackbox-RBQ10_x.zip`);
    expect(deleted).not.toContain(`${DIR}media-front_1.mp4`);
    expect(cache.has('media-rear_1.mp4')).toBe(true);
  });

  it('기다리는 것이 없으면 종전처럼 가장 최근 하나만 남긴다', async () => {
    cache.set('media-a.jpg', 1);
    cache.set('media-b.jpg', 2);
    await saveMediaFile('192.168.0.10', 'clip', 'c.mp4', 0);
    expect(deleted).toEqual([`${DIR}media-a.jpg`]);
  });
});
