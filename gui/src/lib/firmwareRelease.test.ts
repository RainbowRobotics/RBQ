import { beforeEach, describe, expect, it, vi } from 'vitest';

const fs = vi.hoisted(() => ({
  files: new Map<string, number>(),
  plan: {} as Record<string, { status: number; size: number; expected: number }>,
  moveGate: null as Promise<void> | null,
  moving: 0,
}));

vi.mock('react-native', () => ({ Platform: { OS: 'android' } }));
vi.mock('expo-constants', () => ({ default: { expoConfig: { extra: {} } } }));
vi.mock('@/lib/desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('expo-file-system/legacy', () => ({
  cacheDirectory: 'cache/',
  createDownloadResumable: vi.fn((url: string, dest: string, _o: unknown, cb: (p: unknown) => void) => ({
    downloadAsync: async () => {
      const r = fs.plan[url.split('/').pop()!];
      fs.files.set(dest, r.size);
      cb({ totalBytesWritten: r.size, totalBytesExpectedToWrite: r.expected });
      return { status: r.status };
    },
    cancelAsync: async () => {},
  })),
  deleteAsync: async (uri: string) => { fs.files.delete(uri); },
  getInfoAsync: async (uri: string) => ({ exists: fs.files.has(uri), size: fs.files.get(uri) ?? 0 }),
  moveAsync: async ({ from, to }: { from: string; to: string }) => {
    fs.moving++;
    if (fs.moveGate) await fs.moveGate;
    fs.files.set(to, fs.files.get(from)!); fs.files.delete(from);
  },
  writeAsStringAsync: async (uri: string) => { fs.files.set(uri, 1); },
  readDirectoryAsync: async () => [],
  readAsStringAsync: async () => '',
}));

import { createDownloadResumable } from 'expo-file-system/legacy';
import { downloadFirmware, type FwRelease } from './firmwareRelease';

const rel: FwRelease = { tag: 'v1.2610.0', version: '1.2610.0', publishedAt: '', latest: true, channel: 'release' };
const ZIP = 'RBQ-v1.2610.0.zip';
const TAR = 'RBQ-v1.2610.0.tar.gz';
const parts = () => [...fs.files.keys()].filter((k) => k.endsWith('.part'));

beforeEach(() => {
  fs.files.clear();
  fs.moveGate = null;
  fs.moving = 0;
  fs.plan = {
    [ZIP]: { status: 200, size: 100, expected: 100 },
    [TAR]: { status: 200, size: 80, expected: 80 },
  };
  vi.mocked(createDownloadResumable).mockClear();
});

describe('downloadFirmware (네이티브)', () => {
  it('다 받으면 제 이름으로 남고 .part 는 없다', async () => {
    const f = await downloadFirmware(rel, () => {}).done;
    expect(f.name).toBe(ZIP);
    expect(fs.files.get(`cache/${ZIP}`)).toBe(100);
    expect(fs.files.get(`cache/${TAR}`)).toBe(80);
    expect(parts()).toEqual([]);
  });

  it('알린 크기보다 덜 받고 오류 없이 끝나면 잘린 것으로 버린다', async () => {
    fs.plan[ZIP] = { status: 200, size: 60, expected: 100 };
    await expect(downloadFirmware(rel, () => {}).done).rejects.toThrow('끊겼습니다');
    expect(fs.files.has(`cache/${ZIP}`)).toBe(false);
    expect(parts()).toEqual([]);
  });

  it('서버가 크기를 알리지 않으면(0) 크기 대조 없이 받는다', async () => {
    fs.plan[ZIP] = { status: 200, size: 60, expected: 0 };
    await downloadFirmware(rel, () => {}).done;
    expect(fs.files.get(`cache/${ZIP}`)).toBe(60);
  });

  it('앞 파일을 옮기는 사이에 취소하면 다음 파일을 받기 시작하지 않는다', async () => {
    let open!: () => void;
    fs.moveGate = new Promise((r) => { open = r; });
    const dl = downloadFirmware(rel, () => {});
    await vi.waitFor(() => expect(fs.moving).toBe(1));
    dl.cancel();
    open();
    await expect(dl.done).rejects.toThrow('cancelled');
    expect(createDownloadResumable).toHaveBeenCalledTimes(1);
    expect(parts()).toEqual([]);
  });
});
