import { beforeEach, describe, expect, it, vi } from 'vitest';

const fs = vi.hoisted(() => ({
  files: new Map<string, number>(),
  text: new Map<string, string>(),
  plan: {} as Record<string, { status: number; size: number; expected: number; throws?: string }>,
  moveGate: null as Promise<void> | null,
  moving: 0,
}));

vi.mock('react-native', () => ({ Platform: { OS: 'android' } }));
vi.mock('expo-constants', () => ({ default: { expoConfig: { extra: {} } } }));
vi.mock('@/lib/desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('./fileStream', () => ({ openFileStream: vi.fn() }));
vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
vi.mock('expo-file-system/legacy', () => ({
  cacheDirectory: 'cache/',
  createDownloadResumable: vi.fn((url: string, dest: string, _o: unknown, cb: (p: unknown) => void, resumeData?: string) => ({
    downloadAsync: async () => {
      const r = fs.plan[url.split('/').pop()!] ?? { status: 403, size: 0, expected: 0 };
      const from = resumeData ? Number(resumeData) : 0;
      if (r.throws) { fs.files.set(dest, from + r.size); throw new Error(r.throws); }
      fs.files.set(dest, from + r.size);
      cb({ totalBytesWritten: from + r.size, totalBytesExpectedToWrite: r.expected });
      return { status: r.status };
    },
    cancelAsync: async () => {},
  })),
  deleteAsync: async (uri: string) => { fs.files.delete(uri); fs.text.delete(uri); },
  getInfoAsync: async (uri: string) => ({ exists: fs.files.has(uri), size: fs.files.get(uri) ?? 0 }),
  moveAsync: async ({ from, to }: { from: string; to: string }) => {
    fs.moving++;
    if (fs.moveGate) await fs.moveGate;
    fs.files.set(to, fs.files.get(from)!); fs.files.delete(from);
  },
  writeAsStringAsync: async (uri: string, v: string) => { fs.files.set(uri, 1); fs.text.set(uri, v); },
  readDirectoryAsync: async () => [],
  readAsStringAsync: async (uri: string) => { if (!fs.text.has(uri)) throw new Error('no file'); return fs.text.get(uri)!; },
}));

import { createDownloadResumable } from 'expo-file-system/legacy';
import { cacheId, downloadFirmware, coreSystemOf, parseSidecars, pickPackage, pkgCandidates, LEGACY_SYSTEM, type FwRelease } from './firmwareRelease';
import { otaTransfer, packageMismatch } from './ota';

const rel: FwRelease = { tag: 'v1.2610.0', version: '1.2610.0', publishedAt: '', latest: true, channel: 'release' };
const ZIP = 'RBQ-v1.2610.0.zip';
const TAR = 'RBQ-v1.2610.0.tar.gz';
const CORE24 = 'core-v1.2610.0-u24-x86_64.rbq';
const CORE22 = 'core-v1.2610.0-u22-x86_64.rbq';
const ALL_CORES = ['u22-x86_64', 'u22-aarch64', 'u24-x86_64', 'u24-aarch64'].map((sys) => `core-v1.2610.0-${sys}.rbq`);
const parts = () => [...fs.files.keys()].filter((k) => k.endsWith('.part'));

beforeEach(() => {
  fs.files.clear();
  fs.text.clear();
  fs.moveGate = null;
  fs.moving = 0;
  fs.text.clear();
  fs.plan = {
    [ZIP]: { status: 200, size: 100, expected: 100 },
    [TAR]: { status: 200, size: 80, expected: 80 },
    [CORE24]: { status: 200, size: 50, expected: 50 },
  };
  vi.mocked(createDownloadResumable).mockClear();
  sidecars.clear();
  vi.stubGlobal('fetch', vi.fn(async (url: string) => {
    const body = sidecars.get(url.split('/').pop()!);
    return { ok: body != null, status: body != null ? 200 : 404, text: async () => body ?? '' };
  }));
});
const sidecars = new Map<string, string>();
const SHA = 'ab'.repeat(32);
const SIG = `${'A'.repeat(86)}==`;

describe('downloadFirmware (네이티브)', () => {
  it('다 받으면 제 이름으로 남고 .part 는 없다', async () => {
    const f = await downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done;
    expect(f.name).toBe(ZIP);
    expect(fs.files.get(`cache/${ZIP}`)).toBe(100);
    expect(fs.files.get(`cache/${TAR}`)).toBe(80);
    expect(parts()).toEqual([]);
  });

  it('알린 크기보다 덜 받고 오류 없이 끝나면 제 이름으로 두지 않고, 조각은 남겨 다음에 이어 받는다', async () => {
    fs.plan[ZIP] = { status: 200, size: 60, expected: 100 };
    await expect(downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done).rejects.toThrow('끊겼습니다');
    expect(fs.files.has(`cache/${ZIP}`)).toBe(false);
    expect(parts()).toEqual([`cache/${ZIP}.part`]);
    fs.plan[ZIP] = { status: 206, size: 40, expected: 100 };
    await downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done;
    expect(vi.mocked(createDownloadResumable).mock.calls.at(-2)?.[4]).toBe('60');
    expect(fs.files.get(`cache/${ZIP}`)).toBe(100);
    expect(parts()).toEqual([]);
  });

  it('구형 로봇용 두 개를 받으면 진행에 몇 번째인지 실린다', async () => {
    const parts = new Set<string>();
    await downloadFirmware(rel, (_w, _t, p) => { if (p) parts.add(`${p.i}/${p.n}`); }, LEGACY_SYSTEM).done;
    expect([...parts]).toEqual(['1/2', '2/2']);
  });

  it('망 오류로 끊기면 조각을 남기고 이어 받는다고 알린다', async () => {
    fs.plan[ZIP] = { status: 0, size: 30, expected: 100, throws: 'socket closed' };
    await expect(downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done).rejects.toThrow('이어 받습니다');
    expect(parts()).toEqual([`cache/${ZIP}.part`]);
  });

  it('서버가 Range 를 무시하고(200) 처음부터 주면 붙은 것을 버리고 새로 받는다', async () => {
    fs.plan[ZIP] = { status: 200, size: 60, expected: 100 };
    await expect(downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done).rejects.toThrow();
    fs.plan[ZIP] = { status: 200, size: 100, expected: 100 };
    await downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done;
    expect(fs.files.get(`cache/${ZIP}`)).toBe(100);
  });

  it('다른 빌드의 조각에는 이어 붙이지 않는다', async () => {
    fs.plan[ZIP] = { status: 200, size: 60, expected: 100 };
    await expect(downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done).rejects.toThrow();
    fs.text.set(`cache/${ZIP}.part.src`, 'other-build');
    fs.plan[ZIP] = { status: 200, size: 100, expected: 100 };
    vi.mocked(createDownloadResumable).mockClear();
    await downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done;
    expect(vi.mocked(createDownloadResumable).mock.calls[0][4]).toBeUndefined();
  });

  it('서버가 크기를 알리지 않으면(0) 크기 대조 없이 받는다', async () => {
    fs.plan[ZIP] = { status: 200, size: 60, expected: 0 };
    await downloadFirmware(rel, () => {}, LEGACY_SYSTEM).done;
    expect(fs.files.get(`cache/${ZIP}`)).toBe(60);
  });

  it('앞 파일을 옮기는 사이에 취소하면 다음 파일을 받기 시작하지 않는다', async () => {
    let open!: () => void;
    fs.moveGate = new Promise((r) => { open = r; });
    const dl = downloadFirmware(rel, () => {}, LEGACY_SYSTEM);
    await vi.waitFor(() => expect(fs.moving).toBe(1));
    dl.cancel();
    open();
    await expect(dl.done).rejects.toThrow('cancelled');
    expect(createDownloadResumable).toHaveBeenCalledTimes(1);
    expect(parts()).toEqual([]);
  });
});

describe('시스템별 패키지(core-*.rbq)', () => {
  const nightly: FwRelease = { tag: 'v1.2609.0-nightly.a8d05aea', version: '1.2609.0-nightly.a8d05aea',
    publishedAt: '2026-10-15', latest: true, channel: 'nightly',
    core: ['core-v1.2609.0-1016-u22-x86_64.rbq', 'core-v1.2609.0-1016-u24-x86_64.rbq'] };

  it('같은 날 나온 나이틀리 둘은 받음 표식이 다르다(날짜가 아니라 sha 든 태그)', () => {
    const later = { ...nightly, tag: 'v1.2609.0-nightly.b5635a91' };
    expect(cacheId(later)).not.toBe(cacheId(nightly));
  });

  it('이름 끝에서 시스템을 꺼낸다(나이틀리 날짜 포함)', () => {
    expect(coreSystemOf('core-v1.2609.0-1016-u24-aarch64.rbq')).toBe('u24-aarch64');
    expect(coreSystemOf(CORE24)).toBe('u24-x86_64');
    expect(coreSystemOf('RBQ-nightly.zip')).toBeNull();
  });

  it('후보: 구형 로봇이면 옛 것만, 모르면 시스템마다의 core 전부(그다음 옛 것), 알면 그 시스템의 core 만', () => {
    expect(pkgCandidates(rel, LEGACY_SYSTEM)).toEqual([ZIP, TAR]);
    expect(pkgCandidates(rel, undefined)).toEqual([...ALL_CORES, ZIP, TAR]);
    expect(pkgCandidates(nightly, undefined)).toEqual([...nightly.core!, 'RBQ-nightly.zip', 'RBQ-nightly.tar.gz']);
    expect(pkgCandidates(rel, 'u22-x86_64')).toEqual(['core-v1.2610.0-u22-x86_64.rbq']);
    expect(pkgCandidates(rel, 'u24-x86_64')).toEqual([CORE24]);
    expect(pkgCandidates(nightly, 'u24-x86_64')).toEqual(['core-v1.2609.0-1016-u24-x86_64.rbq']);
    expect(pkgCandidates(nightly, 'u24-aarch64')).toEqual([]);
  });

  it('받아 둔 것 중 24.04 로봇에는 옛 RBQ-* 를 고르지 않는다', () => {
    const files = [{ name: TAR }, { name: ZIP }];
    expect(pickPackage(files, rel, 'u24-x86_64')).toBeUndefined();
    expect(pickPackage([...files, { name: CORE24 }], rel, 'u24-x86_64')?.name).toBe(CORE24);
    expect(pickPackage(files, rel, LEGACY_SYSTEM)?.name).toBe(ZIP);
    expect(pickPackage(files, rel, undefined)?.name).toBe(ZIP);
    expect(pickPackage([...files, { name: CORE24 }], rel, undefined)?.name).toBe(CORE24);
    expect(pickPackage([{ name: TAR }, { name: CORE24 }], undefined, LEGACY_SYSTEM)?.name).toBe(TAR);
    expect(pickPackage([{ name: TAR }, { name: CORE24 }], undefined, 'u24-x86_64')?.name).toBe(CORE24);
  });

  it('24.04 로봇이면 core 하나만 받는다', async () => {
    const f = await downloadFirmware(rel, () => {}, 'u24-x86_64').done;
    expect(f).toMatchObject({ name: CORE24, size: 50 });
    expect(f.alts).toBeUndefined();
    expect(fs.files.has(`cache/${TAR}`)).toBe(false);
  });

  it('그 시스템 core 가 없는 버전이면 옛 것으로 물러서지 않고 거부한다', async () => {
    fs.plan['core-v1.2610.0-u22-x86_64.rbq'] = { status: 403, size: 0, expected: 0 };
    fs.plan[CORE24] = { status: 403, size: 0, expected: 0 };
    await expect(downloadFirmware(rel, () => {}, 'u24-x86_64').done).rejects.toThrow('u24-x86_64');
    await expect(downloadFirmware(rel, () => {}, 'u22-x86_64').done).rejects.toThrow('u22-x86_64');
    expect(fs.files.has(`cache/${ZIP}`)).toBe(false);
  });

  it('시스템을 모르면(붙어 본 적 없는 로봇) 시스템마다의 core 를 사이드카와 함께 다 받는다 — 없는 것은 건너뛴다', async () => {
    fs.plan[CORE22] = { status: 200, size: 40, expected: 40 };
    sidecars.set(`${CORE22}.sha256`, SHA); sidecars.set(`${CORE24}.sha256`, SHA);
    const seen = new Set<string>();
    const f = await downloadFirmware(rel, (_w, _t, p) => { if (p) seen.add(`${p.i}/${p.n}`); }).done;
    expect(f.name).toBe(CORE22);
    expect(f.alts?.map((x) => x.name)).toEqual([CORE24]);
    expect(fs.files.has(`cache/${CORE22}.sha256`)).toBe(true);
    expect(fs.files.has(`cache/${CORE24}.sha256`)).toBe(true);
    expect(fs.files.has(`cache/${ZIP}`)).toBe(false);
    expect([...seen]).toEqual(['1/4', '2/4', '3/4', '4/4']);
  });

  it('시스템을 모르는데 core 가 하나도 없는 버전(core 이전 릴리스)이면 옛 것으로 간다', async () => {
    delete fs.plan[CORE24];
    const f = await downloadFirmware(rel, () => {}).done;
    expect(f.name).toBe(ZIP);
    expect(f.alts?.map((x) => x.name)).toEqual([TAR]);
  });

  it('전송 전 대조 — 시스템이 맞지 않으면 사유를 준다', () => {
    expect(packageMismatch('RBQ-nightly.zip', 'u24-x86_64')).not.toBeNull();
    expect(packageMismatch('RBQ-nightly.zip', 'u22-x86_64')).not.toBeNull();
    expect(packageMismatch('RBQ-nightly.zip', undefined)).toBeNull();
    expect(packageMismatch('RBQ-nightly.tar.gz', 'u22-x86_64')).not.toBeNull();
    expect(packageMismatch('RBQ-nightly.tar.gz', undefined)).toBeNull();
    expect(packageMismatch(CORE24, 'u24-x86_64')).toBeNull();
    expect(packageMismatch(CORE24, 'u22-x86_64')).toMatch('u24-x86_64');
    expect(packageMismatch(CORE24, undefined)).not.toBeNull();
    expect(packageMismatch('my-notes.txt', 'u24-x86_64')).toBeNull();
  });
});

describe('사이드카(.sha256 · .sig) — 서명 1단계', () => {
  it('core 를 받으면 사이드카도 옆에 저장한다, 없으면 건너뛴다', async () => {
    sidecars.set(`${CORE24}.sha256`, `${SHA}\n`);
    const f = await downloadFirmware(rel, () => {}, 'u24-x86_64').done;
    expect(f.name).toBe(CORE24);
    expect(fs.files.has(`cache/${CORE24}.sha256`)).toBe(true);
    expect(fs.files.has(`cache/${CORE24}.sig`)).toBe(false);
  });

  it('모양이 틀린 값은 버린다 — 틀린 sha256 을 실으면 로봇이 멀쩡한 패키지를 거부한다', () => {
    expect(parseSidecars(`${SHA.toUpperCase()}  ${CORE24}\n`, `${SIG}\n`)).toEqual({ sha256: SHA, signature: SIG });
    expect(parseSidecars('abc', 'not base64')).toEqual({});
    expect(parseSidecars(null, undefined)).toEqual({});
  });
});

describe('otaTransfer — 받은 것 중 핸드셰이크가 알린 시스템에 맞는 것을 보낸다', () => {
  class FakeWs {
    static system: string | undefined;
    static sent: Record<string, unknown>[] = [];
    binaryType = '';
    onmessage: ((ev: { data: string }) => void) | null = null;
    onerror: (() => void) | null = null;
    onclose: (() => void) | null = null;
    constructor() {
      setTimeout(() => this.onmessage?.({ data: JSON.stringify({ type: 'connected', protocol: 2, system: FakeWs.system }) }), 0);
    }
    send(d: string) { FakeWs.sent.push(JSON.parse(d)); }
    close() {}
  }
  const f22 = { uri: `cache/${CORE22}`, name: CORE22, size: 40 };
  const f24 = { uri: `cache/${CORE24}`, name: CORE24, size: 50 };
  const SHA24 = 'cd'.repeat(32);
  beforeEach(() => {
    FakeWs.sent = [];
    vi.stubGlobal('WebSocket', FakeWs);
    fs.text.set(`cache/${CORE22}.sha256`, SHA);
    fs.text.set(`cache/${CORE24}.sha256`, SHA24);
    fs.text.set(`cache/${CORE24}.sig`, SIG);
  });

  it('24.04 로봇이면 대안 중 u24 core 를 그 사이드카와 함께 보낸다', async () => {
    FakeWs.system = 'u24-x86_64';
    const h = otaTransfer('192.168.0.10', f22, () => {}, [f24]);
    await vi.waitFor(() => expect(FakeWs.sent).toHaveLength(1));
    expect(FakeWs.sent[0]).toMatchObject({ type: 'start_transfer', fileName: CORE24, fileSize: 50, sha256: SHA24, signature: SIG });
    h.cancel(); await h.done.catch(() => {});
  });

  it('맞는 것이 없으면 보내지 않고 사유를 준다', async () => {
    FakeWs.system = 'u24-aarch64';
    const h = otaTransfer('192.168.0.10', f22, () => {}, [f24]);
    await expect(h.done).rejects.toThrow('u24-aarch64');
    expect(FakeWs.sent).toEqual([]);
  });
});
