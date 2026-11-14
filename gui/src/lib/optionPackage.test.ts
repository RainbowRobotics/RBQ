import { describe, it, expect, vi } from 'vitest';

vi.mock('react-native', () => ({ Platform: { OS: 'android' } }));
const cached = new Map<string, number>();
vi.mock('expo-file-system/legacy', () => ({
  cacheDirectory: 'file:///cache/',
  getInfoAsync: async (uri: string) => ({ exists: cached.has(uri), size: cached.get(uri) ?? 0 }),
}));
vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));

import { parseOptionFile, optionRowState, optionTitle, optionErrorText, optionFileFor, optionFollow, optionBeside } from './optionPackage';

describe('옵션 파일 이름', () => {
  it('정식·나이틀리 이름을 푼다', () => {
    expect(parseOptionFile('option-abc-v1.2611.0-u22-x86_64.rbo')).toEqual({ name: 'abc', label: 'v1.2611.0', system: 'u22-x86_64' });
    expect(parseOptionFile('option-abc-v1.2611.0-1102-u24-aarch64.rbo')).toEqual({ name: 'abc', label: 'v1.2611.0-1102', system: 'u24-aarch64' });
  });
  it('꼴이 아니면 null — 대문자·다른 확장자·한 글자 이름·모르는 arch·경로·본체 패키지', () => {
    for (const n of ['option-ABC-v1.0.0-u22-x86_64.rbo', 'option-abc-v1.0.0-u22-x86_64.rbq', 'option-k-v1.0.0-u22-x86_64.rbo',
      'option-abc-v1.0.0-u22-arm.rbo', 'option-../x-v1.0.0-u22-x86_64.rbo', 'core-v1.2609.0-u22-x86_64.rbq']) {
      expect(parseOptionFile(n)).toBeNull();
    }
  });
});

describe('옵션 줄 상태 — 한 로봇에 옵션 하나', () => {
  const abc1 = { name: 'abc', label: 'v1', system: 'u22-x86_64' };
  const robot = (option: string, optionVersion = '', system = 'u22-x86_64') => ({ option, optionVersion, system });
  it('같은 이름·같은 판 = 깔림, 다른 판 = 업데이트(옛 판도 — 판 하한 없음)', () => {
    expect(optionRowState(abc1, robot('abc', 'v1'))).toBe('installed');
    expect(optionRowState(abc1, robot('abc', 'v2'))).toBe('update');
  });
  it('깔린 게 없으면 설치 가능', () => {
    expect(optionRowState(abc1, robot(''))).toBe('installable');
  });
  it('다른 옵션이 깔려 있으면 먼저 지워야 한다', () => {
    expect(optionRowState(abc1, robot('xyz', 'v9'))).toBe('otherInstalled');
  });
  it('다른 시스템용 파일은 어느 경우든 다른 로봇용', () => {
    expect(optionRowState(abc1, robot('', '', 'u24-x86_64'))).toBe('otherSystem');
    expect(optionRowState(abc1, robot('abc', 'v1', 'u22-aarch64'))).toBe('otherSystem');
  });
  it('로봇 시스템을 모르면 시스템으로 막지 않는다(로봇이 받기 전에 다시 본다)', () => {
    expect(optionRowState(abc1, robot('', '', ''))).toBe('installable');
  });
});

describe('표시 이름 — 앱 코드에 고객 이름을 적지 않는다', () => {
  it('파일의 이름을 대문자로', () => {
    expect(optionTitle('abc')).toBe('ABC 옵션');
  });
});

describe('로봇 거부 사유 — 서명은 앱 말로', () => {
  it('서명 없음·틀림은 같은 안내 하나로', () => {
    for (const m of ['Option package signature missing — use a file published by Rainbow Robotics',
      'Option package signature invalid — use a file published by Rainbow Robotics']) {
      expect(optionErrorText(m)).toBe('서명이 없거나 맞지 않는 옵션 파일입니다 — 레인보우로보틱스가 배포한 파일을 쓰세요');
    }
  });
  it('그 밖의 사유는 로봇 문구 그대로', () => {
    expect(optionErrorText('Robot is busy with another update (x) — try again when it finishes')).toBe('Robot is busy with another update (x) — try again when it finishes');
  });
});

describe('본체를 따라가는 옵션 — 본체 패치 때 깔린 옵션의 같은 판도 받아 먼저 보낸다', () => {
  it('본체 파일 이름에서 같은 판·같은 시스템의 옵션 파일 이름', () => {
    expect(optionFileFor('core-v1.2611.0-u22-x86_64.rbq', 'abc')).toBe('option-abc-v1.2611.0-u22-x86_64.rbo');
    expect(optionFileFor('core-v1.2611.0-1102-u24-aarch64.rbq', 'xyz')).toBe('option-xyz-v1.2611.0-1102-u24-aarch64.rbo');
  });
  it('본체 파일이 아니거나 옵션 이름이 규칙 밖이면 null', () => {
    expect(optionFileFor('RBQ-v1.2611.0.tar.gz', 'abc')).toBeNull();
    expect(optionFileFor('core-v1.2611.0-u22-x86_64.rbq', '')).toBeNull();
    expect(optionFileFor('core-v1.2611.0-u22-x86_64.rbq', 'ABC')).toBeNull();
    expect(optionFileFor('core-v1.2611.0-u22-x86_64.rbq', '../x')).toBeNull();
  });
  const core = 'core-v1.2612.0-u22-x86_64.rbq';
  it('깔린 옵션이 없으면 따라갈 것도 없다', () => {
    expect(optionFollow(core, { option: '', optionVersion: '' })).toBeNull();
  });
  it('깔린 옵션의 판이 다르면 보낸다', () => {
    expect(optionFollow(core, { option: 'abc', optionVersion: 'v1.2611.0' }))
      .toEqual({ name: 'option-abc-v1.2612.0-u22-x86_64.rbo', label: 'v1.2612.0', same: false });
  });
  it('이미 같은 판이면 보내지 않는다(same)', () => {
    expect(optionFollow(core, { option: 'abc', optionVersion: 'v1.2612.0' })?.same).toBe(true);
  });
});

describe('본체 옆에 받아 둔 옵션 파일 찾기 — 보낼 때', () => {
  const OPT = 'option-abc-v1.2612.0-u22-x86_64.rbo';
  it('네이티브: 앱 캐시에 있으면 그 파일([옵션] 탭에서 가져온 것도 같은 자리)', async () => {
    cached.clear(); cached.set(`file:///cache/${OPT}`, 9);
    expect(await optionBeside({ uri: 'file:///cache/core-v1.2612.0-u22-x86_64.rbq' }, OPT)).toEqual({ uri: `file:///cache/${OPT}`, name: OPT, size: 9 });
    cached.clear();
    expect(await optionBeside({ uri: 'file:///cache/core-v1.2612.0-u22-x86_64.rbq' }, OPT)).toBeNull();
  });
  it('데스크탑: 본체를 흘려 주는 프록시 주소에서 이름만 바꿔 묻는다(HEAD)', async () => {
    const seen: string[] = [];
    vi.stubGlobal('fetch', vi.fn(async (u: string, o?: { method?: string }) => {
      seen.push(`${o?.method} ${u}`);
      const ok = u.includes(`name=${OPT}`);
      return { ok, headers: { get: () => (ok ? '9' : null) } };
    }));
    const core = { uri: '/fw-file?channel=release&tag=v1.2612.0&folder=v1.2612.0&name=core-v1.2612.0-u22-x86_64.rbq' };
    expect(await optionBeside(core, OPT)).toEqual({ uri: `/fw-file?channel=release&tag=v1.2612.0&folder=v1.2612.0&name=${OPT}`, name: OPT, size: 9 });
    expect(seen).toEqual([`HEAD /fw-file?channel=release&tag=v1.2612.0&folder=v1.2612.0&name=${OPT}`]);
    vi.unstubAllGlobals();
  });
  it('옵션 이름이 규칙 밖이면 찾지 않는다', async () => {
    expect(await optionBeside({ uri: 'file:///cache/x' }, '../etc/passwd')).toBeNull();
  });
});
