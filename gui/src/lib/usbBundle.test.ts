import { describe, it, expect, vi } from 'vitest';

vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
import {
  isBundleName, parseTarHeader, listTar, readBundle, parseManifest, pickBundle, coreOn, installBundle,
  RESTART_MS, REBOOT_MS, type BundleDeps, type RobotNow, type UsbStep, type BundleSend,
} from './usbBundle';

const enc = new TextEncoder();
function header(name: string, size: number, opts: { type?: string; prefix?: string; magic?: string } = {}) {
  const h = new Uint8Array(512);
  const put = (at: number, s: string) => h.set(enc.encode(s), at);
  put(0, name);
  put(100, '0000644\0');
  put(108, '0000000\0');
  put(116, '0000000\0');
  put(124, `${size.toString(8).padStart(11, '0')}\0`);
  put(136, `${(1763000000).toString(8).padStart(11, '0')}\0`);
  put(156, opts.type ?? '0');
  put(257, opts.magic ?? 'ustar\x0000');
  if (opts.prefix) put(345, opts.prefix);
  put(148, '        ');
  const sum = h.reduce((a, b) => a + b, 0);
  put(148, `${sum.toString(8).padStart(6, '0')}\0 `);
  return h;
}
type Item = { name: string; data: Uint8Array | string; type?: string };
function tar(items: Item[]): Uint8Array {
  const parts: Uint8Array[] = [];
  for (const it of items) {
    const d = typeof it.data === 'string' ? enc.encode(it.data) : it.data;
    parts.push(header(it.name, d.length, { type: it.type }), d, new Uint8Array((512 - (d.length % 512)) % 512));
  }
  parts.push(new Uint8Array(1024));
  const out = new Uint8Array(parts.reduce((n, p) => n + p.length, 0));
  let at = 0;
  for (const p of parts) { out.set(p, at); at += p.length; }
  return out;
}
const reader = (bytes: Uint8Array) => {
  const calls: number[] = [];
  const read = async (offset: number, length: number) => { calls.push(length); return bytes.slice(offset, offset + length); };
  return { read, calls };
};

const LABEL = 'v1.2611.0';
const SHA = 'ab'.repeat(32);
const SIG = `${'A'.repeat(86)}==`;
const SYSTEMS = ['u22-x86_64', 'u22-aarch64', 'u24-x86_64', 'u24-aarch64'];
const coreName = (s: string, l = LABEL) => `core-${l}-${s}.rbq`;
const optName = (s: string, l = LABEL) => `option-demo-${l}-${s}.rbo`;
const LEGACY = 'RBQ-1.2611.0.tar.gz';
const blob = (n: number) => new Uint8Array(n).fill(7);
const SIZES = { core: 1300, opt: 700, legacy: 900 };

function manifest(o: { legacy?: boolean; label?: string; extra?: object[] } = {}) {
  const l = o.label ?? LABEL;
  const files: object[] = [];
  for (const s of SYSTEMS) {
    files.push({ kind: 'core', system: s, name: coreName(s, l), sig: `${coreName(s, l)}.sig`, sha256: SHA, size: SIZES.core });
    files.push({ kind: 'option', option: 'demo', system: s, name: optName(s, l), sha256: SHA, size: SIZES.opt });
  }
  if (o.legacy) files.push({ kind: 'legacy', name: LEGACY, sha256: SHA, size: SIZES.legacy });
  return { format: 1, label: l, channel: 'release', created: '2026-11-13T01:00:00+00:00', files: [...files, ...(o.extra ?? [])] };
}
function bundleTar(o: { legacy?: boolean; skip?: string } = {}) {
  const items: Item[] = [{ name: 'rbq-usb.json', data: JSON.stringify(manifest(o)) }];
  for (const s of SYSTEMS) {
    items.push({ name: coreName(s), data: blob(SIZES.core) }, { name: `${coreName(s)}.sig`, data: `${SIG}\n` },
      { name: `${coreName(s)}.sha256`, data: `${SHA}  ${coreName(s)}\n` }, { name: optName(s), data: blob(SIZES.opt) });
  }
  if (o.legacy) items.push({ name: LEGACY, data: blob(SIZES.legacy) });
  return tar(items.filter((i) => i.name !== o.skip));
}
const opened = async (o: { legacy?: boolean; skip?: string } = {}) => {
  const b = bundleTar(o);
  return readBundle(reader(b).read, b.length);
};

describe('USB 묶음 — 이름', () => {
  it('rbq-usb-<판>.tar 만 묶음으로 본다', () => {
    expect(isBundleName('rbq-usb-v1.2611.0.tar')).toBe(true);
    expect(isBundleName('rbq-usb-v1.2611.0-1113.tar')).toBe(true);
    expect(isBundleName('core-v1.2611.0-u22-x86_64.rbq')).toBe(false);
    expect(isBundleName('RBQ-1.2611.0.tar.gz')).toBe(false);
  });
});

describe('USB 묶음 — tar 머리말', () => {
  it('이름·크기·종류를 읽고, 0 블록은 끝(null)', () => {
    expect(parseTarHeader(header('rbq-usb.json', 1234))).toEqual({ name: 'rbq-usb.json', size: 1234, type: '0' });
    expect(parseTarHeader(new Uint8Array(512))).toBeNull();
  });
  it('prefix 가 있으면 앞에 붙인다', () => {
    expect(parseTarHeader(header('a.rbq', 1, { prefix: 'dir' }))!.name).toBe('dir/a.rbq');
  });
  it('tar 가 아니거나(magic) 체크섬이 틀리면 거부', () => {
    expect(() => parseTarHeader(header('x', 1, { magic: 'nope\0\0' }))).toThrow();
    const h = header('x', 1);
    h[0] = 'y'.charCodeAt(0);
    expect(() => parseTarHeader(h)).toThrow();
  });
});

describe('USB 묶음 — 항목 목록(머리말만 따라간다)', () => {
  it('데이터 시작 위치·크기를 알고, 512 바이트 머리말만 읽는다', async () => {
    const b = tar([{ name: 'a', data: 'hello' }, { name: 'b', data: blob(513) }, { name: 'c', data: blob(512) }]);
    const r = reader(b);
    const list = await listTar(r.read, b.length);
    expect(list).toEqual([{ name: 'a', offset: 512, size: 5 }, { name: 'b', offset: 1536, size: 513 }, { name: 'c', offset: 3072, size: 512 }]);
    expect(r.calls.every((n) => n === 512)).toBe(true);
  });
  it('일반 파일이 아닌 항목(폴더 등)은 건너뛴다', async () => {
    const b = tar([{ name: 'd/', data: '', type: '5' }, { name: 'a', data: 'x' }]);
    expect((await listTar(reader(b).read, b.length)).map((e) => e.name)).toEqual(['a']);
  });
  it('잘린 파일은 거부', async () => {
    const b = tar([{ name: 'a', data: blob(2000) }]).slice(0, 1500);
    await expect(listTar(reader(b).read, b.length)).rejects.toThrow();
  });
  it('첫 항목이 rbq-usb.json 이 아니면 묶음이 아니다', async () => {
    const b = tar([{ name: 'a', data: 'x' }]);
    await expect(readBundle(reader(b).read, b.length)).rejects.toThrow();
  });
});

describe('USB 묶음 — rbq-usb.json', () => {
  it('시스템별 본체·옵션, 구형 꾸러미를 읽는다', () => {
    const m = parseManifest(JSON.stringify(manifest({ legacy: true })));
    expect(m.label).toBe(LABEL);
    expect(m.cores.map((c) => c.system)).toEqual(SYSTEMS);
    expect(m.options[2]).toMatchObject({ option: 'demo', system: 'u24-x86_64', name: optName('u24-x86_64') });
    expect(m.legacy).toMatchObject({ name: LEGACY, size: SIZES.legacy });
  });
  it('모르는 종류·모르는 키는 무시한다(키는 더해질 수 있다)', () => {
    const j = manifest({ extra: [{ kind: 'manual', name: 'x.pdf' }] }) as Record<string, unknown>;
    j.future = { a: 1 };
    expect(parseManifest(JSON.stringify(j)).cores).toHaveLength(4);
  });
  it('format 1 이 아니면 거부', () => {
    expect(() => parseManifest(JSON.stringify({ ...manifest(), format: 2 }))).toThrow();
    expect(() => parseManifest('not json')).toThrow();
  });
  it('sha256·크기 모양이 틀리거나 이름이 판·시스템과 어긋나면 거부', () => {
    const bad = (f: object) => JSON.stringify({ ...manifest(), files: [f] });
    const core = { kind: 'core', system: 'u22-x86_64', name: coreName('u22-x86_64'), sig: `${coreName('u22-x86_64')}.sig`, sha256: SHA, size: 1 };
    expect(() => parseManifest(bad({ ...core, sha256: 'XYZ' }))).toThrow();
    expect(() => parseManifest(bad({ ...core, size: -1 }))).toThrow();
    expect(() => parseManifest(bad({ ...core, system: 'u24-x86_64' }))).toThrow();
    expect(() => parseManifest(bad({ kind: 'legacy', name: 'core.tar', sha256: SHA, size: 1 }))).toThrow();
  });
  it('서명 이름이 <본체 이름>.sig 가 아니면 거부 — 로봇이 거부하기 전에, 큰 파일을 보내기 전에', () => {
    const one = (sig: string) => JSON.stringify({ ...manifest(), files: [{ kind: 'core', system: 'u22-x86_64', name: coreName('u22-x86_64'), sig, sha256: SHA, size: 1 }] });
    expect(() => parseManifest(one(`${coreName('u22-x86_64')}.sig`))).not.toThrow();
    expect(() => parseManifest(one(`${coreName('u24-x86_64')}.sig`))).toThrow();
    expect(() => parseManifest(one('x.sig'))).toThrow();
  });
  it('tar 안에 같은 이름 항목이 둘이면 거부', async () => {
    const b = tar([{ name: 'rbq-usb.json', data: JSON.stringify(manifest()) }, { name: 'a', data: 'x' }, { name: 'a', data: 'y' }]);
    await expect(readBundle(reader(b).read, b.length)).rejects.toThrow();
  });
});

const robot = (o: Partial<RobotNow> = {}): RobotNow => ({ system: 'u24-x86_64', version: '1.2609.0', option: '', optionVersion: '', ...o });

describe('USB 묶음 — 붙은 로봇에 맞는 것 고르기', () => {
  it('로봇 시스템의 본체·서명·옵션을 고른다', async () => {
    const p = pickBundle(await opened(), robot());
    if (typeof p === 'string') throw new Error(p);
    expect(p.system).toBe('u24-x86_64');
    expect(p.core.name).toBe(coreName('u24-x86_64'));
    expect(p.core.sig.name).toBe(`${coreName('u24-x86_64')}.sig`);
    expect(p.option.name).toBe(optName('u24-x86_64'));
    expect(p.legacy).toBeNull();
  });
  it('시스템을 모르면 시작하지 않는다', async () => {
    expect(typeof pickBundle(await opened(), robot({ system: undefined }))).toBe('string');
  });
  it('구형 로봇 — 묶음에 구형 꾸러미가 있으면 그것부터, 대상은 22.04 x86_64', async () => {
    const p = pickBundle(await opened({ legacy: true }), robot({ system: undefined, legacy: true }));
    if (typeof p === 'string') throw new Error(p);
    expect(p.legacy?.name).toBe(LEGACY);
    expect(p.system).toBe('u22-x86_64');
    expect(p.core.name).toBe(coreName('u22-x86_64'));
  });
  it('구형 로봇인데 묶음에 구형 꾸러미가 없으면 사유', async () => {
    expect(pickBundle(await opened(), robot({ system: undefined, legacy: true }))).toContain('구형');
  });
  it('묶음 안에 고른 파일이 없거나 크기가 다르면 사유', async () => {
    expect(pickBundle(await opened({ skip: `${coreName('u24-x86_64')}.sig` }), robot())).toContain('.sig');
    expect(pickBundle(await opened({ skip: optName('u24-x86_64') }), robot())).toContain(optName('u24-x86_64'));
  });
  it('다른 옵션이 깔린 로봇은 시작하지 않는다', async () => {
    expect(typeof pickBundle(await opened(), robot({ option: 'other' }))).toBe('string');
  });
});

describe('USB 묶음 — 본체 판 비교', () => {
  it('정식은 글자 그대로 같아야 한다', () => {
    expect(coreOn('1.2611.0', 'v1.2611.0', false)).toBe(true);
    expect(coreOn('1.2611.0-3-gabc1234', 'v1.2611.0', false)).toBe(false);
    expect(coreOn(undefined, 'v1.2611.0', true)).toBe(false);
  });
  it('나이틀리는 날짜를 볼 수 없어 — 방금 깐 뒤에만 같은 줄이면 맞다고 본다', () => {
    expect(coreOn('1.2611.0-nightly.ab12cd34', 'v1.2611.0-1113', false)).toBe(false);
    expect(coreOn('1.2611.0-nightly.ab12cd34', 'v1.2611.0-1113', true)).toBe(true);
    expect(coreOn('1.2609.0-nightly.ab12cd34', 'v1.2611.0-1113', true)).toBe(false);
  });
});

function fake(init: RobotNow, o: { restartFails?: number; noEffect?: string; selfRestart?: string; replyLost?: boolean; lostOnce?: boolean; standing?: boolean } = {}) {
  let clock = 0;
  const st: RobotNow = { ...init };
  const link = { up: true, drops: [] as number[] };
  const pending: Partial<RobotNow>[] = [];
  const log: string[] = [];
  const steps: UsbStep[] = [];
  const sent: BundleSend[] = [];
  let lost = !!o.lostOnce;
  const effect = (name: string): Partial<RobotNow> =>
    name === o.noEffect ? {}
      : name.startsWith('RBQ-') ? { legacy: false, system: 'u22-x86_64', version: '1.2611.0' }
      : name.startsWith('core-') ? { version: '1.2611.0' }
      : { option: 'demo', optionVersion: LABEL };
  const bounceRobot = () => { for (const p of pending.splice(0)) Object.assign(st, p); link.up = false; link.drops.push(++clock); };
  const status = (n: number, m = 'restart') => Object.assign(new Error(m), { status: n });
  const deps: BundleDeps = {
    robot: async () => ({ ...st }),
    signature: async () => SIG,
    send: async (f) => {
      log.push(`send ${f.name}`); sent.push(f); pending.push(effect(f.name));
      if (f.name === o.selfRestart) bounceRobot();
    },
    own: async () => { log.push('own'); },
    restart: async () => {
      log.push('restart');
      if (!link.up) throw status(0, 'not sent');
      if (lost) { lost = false; throw status(0, 'no reply'); }
      if (o.restartFails != null) throw status(o.restartFails);
      if (st.legacy) throw status(404, 'unknown command route');
      bounceRobot();
      if (o.replyLost) throw status(0, 'no reply');
    },
    reboot: async () => { log.push('reboot'); bounceRobot(); },
    standing: () => !!o.standing,
    waitBack: async (since, ms) => {
      log.push(`wait ${ms}`);
      link.up = true;
      if ((link.drops.at(-1) ?? 0) < since) throw new Error('timeout');
    },
    step: (s) => steps.push(s),
    now: () => ++clock,
  };
  return { deps, log, steps, sent, st };
}

const W = `wait ${RESTART_MS}`;
describe('USB 묶음 설치 — 단계', () => {
  it('새 로봇: 본체(서명 포함) → 재시작 → 옵션 → 재시작, 범위만 보낸다', async () => {
    const b = await opened();
    const f = fake(robot());
    const done = await installBundle(b, f.deps);
    expect(f.log).toEqual([`send ${coreName('u24-x86_64')}`, W, 'own', 'restart', W, 'own', `send ${optName('u24-x86_64')}`, W, 'own', 'restart', W, 'own']);
    expect(f.sent[0]).toMatchObject({ size: SIZES.core, sidecars: { sha256: SHA, signature: SIG } });
    expect(f.sent[0].range.length).toBe(SIZES.core);
    expect(f.sent[1].sidecars).toEqual({ sha256: SHA });
    expect(f.steps).toEqual(['check', 'core', 'restart', 'check', 'option', 'restart', 'check']);
    expect(done).toContain(LABEL);
    expect(done).toContain('DEMO');
  });

  it('이미 같은 판인 단계는 건너뛴다 — 본체만 같으면 옵션만, 둘 다 같으면 아무것도 안 보낸다', async () => {
    const b = await opened();
    const one = fake(robot({ version: '1.2611.0' }));
    await installBundle(b, one.deps);
    expect(one.log.filter((l) => l.startsWith('send'))).toEqual([`send ${optName('u24-x86_64')}`]);
    const none = fake(robot({ version: '1.2611.0', option: 'demo', optionVersion: LABEL }));
    expect(await installBundle(b, none.deps)).toContain(LABEL);
    expect(none.log).toEqual([]);
  });

  it('구형 로봇: 옛 꾸러미 → 재시작 길이 없어(404) PC 재부팅 → 시스템이 나오고 본체가 같은 판이면 본체는 건너뛰고 옵션', async () => {
    const b = await opened({ legacy: true });
    const f = fake(robot({ system: undefined, legacy: true, version: '1.2609.0' }));
    await installBundle(b, f.deps);
    expect(f.log).toEqual([`send ${LEGACY}`, W, 'own', 'restart', 'reboot', `wait ${REBOOT_MS}`,
      'own', `send ${optName('u22-x86_64')}`, W, 'own', 'restart', W, 'own']);
    expect(f.steps[1]).toBe('legacy');
  });

  it('구형 로봇이 서 있으면 재부팅을 보내지 않고 앉히라는 사유로 바로 멈춘다 — 옛 판은 거부를 알리지 않아 6분을 헛기다렸다', async () => {
    const b = await opened({ legacy: true });
    const f = fake(robot({ system: undefined, legacy: true, version: '1.2609.0' }), { standing: true });
    await expect(installBundle(b, f.deps)).rejects.toThrow('앉히');
    expect(f.log).not.toContain('reboot');
    expect(f.log.some((l) => l.startsWith(`wait ${REBOOT_MS}`))).toBe(false);
  });

  it('구형 꾸러미를 깔았는데도 구형으로 답하면 멈춘다', async () => {
    const b = await opened({ legacy: true });
    const f = fake(robot({ system: undefined, legacy: true }), { noEffect: LEGACY });
    await expect(installBundle(b, f.deps)).rejects.toThrow();
    expect(f.log.some((l) => l.startsWith('send core'))).toBe(false);
  });

  it('서 있어 재시작이 거부되면(409) 앉히라는 사유로 멈춘다', async () => {
    const f = fake(robot(), { restartFails: 409 });
    await expect(installBundle(await opened(), f.deps)).rejects.toThrow('앉히');
    expect(f.log).not.toContain(`send ${optName('u24-x86_64')}`);
  });

  it('재시작이 503(서비스 없음) 등으로 실패하면 재부팅으로 넘어가지 않고 사유로 멈춘다', async () => {
    const f = fake(robot(), { restartFails: 503 });
    await expect(installBundle(await opened(), f.deps)).rejects.toThrow();
    expect(f.log).not.toContain('reboot');
    expect(f.log).not.toContain(`send ${optName('u24-x86_64')}`);
  });

  it('다시 켜진 뒤 본체 판이 그대로면 옵션을 보내지 않고 멈춘다', async () => {
    const f = fake(robot(), { noEffect: coreName('u24-x86_64') });
    await expect(installBundle(await opened(), f.deps)).rejects.toThrow('1.2609.0');
    expect(f.log).not.toContain(`send ${optName('u24-x86_64')}`);
  });

  it('다시 연결 기한을 넘기면(waitBack 실패) 그 사유로 끝난다 — 도는 중에 머물지 않는다', async () => {
    const f = fake(robot());
    f.deps.waitBack = async () => { throw new Error('timeout'); };
    await expect(installBundle(await opened(), f.deps)).rejects.toThrow('timeout');
  });

  it('재시작은 됐는데 답을 못 받았으면(0) 다시 붙는지로 판단한다', async () => {
    const f = fake(robot(), { replyLost: true });
    await expect(installBundle(await opened(), f.deps)).resolves.toContain(LABEL);
  });

  it('보내는 중에 로봇이 스스로 다시 켜져 끊겨 있으면 — 다시 붙기를 기다린 뒤 재시작을 요청하고 이어 간다', async () => {
    const f = fake(robot(), { selfRestart: coreName('u24-x86_64') });
    await expect(installBundle(await opened(), f.deps)).resolves.toContain(LABEL);
    expect(f.log.slice(0, 4)).toEqual([`send ${coreName('u24-x86_64')}`, W, 'own', 'restart']);
  });

  it('마지막으로 다시 켜진 뒤에도 빈 자리를 다시 잡아 시작한 사람이 조종하던 그대로 끝난다 — 남이 쥐었으면 완료는 그대로', async () => {
    const f = fake(robot());
    await installBundle(await opened(), f.deps);
    expect(f.log.at(-1)).toBe('own');
    const g = fake(robot());
    let n = 0;
    g.deps.own = async () => { if (++n === 4) throw new Error('다른 기기'); };
    await expect(installBundle(await opened(), g.deps)).resolves.toContain(LABEL);
    expect(n).toBe(4);
  });

  it('다시 켜진 뒤 남이 제어권을 쥐었으면 옵션을 보내지 않고 멈춘다 — 옵션만 깔리고 재시작을 못 하는 채로 남지 않게', async () => {
    const f = fake(robot());
    let n = 0;
    f.deps.own = async () => { if (++n === 2) throw new Error('다른 기기'); };
    await expect(installBundle(await opened(), f.deps)).rejects.toThrow('다른 기기');
    expect(f.log).not.toContain(`send ${optName('u24-x86_64')}`);
  });

  it('스스로 다시 켜진 뒤 재시작 요청이 닿지 않았으면(0) 전송을 시작한 뒤의 끊김으로 다시 켜짐을 인정한다', async () => {
    const f = fake(robot(), { selfRestart: coreName('u24-x86_64'), lostOnce: true });
    await expect(installBundle(await opened(), f.deps)).resolves.toContain(LABEL);
  });
});
