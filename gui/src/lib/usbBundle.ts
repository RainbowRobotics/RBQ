import { t } from '@/lib/i18n';

export const isBundleName = (name: string) => /^rbq-usb-.+\.tar$/.test(name);

export type ReadAt = (offset: number, length: number) => Promise<Uint8Array>;
export interface TarEntry { name: string; offset: number; size: number }

const BLOCK = 512;
const BROKEN = () => t('묶음 파일이 손상됐습니다 — USB 에 다시 복사하세요');

function field(b: Uint8Array, at: number, len: number): string {
  let s = '';
  for (let i = at; i < at + len && b[i]; i++) s += String.fromCharCode(b[i]);
  return s;
}
const octal = (s: string) => parseInt(s.trim() || '0', 8);

export function parseTarHeader(b: Uint8Array): { name: string; size: number; type: string } | null {
  if (b.length < BLOCK) throw new Error(BROKEN());
  if (b.every((x) => x === 0)) return null;
  if (field(b, 257, 5) !== 'ustar') throw new Error(t('USB 묶음(tar) 파일이 아닙니다'));
  let sum = 0;
  for (let i = 0; i < BLOCK; i++) sum += i >= 148 && i < 156 ? 32 : b[i];
  if (sum !== octal(field(b, 148, 8))) throw new Error(BROKEN());
  const prefix = field(b, 345, 155), name = field(b, 0, 100);
  return { name: prefix ? `${prefix}/${name}` : name, size: octal(field(b, 124, 12)), type: field(b, 156, 1) || '0' };
}

export async function listTar(read: ReadAt, total: number): Promise<TarEntry[]> {
  const out: TarEntry[] = [];
  for (let at = 0; at + BLOCK <= total;) {
    const h = parseTarHeader(await read(at, BLOCK));
    if (!h) break;
    const offset = at + BLOCK;
    if (offset + h.size > total) throw new Error(BROKEN());
    if (h.type === '0') out.push({ name: h.name, offset, size: h.size });
    at = offset + Math.ceil(h.size / BLOCK) * BLOCK;
  }
  return out;
}

export interface BundleCore { system: string; name: string; sig: string; sha256: string; size: number }
export interface BundleOption { option: string; system: string; name: string; sha256: string; size: number }
export interface BundleLegacy { name: string; sha256: string; size: number }
export interface BundleManifest {
  label: string; channel: string; created: string;
  cores: BundleCore[]; options: BundleOption[]; legacy: BundleLegacy | null;
}

const SHA_RE = /^[0-9a-f]{64}$/;
type Raw = Record<string, unknown>;

export function parseManifest(text: string): BundleManifest {
  let j: Raw;
  try { j = JSON.parse(text); } catch { throw new Error(BROKEN()); }
  if (j?.format !== 1) throw new Error(`${t('이 앱이 모르는 묶음 형식입니다 — 앱을 업데이트하세요')} (format ${String(j?.format)})`);
  const label = j.label;
  if (typeof label !== 'string' || !/^v\S+$/.test(label) || !Array.isArray(j.files)) throw new Error(BROKEN());
  const str = (f: Raw, k: string) => (typeof f[k] === 'string' ? (f[k] as string) : '');
  const base = (f: Raw, nameOk: boolean) => {
    const size = f.size;
    if (!nameOk || !SHA_RE.test(str(f, 'sha256')) || !Number.isSafeInteger(size) || (size as number) <= 0) {
      throw new Error(`${BROKEN()} (${str(f, 'name') || '?'})`);
    }
    return { name: str(f, 'name'), sha256: str(f, 'sha256'), size: size as number };
  };
  const m: BundleManifest = { label, channel: str(j, 'channel'), created: str(j, 'created'), cores: [], options: [], legacy: null };
  for (const f of j.files as Raw[]) {
    const system = str(f, 'system');
    if (f?.kind === 'core') {
      const name = `core-${label}-${system}.rbq`;
      m.cores.push({ ...base(f, str(f, 'name') === name && str(f, 'sig') === `${name}.sig`), system, sig: str(f, 'sig') });
    } else if (f?.kind === 'option') {
      const option = str(f, 'option');
      m.options.push({ ...base(f, /^[a-z]{2,16}$/.test(option) && str(f, 'name') === `option-${option}-${label}-${system}.rbo`), option, system });
    } else if (f?.kind === 'legacy') {
      m.legacy = base(f, /^RBQ-.+\.tar\.gz$/.test(str(f, 'name')));
    }
  }
  return m;
}

export interface Bundle { manifest: BundleManifest; entries: TarEntry[] }

export async function readBundle(read: ReadAt, total: number): Promise<Bundle> {
  const entries = await listTar(read, total);
  if (new Set(entries.map((e) => e.name)).size !== entries.length) throw new Error(BROKEN());
  const head = entries[0];
  if (head?.name !== 'rbq-usb.json' || head.size > 1 << 20) throw new Error(t('USB 묶음(tar) 파일이 아닙니다'));
  return { manifest: parseManifest(new TextDecoder().decode(await read(head.offset, head.size))), entries };
}

export interface RobotNow { system?: string; legacy?: boolean; version?: string; option?: string; optionVersion?: string }
export interface Part { name: string; offset: number; size: number }
export interface BundlePick {
  system: string;
  legacy: Part | null;
  core: Part & { sha256: string; sig: Part };
  option: Part & { option: string; sha256: string };
}

const LEGACY_BUILD = 'u22-x86_64';

export function pickBundle(b: Bundle, robot: RobotNow): BundlePick | string {
  const m = b.manifest;
  if (robot.legacy && !m.legacy) return t('이 묶음에는 구형 로봇용 꾸러미가 없습니다 — 대시보드에서 다시 받으세요');
  const system = robot.legacy ? LEGACY_BUILD : robot.system;
  if (!system) return t('로봇의 시스템을 아직 모릅니다 — 로봇에 연결한 채 잠시 뒤 다시 고르세요');
  const core = m.cores.find((c) => c.system === system);
  const option = m.options.find((o) => o.system === system);
  if (!core || !option) return `${t('이 묶음에는 이 로봇 시스템용 파일이 없습니다')} (${system})`;
  if (robot.option && robot.option !== option.option) {
    return t('이 로봇에는 {x}이 깔려 있습니다 — 먼저 지우세요').replace('{x}', `${robot.option.toUpperCase()} ${t('옵션')}`);
  }
  const at = (name: string, size?: number): Part | string => {
    const e = b.entries.find((x) => x.name === name);
    return e && (size == null || e.size === size) ? e : `${t('묶음 안에 이 파일이 없거나 크기가 다릅니다 — 다시 받으세요')}: ${name}`;
  };
  const parts = [at(core.name, core.size), at(core.sig), at(option.name, option.size), ...(robot.legacy && m.legacy ? [at(m.legacy.name, m.legacy.size)] : [])];
  const why = parts.find((p): p is string => typeof p === 'string');
  if (why) return why;
  const [c, sig, o, legacy] = parts as Part[];
  if (sig.size > 1024) return `${t('묶음 안에 이 파일이 없거나 크기가 다릅니다 — 다시 받으세요')}: ${core.sig}`;
  return { system, legacy: legacy ?? null, core: { ...c, sha256: core.sha256, sig }, option: { ...o, option: option.option, sha256: option.sha256 } };
}

export function coreOn(version: string | undefined, label: string, sent: boolean): boolean {
  const m = /^v(.+?)(-\d{4})?$/.exec(label);
  if (!version || !m) return false;
  return m[2] ? sent && version.startsWith(`${m[1]}-nightly.`) : version === m[1];
}

export type UsbStep = 'check' | 'legacy' | 'core' | 'option' | 'restart';
export interface BundleSend { name: string; size: number; range: { offset: number; length: number }; sidecars?: { sha256: string; signature?: string } }

export interface BundleDeps {
  robot(): Promise<RobotNow>;
  signature(sig: Part): Promise<string>;
  send(f: BundleSend): Promise<void>;
  own(): Promise<void>;
  restart(): Promise<void>;
  reboot(): Promise<void>;
  standing(): boolean;
  waitBack(since: number, ms: number): Promise<void>;
  step(s: UsbStep): void;
  now(): number;
}

export const RESTART_MS = 3 * 60_000;
export const REBOOT_MS = 6 * 60_000;

const statusOf = (e: unknown) => (typeof (e as { status?: unknown })?.status === 'number' ? (e as { status: number }).status : -1);

async function bounce(d: BundleDeps, sent: number) {
  d.step('restart');
  await d.waitBack(0, RESTART_MS);
  await d.own();
  const at = d.now();
  let reboot = false;
  let reached = true;
  try { await d.restart(); } catch (e) {
    const st = statusOf(e);
    if (st === 409) throw new Error(t('로봇이 서 있어 로봇 소프트웨어를 다시 시작하지 못했습니다 — 앉히고 묶음을 다시 고르세요'));
    if (st === 503) throw new Error(t('이 로봇은 로봇 서비스를 다시 시작할 수 없습니다 — 로봇 PC 를 직접 다시 켜고 묶음을 다시 고르면 깔린 단계는 건너뜁니다'));
    if (st === 0) reached = false;
    else if (st === 404 || st === 405) reboot = true;
    else throw e;
  }
  if (reboot && d.standing()) throw new Error(t('로봇이 서 있어 로봇 PC 재부팅을 보내지 않았습니다 — 앉히고 묶음을 다시 고르세요'));
  if (reboot) { try { await d.reboot(); } catch (e) { if (statusOf(e) !== 0) throw e; reached = false; } }
  await d.waitBack(reached ? at : sent, reboot ? REBOOT_MS : RESTART_MS);
  d.step('check');
}

export async function installBundle(b: Bundle, d: BundleDeps): Promise<string> {
  const label = b.manifest.label;
  d.step('check');
  let now = await d.robot();
  const pick = pickBundle(b, now);
  if (typeof pick === 'string') throw new Error(pick);
  const signature = await d.signature(pick.core.sig);
  const part = (p: Part): BundleSend => ({ name: p.name, size: p.size, range: { offset: p.offset, length: p.size } });
  let sent = false;
  let bounced = false;
  if (pick.legacy) {
    d.step('legacy');
    const at = d.now();
    await d.send(part(pick.legacy));
    await bounce(d, at);
    bounced = true;
    now = await d.robot();
    sent = true;
  }
  if (now.system !== pick.system) {
    throw new Error(`${t('로봇 시스템이 묶음에서 고른 것과 다릅니다')} — ${t('로봇')} ${now.system ?? '?'}, ${t('묶음')} ${pick.system}`);
  }
  if (!coreOn(now.version, label, sent)) {
    d.step('core');
    const at = d.now();
    await d.send({ ...part(pick.core), sidecars: { sha256: pick.core.sha256, signature } });
    await bounce(d, at);
    bounced = true;
    now = await d.robot();
    if (!coreOn(now.version, label, true)) throw new Error(`${t('본체 판이 바뀌지 않았습니다')} — ${t('로봇')} v${now.version ?? '?'}`);
  }
  const optOn = () => now.option === pick.option.option && now.optionVersion === label;
  if (!optOn()) {
    d.step('option');
    if (bounced) await d.own();
    const at = d.now();
    await d.send({ ...part(pick.option), sidecars: { sha256: pick.option.sha256 } });
    await bounce(d, at);
    bounced = true;
    now = await d.robot();
    if (!optOn()) throw new Error(`${t('옵션이 깔리지 않았습니다')} — ${t('로봇')} ${now.option || '—'} ${now.optionVersion ?? ''}`.trim());
  }
  if (bounced) await d.own().catch(() => undefined);
  return `${t('설치 완료')} — ${t('본체')} ${label} · ${pick.option.option.toUpperCase()} ${label}`;
}
