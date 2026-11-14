export type Probe = { kind: 'tls'; fp: string; serial: string | null } | { kind: 'plain' } | { kind: 'down' };
export type Verdict = 'learn' | 'ok' | 'changed' | 'fallback' | 'refuse-old' | 'down';

export const pinKey = (entryKey: string, port: number) => `${entryKey}@${port}`;
export const RELEARN_PIN = 'relearn';

export function decide(pinned: string | undefined, probe: Probe): Verdict {
  if (probe.kind === 'down') return 'down';
  if (probe.kind === 'plain') return pinned ? 'refuse-old' : 'fallback';
  if (!pinned || pinned === RELEARN_PIN) return 'learn';
  return pinned === probe.fp ? 'ok' : 'changed';
}

export interface TlsPlatform {
  probe(host: string, port: number): Promise<Probe>;
  engage(host: string, port: number, fp: string | null): Promise<void>;
  block(host: string, port: number): void;
  kept(host: string, port: number): boolean;
  pinnedFp(host: string, port: number): string | undefined;
}

export interface RobotTlsPlatform extends TlsPlatform {
  base(host: string, port: number): string | undefined;
  forgetPlain(): void;
  allowPlain(): void;
  plainHeld(): boolean;
}

export type TlsIssue =
  | { kind: 'changed'; entryKey: string; port: number; fp: string; forget: number[] }
  | { kind: 'refuse-old'; entryKey: string };

type PinSource = {
  selectedKey: string | null;
  selectedSsid: string | null;
  pins: Record<string, string>;
  keyFor(serial: string, ip: string): Promise<string>;
  currentSsid(): Promise<string | null>;
};

let source: () => PinSource = () => ({ selectedKey: null, selectedSsid: null, pins: {}, keyFor: async (s) => s, currentSsid: async () => null });
let onIssue: (i: TlsIssue) => void = () => {};

export function setTlsPinSource(fn: () => PinSource) { source = fn; }
export function setTlsIssueHandler(fn: (i: TlsIssue) => void) { onIssue = fn; }
export function reportTlsIssue(i: TlsIssue) { onIssue(i); }

const PORTS = [8080, 8081, 8082] as const;
let last: { motionIp: string; probes: Partial<Record<number, Probe>>; raw: Partial<Record<number, Probe>> } | null = null;

function judge(key: string | null, port: number, probe: Probe, pins: Record<string, string>): Verdict {
  return decide(key ? pins[pinKey(key, port)] : undefined, probe);
}

function unchecked(key: string, raw: Partial<Record<number, Probe>>, pins: Record<string, string>): number[] {
  return PORTS.filter((port) => raw[port]?.kind === 'down' && pins[pinKey(key, port)] !== undefined);
}

export async function secureRobot(
  motionIp: string, visionIp: string, p: TlsPlatform, report: (i: TlsIssue) => void = onIssue, opts: { live?: boolean } = {},
): Promise<{ result: 'ok' | 'refused'; key: string | null }> {
  const src = source();
  const hostOf = (port: number) => (port === 8080 ? motionIp : visionIp || motionIp);
  const raw: Partial<Record<number, Probe>> = {};
  const look = async (host: string, port: number): Promise<Probe> => {
    const r = (raw[port] = await p.probe(host, port));
    const mine = src.selectedKey ? src.pins[pinKey(src.selectedKey, port)] : undefined;
    return opts.live && r.kind === 'plain' && mine !== undefined && mine !== RELEARN_PIN && p.pinnedFp(host, port) === mine ? { kind: 'down' } : r;
  };
  const probes: Partial<Record<number, Probe>> = {};
  probes[8080] = await look(motionIp, 8080);
  const first = probes[8080];

  let key: string | null;
  const serial = first.kind === 'tls' ? first.serial : null;
  if (serial !== null) key = await src.keyFor(serial, motionIp);
  else if (first.kind === 'plain') {
    key = src.selectedKey;
    const now = await src.currentSsid();
    if (key && src.pins[pinKey(key, 8080)] && now && src.selectedSsid && now !== src.selectedSsid) key = null;
  } else key = src.selectedKey;

  const rest = await Promise.all(PORTS.slice(1).map((port) => look(hostOf(port), port)));
  PORTS.slice(1).forEach((port, i) => { probes[port] = rest[i]; });
  last = { motionIp, probes, raw };

  let refused = false;
  let oldRaised = false;
  for (const port of PORTS) {
    const probe = probes[port]!;
    const host = hostOf(port);
    const v = judge(key, port, probe, src.pins);
    if (v === 'learn' || v === 'ok') await p.engage(host, port, (probe as { fp: string }).fp);
    else if (v === 'fallback') await p.engage(host, port, null);
    else if (v === 'changed' || v === 'refuse-old') {
      refused = true;
      p.block(host, port);
      if (v === 'changed') report({ kind: 'changed', entryKey: key!, port, fp: (probe as { fp: string }).fp, forget: serial !== null ? unchecked(key!, raw, src.pins) : [] });
      else if (!oldRaised) { oldRaised = true; report({ kind: 'refuse-old', entryKey: key! }); }
    } else {
      if (!p.kept(host, port)) p.block(host, port);
    }
  }
  return { result: refused ? 'refused' : 'ok', key };
}

export function settleRobot(entryKey: string, motionIp: string): { learn: Record<number, string>; refused: boolean } {
  const mine = last;
  if (!mine || mine.motionIp !== motionIp) return { learn: {}, refused: false };
  last = null;
  const { pins } = source();
  const learn: Record<number, string> = {};
  let refused = false;
  let oldRaised = false;
  for (const port of PORTS) {
    const probe = mine.probes[port];
    if (!probe) continue;
    const v = judge(entryKey, port, probe, pins);
    if (v === 'learn') learn[port] = (probe as { fp: string }).fp;
    else if (v === 'changed') { refused = true; onIssue({ kind: 'changed', entryKey, port, fp: (probe as { fp: string }).fp, forget: unchecked(entryKey, mine.raw, pins) }); }
    else if (v === 'refuse-old') { refused = true; if (!oldRaised) { oldRaised = true; onIssue({ kind: 'refuse-old', entryKey }); } }
  }
  return { learn, refused };
}

export function isPinnedSerial(serial: string): boolean {
  const prefix = `${serial}@`;
  return Object.keys(source().pins).some((k) => k.startsWith(prefix));
}
