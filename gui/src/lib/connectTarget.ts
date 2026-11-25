
export type Route = 'direct' | 'remote';

export type Target = {
  route: Route;
  addr: string;
  remote?: Record<string, string>;
  webrtcToken: string;
  label: string;
};

export type TargetError = 'no_addr';

export function directTarget(addr: string, opts: { token?: string; label?: string } = {}):
  { ok: true; target: Target } | { ok: false; reason: TargetError } {
  const a = addr.trim();
  if (!a) return { ok: false, reason: 'no_addr' };
  return {
    ok: true,
    target: {
      route: 'direct', addr: a,
      webrtcToken: (opts.token ?? '').trim(), label: opts.label?.trim() || a,
    },
  };
}

const sameValues = (a: Record<string, string> = {}, b: Record<string, string> = {}) =>
  Object.keys(a).length === Object.keys(b).length && Object.keys(a).every((k) => a[k] === b[k]);

export function sameTarget(a: Target | null, b: Target | null): boolean {
  if (!a || !b) return false;
  return a.route === b.route && a.addr === b.addr && sameValues(a.remote, b.remote) && a.webrtcToken === b.webrtcToken;
}

export function settingsFor(t: Target): {
  connProfile: 'lo' | 'lan' | 'wan'; lanIp?: string; wanIp?: string; webrtcToken: string;
} {
  if (t.route === 'remote') return { connProfile: 'wan', wanIp: '', webrtcToken: '' };
  if (isLoopback(t.addr)) return { connProfile: 'lo', webrtcToken: t.webrtcToken };
  const priv = isPrivateIp(t.addr);
  return {
    connProfile: priv ? 'lan' : 'wan',
    ...(priv ? { lanIp: t.addr } : {}),
    wanIp: priv ? '' : t.addr,
    webrtcToken: t.webrtcToken,
  };
}

export function isLoopback(addr: string): boolean {
  const a = addr.trim();
  return a === 'localhost' || /^127\.\d{1,3}\.\d{1,3}\.\d{1,3}$/.test(a);
}

export function isPrivateIp(addr: string): boolean {
  const m = /^(\d{1,3})\.(\d{1,3})\.(\d{1,3})\.(\d{1,3})$/.exec(addr.trim());
  if (!m) return false;
  const [a, b] = [Number(m[1]), Number(m[2])];
  return a === 10 || a === 127 || (a === 192 && b === 168) || (a === 172 && b >= 16 && b <= 31);
}
