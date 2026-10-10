
export type Route = 'direct' | 'rendezvous';

export type Target = {
  route: Route;
  addr: string;
  robotId: string;
  rendezvousUrl: string;
  webrtcToken: string;
  label: string;
};

export type TargetError = 'no_addr' | 'no_robot_id' | 'no_rendezvous_url';

export function directTarget(addr: string, opts: { token?: string; label?: string } = {}):
  { ok: true; target: Target } | { ok: false; reason: TargetError } {
  const a = addr.trim();
  if (!a) return { ok: false, reason: 'no_addr' };
  return {
    ok: true,
    target: {
      route: 'direct', addr: a, robotId: '', rendezvousUrl: '',
      webrtcToken: (opts.token ?? '').trim(), label: opts.label?.trim() || a,
    },
  };
}

export function rendezvousTarget(
  robotId: string, rendezvousUrl: string, opts: { label?: string } = {},
): { ok: true; target: Target } | { ok: false; reason: TargetError } {
  const id = robotId.trim();
  const url = rendezvousUrl.trim();
  if (!id) return { ok: false, reason: 'no_robot_id' };
  if (!url) return { ok: false, reason: 'no_rendezvous_url' };
  return {
    ok: true,
    target: {
      route: 'rendezvous', addr: id, robotId: id, rendezvousUrl: url,
      webrtcToken: '', label: opts.label?.trim() || id,
    },
  };
}

export function sameTarget(a: Target | null, b: Target | null): boolean {
  if (!a || !b) return false;
  return a.route === b.route && a.addr === b.addr && a.robotId === b.robotId
    && a.rendezvousUrl === b.rendezvousUrl && a.webrtcToken === b.webrtcToken;
}

export function settingsFor(t: Target): {
  connProfile: 'lo' | 'lan' | 'wan'; lanIp?: string; wanIp?: string;
  rendezvousUrl: string; robotId: string; webrtcToken: string;
} {
  if (t.route === 'rendezvous') {
    return {
      connProfile: 'wan', wanIp: '',
      rendezvousUrl: t.rendezvousUrl, robotId: t.robotId, webrtcToken: '',
    };
  }
  if (isLoopback(t.addr)) {
    return { connProfile: 'lo', rendezvousUrl: '', robotId: '', webrtcToken: t.webrtcToken };
  }
  const priv = isPrivateIp(t.addr);
  return {
    connProfile: priv ? 'lan' : 'wan',
    ...(priv ? { lanIp: t.addr } : {}),
    wanIp: priv ? '' : t.addr,
    rendezvousUrl: '', robotId: '', webrtcToken: t.webrtcToken,
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
