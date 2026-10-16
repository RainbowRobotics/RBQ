
export const CACHE_TTL_MS = 24 * 60 * 60 * 1000;

export type RemoteAccount = {
  accountId: string;
  accountName: string;
  level: 1 | 2 | 3;
  robotSerial: string | null;
  robotName: string | null;
  lanIp: string | null;
  wanIp: string | null;
  rendezvousUrl: string | null;
  robotId: string | null;
  webrtcToken: string | null;
  expiresAt: string | null;
};

export type RemoteLoginCache = { account: RemoteAccount; at: number; accounts?: RemoteAccount[]; pin?: string };

export type LoginOutcome =
  | { ok: true; account: RemoteAccount; accounts: RemoteAccount[]; fromCache: boolean }
  | { ok: false; reason: 'bad_pin' | 'offline' | 'server' | 'throttled' };

const isPin = (s: string) => /^[0-9]{4}$/.test(s.trim());

export function rowToAccount(r: Record<string, unknown>): RemoteAccount {
  const lv = Number(r.level);
  const s = (v: unknown) => (typeof v === 'string' && v !== '' ? v : null);
  return {
    accountId: String(r.account_id ?? ''),
    accountName: String(r.account_name ?? ''),
    level: (lv === 3 ? 3 : lv === 2 ? 2 : 1),
    robotSerial: s(r.robot_serial),
    robotName: s(r.robot_name),
    lanIp: s(r.lan_ip),
    wanIp: s(r.wan_ip),
    rendezvousUrl: s(r.rendezvous_url),
    robotId: s(r.robot_id),
    webrtcToken: s(r.webrtc_token),
    expiresAt: s(r.expires_at),
  };
}

export function isCacheValid(c: RemoteLoginCache | null, now: number, ttl = CACHE_TTL_MS): boolean {
  return !!c && now - c.at < ttl && now >= c.at;
}

export function isExpired(a: Pick<RemoteAccount, 'expiresAt'>, now: number): boolean {
  if (!a.expiresAt) return false;
  const t = Date.parse(a.expiresAt);
  return Number.isFinite(t) && now > t;
}


type Deps = {
  url: string;
  anonKey: string;
  fetchFn?: typeof fetch;
  now?: () => number;
  cache?: RemoteLoginCache | null;
};

export async function loginWithPin(pin: string, deps: Deps): Promise<LoginOutcome> {
  const { url, anonKey, fetchFn = fetch, now = Date.now, cache = null } = deps;
  if (!isPin(pin)) return { ok: false, reason: 'bad_pin' };

  try {
    const res = await fetchFn(`${url.replace(/\/$/, '')}/rest/v1/rpc/login_with_pin`, {
      method: 'POST',
      headers: { apikey: anonKey, Authorization: `Bearer ${anonKey}`, 'Content-Type': 'application/json' },
      body: JSON.stringify({ p_pin: pin.trim() }),
    });
    if (res.status === 429) return { ok: false, reason: 'throttled' };
    if (!res.ok) return { ok: false, reason: 'server' };
    const rows = (await res.json()) as Record<string, unknown>[];
    if (!Array.isArray(rows) || rows.length === 0) return { ok: false, reason: 'bad_pin' };
    const accounts = rows.map(rowToAccount);
    return { ok: true, account: accounts[0], accounts, fromCache: false };
  } catch {
    if (isCacheValid(cache, now())) {
      const c = cache!;
      return { ok: true, account: c.account, accounts: c.accounts?.length ? c.accounts : [c.account], fromCache: true };
    }
    return { ok: false, reason: 'offline' };
  }
}

export async function reportConnected(pin: string, serial: string, deps: Deps): Promise<boolean> {
  const { url, anonKey, fetchFn = fetch } = deps;
  try {
    const res = await fetchFn(`${url.replace(/\/$/, '')}/rest/v1/rpc/report_connected`, {
      method: 'POST',
      headers: { apikey: anonKey, Authorization: `Bearer ${anonKey}`, 'Content-Type': 'application/json' },
      body: JSON.stringify({ p_pin: pin.trim(), p_serial: serial }),
    });
    return res.ok;
  } catch {
    return false;
  }
}

export async function registerRobot(
  pin: string,
  robot: { serial: string; name: string; lanIp: string; robotId: string; rendezvousUrl: string },
  deps: Deps,
): Promise<{ ok: true; rendezvousUrl: string; rendezvousToken: string } | { ok: false; reason: 'denied' | 'server' | 'offline' }> {
  const { url, anonKey, fetchFn = fetch } = deps;
  try {
    const res = await fetchFn(`${url.replace(/\/$/, '')}/rest/v1/rpc/register_robot`, {
      method: 'POST',
      headers: { apikey: anonKey, Authorization: `Bearer ${anonKey}`, 'Content-Type': 'application/json' },
      body: JSON.stringify({
        p_pin: pin.trim(),
        p_serial: robot.serial,
        p_name: robot.name,
        p_lan_ip: robot.lanIp,
        p_robot_id: robot.robotId,
        p_rendezvous_url: robot.rendezvousUrl,
      }),
    });
    if (res.ok) {
      const rows = (await res.json()) as {
        assigned_rendezvous_url?: string; assigned_rendezvous_token?: string;
      }[];
      return {
        ok: true,
        rendezvousUrl: rows?.[0]?.assigned_rendezvous_url ?? '',
        rendezvousToken: rows?.[0]?.assigned_rendezvous_token ?? '',
      };
    }
    return { ok: false, reason: res.status === 403 || res.status === 401 ? 'denied' : 'server' };
  } catch {
    return { ok: false, reason: 'offline' };
  }
}

export type Ticket = { ticket: string; robotId: string; expiresAt: string };

export async function issueTicket(
  pin: string,
  serial: string,
  clientId: string,
  deps: Deps,
): Promise<{ ok: true; ticket: Ticket } | { ok: false; reason: 'denied' | 'not_ready' | 'server' | 'offline' }> {
  const { url, anonKey, fetchFn = fetch } = deps;
  try {
    const res = await fetchFn(`${url.replace(/\/$/, '')}/rest/v1/rpc/issue_ticket`, {
      method: 'POST',
      headers: { apikey: anonKey, Authorization: `Bearer ${anonKey}`, 'Content-Type': 'application/json' },
      body: JSON.stringify({ p_pin: pin.trim(), p_serial: serial, p_client_id: clientId }),
    });
    if (res.status === 401 || res.status === 403) return { ok: false, reason: 'denied' };
    if (!res.ok) {
      const body = await res.text().catch(() => '');
      return { ok: false, reason: body.includes('22023') ? 'not_ready' : 'server' };
    }
    const rows = (await res.json()) as { ticket?: string; robot_id?: string; expires_at?: string }[];
    const r = rows?.[0];
    if (!r?.ticket) return { ok: false, reason: 'server' };
    return { ok: true, ticket: { ticket: r.ticket, robotId: r.robot_id ?? '', expiresAt: r.expires_at ?? '' } };
  } catch {
    return { ok: false, reason: 'offline' };
  }
}

export function pickAccount(accounts: RemoteAccount[], serial: string | null | undefined): RemoteAccount | null {
  if (accounts.length === 0) return null;
  return accounts.find((a) => a.robotSerial && a.robotSerial === serial) ?? accounts[0];
}
