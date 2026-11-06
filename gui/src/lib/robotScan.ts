import { robotAuth, reportAuthFailure } from './auth';

const LOCAL_SIM = '127.0.0.1';

function authRequired(target: string, sent: Record<string, string>, status: number | undefined, j?: { auth?: boolean }) {
  if (status === 401 || j?.auth) reportAuthFailure(target, true, sent);
}

export const ROBOT_LAN_IP = '192.168.0.10';

export async function probeLan(ip: string, opts: { fetchFn?: typeof fetch; timeoutMs?: number; base?: string } = {}): Promise<string | null> {
  const { fetchFn = fetch, timeoutMs = 1200, base = `http://${ip}:8080` } = opts;
  if (typeof location !== 'undefined' && /[?&]nolan=1/.test(location.search)) return null;
  if (base === '') {
    const r = await probeAddress(ip, true, fetchFn);
    return r ? r.serial : null;
  }
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), timeoutMs);
  try {
    const h = robotAuth(ip);
    const r = await fetchFn(`${base}/api/robot/serial_number`, { headers: h, signal: ctl.signal });
    authRequired(ip, h, r.status);
    if (!r.ok) return null;
    const j = (await r.json()) as { serial_number?: unknown };
    return typeof j.serial_number === 'string' ? j.serial_number.trim() : null;
  } catch { return null; } finally { clearTimeout(timer); }
}

export function probeRobotLan(opts?: { fetchFn?: typeof fetch; timeoutMs?: number; base?: string }): Promise<string | null> {
  return probeLan(ROBOT_LAN_IP, opts);
}

export async function probeLocalSim(fetchFn: typeof fetch = fetch): Promise<boolean> {
  try {
    const h = robotAuth(LOCAL_SIM);
    const r = await fetchFn('/local-sim', { headers: h });
    if (!r.ok) return false;
    const j = (await r.json()) as { ok?: boolean; auth?: boolean };
    authRequired(LOCAL_SIM, h, undefined, j);
    return !!j.ok;
  } catch { return false; }
}

export async function probeAddress(ip: string, web: boolean, fetchFn: typeof fetch = fetch): Promise<{ serial: string } | null> {
  if (web) {
    try {
      const h = robotAuth(ip);
      const r = await fetchFn(`/probe-robot?ip=${encodeURIComponent(ip)}`, { headers: h });
      if (!r.ok) return null;
      const j = (await r.json()) as { ok?: boolean; serial?: string; auth?: boolean };
      authRequired(ip, h, undefined, j);
      return j.ok ? { serial: (j.serial ?? '').trim() } : null;
    } catch { return null; }
  }
  const ctl = new AbortController();
  const timer = setTimeout(() => ctl.abort(), 1500);
  try {
    const h = robotAuth(ip);
    const r = await fetchFn(`http://${ip}:8080/api/robot/serial_number`, { headers: h, signal: ctl.signal });
    authRequired(ip, h, r.status);
    if (!r.ok) return null;
    const j = (await r.json()) as { serial_number?: unknown };
    return typeof j.serial_number === 'string' ? { serial: j.serial_number.trim() } : null;
  } catch { return null; } finally { clearTimeout(timer); }
}
