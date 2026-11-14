import { robotAuth, reportAuthFailure } from './auth';
import { isPinnedSerial, reportTlsIssue } from './robotTls';
import { robotTlsPlatform } from './robotTlsPlatform';

const LOCAL_SIM = '127.0.0.1';

function authRequired(target: string, sent: Record<string, string>, status: number | undefined, j?: { auth?: boolean }) {
  if (status === 401 || j?.auth) reportAuthFailure(target, true, sent);
}

export const ROBOT_LAN_IP = '192.168.0.10';

async function nativeSerial(ip: string, fetchFn: typeof fetch, timeoutMs: number, tlsOnly = false): Promise<string | null> {
  let timer: ReturnType<typeof setTimeout> | undefined;
  try {
    const probe = await robotTlsPlatform.probe(ip, 8080);
    if (probe.kind === 'tls') return probe.serial;
    if (probe.kind === 'down' || tlsOnly) return null;
    const ctl = new AbortController();
    timer = setTimeout(() => ctl.abort(), timeoutMs);
    const h = robotAuth(ip);
    const r = await fetchFn(`http://${ip}:8080/api/robot/serial_number`, { headers: h, signal: ctl.signal });
    authRequired(ip, h, r.status);
    if (!r.ok) return null;
    const j = (await r.json()) as { serial_number?: unknown };
    if (typeof j.serial_number !== 'string') return null;
    const serial = j.serial_number.trim();
    if (serial && isPinnedSerial(serial)) { reportTlsIssue({ kind: 'refuse-old', entryKey: serial }); return null; }
    return serial;
  } catch { return null; } finally { clearTimeout(timer); }
}

export async function probeLan(ip: string, opts: { fetchFn?: typeof fetch; timeoutMs?: number; base?: string; tlsOnly?: boolean } = {}): Promise<string | null> {
  const { fetchFn = fetch, timeoutMs = 1200, base = `http://${ip}:8080`, tlsOnly = false } = opts;
  if (typeof location !== 'undefined' && /[?&]nolan=1/.test(location.search)) return null;
  if (base === '') {
    const r = await probeAddress(ip, true, fetchFn, tlsOnly);
    return r ? r.serial : null;
  }
  return nativeSerial(ip, fetchFn, timeoutMs, tlsOnly);
}

export function probeRobotLan(opts?: { fetchFn?: typeof fetch; timeoutMs?: number; base?: string }): Promise<string | null> {
  return probeLan(ROBOT_LAN_IP, opts);
}

export const LAN_RETRY_MS = 3000;

export function retryLanProbe(o: { probe: () => Promise<string | null>; canTry: () => boolean; onFound: () => void; everyMs?: number }): () => void {
  let stopped = false;
  let busy = false;
  const timer = setInterval(async () => {
    if (stopped || busy || !o.canTry()) return;
    busy = true;
    try {
      const found = await o.probe();
      if (!stopped && found !== null && o.canTry()) o.onFound();
    } finally { busy = false; }
  }, o.everyMs ?? LAN_RETRY_MS);
  return () => { stopped = true; clearInterval(timer); };
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

export async function probeAddress(ip: string, web: boolean, fetchFn: typeof fetch = fetch, tlsOnly = false): Promise<{ serial: string } | null> {
  if (web) {
    try {
      const h = robotAuth(ip);
      const r = await fetchFn(`/probe-robot?ip=${encodeURIComponent(ip)}${tlsOnly ? '&tlsOnly=1' : ''}`, { headers: h });
      if (!r.ok) return null;
      const j = (await r.json()) as { ok?: boolean; serial?: string; auth?: boolean; tls?: string | null };
      authRequired(ip, h, undefined, j);
      if (!j.ok) return null;
      const serial = (j.serial ?? '').trim();
      if (!j.tls && serial && isPinnedSerial(serial)) { reportTlsIssue({ kind: 'refuse-old', entryKey: serial }); return null; }
      return { serial };
    } catch { return null; }
  }
  const serial = await nativeSerial(ip, fetchFn, 1500);
  return serial === null ? null : { serial };
}
