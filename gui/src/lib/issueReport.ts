import { useEffect, useState } from 'react';
import { Platform } from 'react-native';
import { currentAppVersion } from '@/lib/appSelfUpdate';

const REVIEW_URL = (process.env.EXPO_PUBLIC_REVIEW_URL || '').replace(/\/+$/, '');
export const REVIEW_BASE = !REVIEW_URL ? '' : Platform.OS === 'web' ? '/issue' : REVIEW_URL;
const reviewHeaders: Record<string, string> = Platform.OS === 'web' && REVIEW_URL ? { 'X-Review-Target': REVIEW_URL } : {};

const HEALTH_TIMEOUT_MS = 2500;
const HEALTH_POLL_MS = 30_000;

function fetchTimeout(url: string, ms: number, init?: RequestInit): Promise<Response> {
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), ms);
  return fetch(url, { ...init, signal: ctrl.signal }).finally(() => clearTimeout(timer));
}

export async function reviewServerReachable(base = REVIEW_BASE): Promise<boolean> {
  if (!base) return false;
  try {
    const r = await fetchTimeout(`${base}/health`, HEALTH_TIMEOUT_MS, { headers: reviewHeaders });
    return r.ok;
  } catch { return false; }
}

export function useReviewServer(enabled: boolean, base = REVIEW_BASE): boolean {
  const [ok, setOk] = useState(false);
  useEffect(() => {
    if (!enabled) { setOk(false); return; }
    let alive = true;
    const check = () => reviewServerReachable(base).then((v) => { if (alive) setOk(v); });
    check();
    const t = setInterval(check, HEALTH_POLL_MS);
    return () => { alive = false; clearInterval(t); };
  }, [enabled, base]);
  return ok;
}

export type IssueReportMeta = {
  appVersion: string; platform: string; robotIp: string; robotSerial?: string; screen?: string;
};

export function buildMeta(robotIp: string, extra?: Partial<IssueReportMeta>): IssueReportMeta {
  return { appVersion: currentAppVersion(), platform: Platform.OS, robotIp: robotIp || '-', ...extra };
}

export async function sendIssueReport(
  { comment, meta, shotB64, base = REVIEW_BASE }: {
    comment: string; meta: IssueReportMeta; shotB64?: string | null; base?: string;
  },
): Promise<{ id: string }> {
  const r = await fetchTimeout(`${base}/report`, 15_000, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json', ...reviewHeaders },
    body: JSON.stringify({ meta, comment, shot_b64: shotB64 ?? null }),
  });
  if (!r.ok) throw new Error(`HTTP ${r.status}`);
  return r.json();
}
