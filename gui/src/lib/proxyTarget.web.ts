import { isDemo } from '@/lib/demoFlag';
export type ProxyTargetSync = 'unchanged' | 'switched' | 'failed';

const TARGET_PATH = '/proxy/target';

let inflight: { key: string; p: Promise<ProxyTargetSync> } | null = null;

let latestGen = 0;

const DEMO_SENTINEL = 'demo';
function sendableHost(v: string): boolean {
  return v !== '' && v !== DEMO_SENTINEL && !/[\s/:@]/.test(v);
}

export async function syncProxyTarget(
  robot: string,
  vision: string,
  f: typeof fetch = fetch,
): Promise<ProxyTargetSync> {
  if (isDemo()) return 'unchanged';
  const wantRobot = (robot ?? '').trim();
  const wantVision = (vision ?? '').trim() || wantRobot;
  if (!sendableHost(wantRobot) || !sendableHost(wantVision)) return 'failed';
  const key = `${wantRobot}|${wantVision}`;
  if (inflight?.key === key) return inflight.p;
  const myGen = ++latestGen;

  const run = (async (): Promise<ProxyTargetSync> => {
    try {
      const cur = await f(TARGET_PATH);
      if (!cur.ok) return 'failed';
      const now = (await cur.json()) as { robot?: string; vision?: string };
      if (now?.robot === wantRobot && now?.vision === wantVision) return 'unchanged';
      if (myGen !== latestGen) return 'failed';
      const res = await f(TARGET_PATH, {
        method: 'POST',
        headers: { 'content-type': 'application/json' },
        body: JSON.stringify({ robot: wantRobot, vision: wantVision }),
      });
      return res.ok ? 'switched' : 'failed';
    } catch {
      return 'failed';
    }
  })();

  inflight = { key, p: run };
  try {
    return await run;
  } finally {
    if (inflight?.p === run) inflight = null;
  }
}
