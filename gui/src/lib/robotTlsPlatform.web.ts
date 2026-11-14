import type { Probe, RobotTlsPlatform } from './robotTls';

type Route = string | null | 'blocked';
const routes = new Map<string, Route>();
const key = (host: string, port: number) => `${host}:${port}`;
let plainHold = false;
let chain: Promise<unknown> = Promise.resolve();

function post(body: object, route?: [string, Route]): Promise<void> {
  const p = chain
    .then(() => fetch('/proxy/tls-route', { method: 'POST', headers: { 'content-type': 'application/json' }, body: JSON.stringify(body) }))
    .then((r) => { if (!r.ok) throw new Error(`tls-route ${r.status}`); })
    .then(
      () => { if (route) routes.set(route[0], route[1]); },
      (e) => { if (route) routes.set(route[0], 'blocked'); throw e; },
    );
  chain = p.catch(() => {});
  return p;
}

export const robotTlsPlatform: RobotTlsPlatform = {
  async probe(host, port): Promise<Probe> {
    try {
      const r = await fetch(`/proxy/tls-probe?host=${encodeURIComponent(host)}&port=${port}`);
      return r.ok ? ((await r.json()) as Probe) : { kind: 'down' };
    } catch { return { kind: 'down' }; }
  },

  engage(host, port, fp) {
    const v: Route = fp === null && plainHold ? 'blocked' : fp;
    return post({ host, port, fp: v }, [key(host, port), v]);
  },

  block(host, port) {
    const k = key(host, port);
    routes.set(k, 'blocked');
    post({ host, port, fp: 'blocked' }, [k, 'blocked']).catch(() => {});
  },

  kept(host, port) {
    const r = routes.get(key(host, port));
    return r !== undefined && r !== null;
  },

  pinnedFp(host, port) {
    const r = routes.get(key(host, port));
    return r && r !== 'blocked' ? r : undefined;
  },

  base: () => undefined,

  forgetPlain() {
    plainHold = true;
    for (const [k, r] of routes) if (r === null) routes.delete(k);
    post({ forget: 'plain' }).catch(() => {});
  },

  allowPlain() { plainHold = false; },
  plainHeld: () => plainHold,
};
