import RobotTls from '../../modules/robot-tls/src/RobotTlsModule';
import type { RobotTlsPlatform } from './robotTls';

type Route = { fp: string; local: number } | 'plain' | 'block';
const routes = new Map<string, Route>();
const last = new Map<string, { n: number; relay: boolean }>();
const key = (host: string, port: number) => `${host}:${port}`;
let plainHold = false;

function mark(k: string, relay: boolean) {
  const n = (last.get(k)?.n ?? 0) + 1;
  last.set(k, { n, relay });
  return n;
}

export const robotTlsPlatform: RobotTlsPlatform = {
  probe: (host, port) => RobotTls.probe(host, port),

  async engage(host, port, fp) {
    const k = key(host, port);
    const n = mark(k, fp !== null);
    if (fp === null) {
      RobotTls.close(host, port);
      routes.set(k, plainHold ? 'block' : 'plain');
      return;
    }
    try {
      const local = await RobotTls.relay(host, port, fp);
      const now = last.get(k)!;
      if (now.n === n) routes.set(k, { fp, local });
      else if (!now.relay) RobotTls.close(host, port);
    } catch (e) {
      if (last.get(k)!.n === n) routes.set(k, 'block');
      throw e;
    }
  },

  block(host, port) {
    const k = key(host, port);
    mark(k, false);
    RobotTls.close(host, port);
    routes.set(k, 'block');
  },

  kept(host, port) {
    const r = routes.get(key(host, port));
    return r !== undefined && r !== 'plain';
  },

  pinnedFp(host, port) {
    const r = routes.get(key(host, port));
    return typeof r === 'object' ? r.fp : undefined;
  },

  base(host, port) {
    const r = routes.get(key(host, port));
    if (r === undefined) return undefined;
    if (r === 'plain') return `http://${host}:${port}`;
    if (r === 'block') return `https://${host}:${port}`;
    return `http://127.0.0.1:${r.local}`;
  },

  forgetPlain() {
    plainHold = true;
    for (const [k, r] of routes) if (r === 'plain') routes.delete(k);
  },

  allowPlain() { plainHold = false; },
  plainHeld: () => plainHold,
};
