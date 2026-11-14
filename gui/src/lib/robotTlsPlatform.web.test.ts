import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';
import { robotTlsPlatform as p } from './robotTlsPlatform.web';

let posts: unknown[];
let hold: Promise<void> | null;
const ok = (body?: unknown) => ({ ok: true, status: 200, json: async () => body });

beforeEach(() => {
  posts = []; hold = null;
  vi.stubGlobal('fetch', vi.fn(async (url: string, init?: { body?: string; headers?: Record<string, string> }) => {
    if (url.startsWith('/proxy/tls-probe')) return ok({ kind: 'tls', fp: 'F', serial: 'S' });
    expect(init?.headers?.['content-type']).toBe('application/json');
    if (hold) await hold;
    posts.push(JSON.parse(init!.body!));
    return { ok: true, status: 204 };
  }));
});
afterEach(() => { p.allowPlain(); vi.unstubAllGlobals(); });

describe('robotTlsPlatform.web', () => {
  it('probe_asks_proxy_and_down_on_failure', async () => {
    expect(await p.probe('10.0.0.1', 8080)).toEqual({ kind: 'tls', fp: 'F', serial: 'S' });
    expect(vi.mocked(fetch)).toHaveBeenCalledWith('/proxy/tls-probe?host=10.0.0.1&port=8080');
    vi.mocked(fetch).mockResolvedValueOnce({ ok: false, status: 403 } as Response);
    expect(await p.probe('10.0.0.1', 8080)).toEqual({ kind: 'down' });
    vi.mocked(fetch).mockRejectedValueOnce(new TypeError('proxy gone'));
    expect(await p.probe('10.0.0.1', 8080)).toEqual({ kind: 'down' });
  });

  it('engage_block_set_proxy_routes_and_kept', async () => {
    await p.engage('10.0.0.2', 8080, 'F');
    await p.engage('10.0.0.2', 8081, null);
    p.block('10.0.0.2', 8082);
    await vi.waitFor(() => expect(posts.length).toBe(3));
    expect(posts).toEqual([
      { host: '10.0.0.2', port: 8080, fp: 'F' }, { host: '10.0.0.2', port: 8081, fp: null }, { host: '10.0.0.2', port: 8082, fp: 'blocked' },
    ]);
    expect([8080, 8081, 8082].map((port) => p.kept('10.0.0.2', port))).toEqual([true, false, true]);
    expect([8080, 8081, 8082].map((port) => p.pinnedFp('10.0.0.2', port))).toEqual(['F', undefined, undefined]);
    expect(p.kept('10.0.0.3', 8080)).toBe(false);
    expect(p.base('10.0.0.2', 8080)).toBeUndefined();
  });

  it('routes_reach_proxy_in_call_order', async () => {
    let release!: () => void;
    hold = new Promise((r) => { release = r; });
    const e = p.engage('10.0.0.4', 8080, 'F');
    p.block('10.0.0.4', 8080);
    await Promise.resolve();
    expect(vi.mocked(fetch).mock.calls.length).toBe(1);
    hold = null; release();
    await e;
    await vi.waitFor(() => expect(posts.length).toBe(2));
    expect(posts.map((x) => (x as { fp: unknown }).fp)).toEqual(['F', 'blocked']);
    expect(p.kept('10.0.0.4', 8080)).toBe(true);
    expect(p.pinnedFp('10.0.0.4', 8080)).toBeUndefined();
  });

  it('route_recorded_only_after_proxy_ok', async () => {
    let release!: () => void;
    hold = new Promise((r) => { release = r; });
    const e = p.engage('10.0.0.8', 8080, 'F');
    await Promise.resolve();
    expect(p.kept('10.0.0.8', 8080)).toBe(false);
    expect(p.pinnedFp('10.0.0.8', 8080)).toBeUndefined();
    hold = null; release();
    await e;
    expect(p.pinnedFp('10.0.0.8', 8080)).toBe('F');
    vi.mocked(fetch).mockResolvedValueOnce({ ok: false, status: 403 } as Response);
    await expect(p.engage('10.0.0.8', 8081, 'F')).rejects.toThrow('tls-route 403');
    expect(p.kept('10.0.0.8', 8081)).toBe(true);
    expect(p.pinnedFp('10.0.0.8', 8081)).toBeUndefined();
    vi.mocked(fetch).mockRejectedValueOnce(new TypeError('proxy gone'));
    await expect(p.engage('10.0.0.8', 8082, null)).rejects.toThrow('proxy gone');
    expect(p.kept('10.0.0.8', 8082)).toBe(true);
  });

  it('engage_throws_when_proxy_refuses', async () => {
    vi.mocked(fetch).mockResolvedValueOnce({ ok: false, status: 403 } as Response);
    await expect(p.engage('10.0.0.5', 8080, 'F')).rejects.toThrow('tls-route 403');
    await p.engage('10.0.0.5', 8081, 'F');
    expect(posts).toEqual([{ host: '10.0.0.5', port: 8081, fp: 'F' }]);
  });

  it('forget_plain_latches_until_allow', async () => {
    await p.engage('10.0.0.6', 8080, null);
    p.forgetPlain();
    expect(p.plainHeld()).toBe(true);
    await p.engage('10.0.0.6', 8080, null);
    expect(p.kept('10.0.0.6', 8080)).toBe(true);
    p.allowPlain();
    await p.engage('10.0.0.6', 8081, null);
    expect(posts).toEqual([
      { host: '10.0.0.6', port: 8080, fp: null }, { forget: 'plain' },
      { host: '10.0.0.6', port: 8080, fp: 'blocked' }, { host: '10.0.0.6', port: 8081, fp: null },
    ]);
    expect(p.plainHeld()).toBe(false);
  });

  it('forget_plain_drops_plain_from_kept_view', async () => {
    await p.engage('10.0.0.7', 8080, null);
    await p.engage('10.0.0.7', 8081, 'F');
    p.forgetPlain();
    expect(p.kept('10.0.0.7', 8080)).toBe(false);
    expect(p.kept('10.0.0.7', 8081)).toBe(true);
  });
});
