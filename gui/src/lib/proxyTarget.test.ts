import { describe, it, expect, vi } from 'vitest';

let demoOn = false;
vi.mock('@/lib/demoFlag', () => ({ isDemo: () => demoOn }));
import { syncProxyTarget } from './proxyTarget.web';

type Call = { url: string; method: string; body?: string; ct?: string };

function fakeFetch(opts: { get?: { robot: string; vision: string } | 'fail'; post?: 'ok' | 'deny' }) {
  const calls: Call[] = [];
  const f = (async (url: unknown, init?: Record<string, any>) => {
    const method = String(init?.method ?? 'GET').toUpperCase();
    calls.push({
      url: String(url),
      method,
      body: init?.body,
      ct: init?.headers?.['content-type'],
    });
    if (method === 'POST') {
      if (opts.post === 'deny') return { ok: false, status: 403, json: async () => ({}) };
      return { ok: true, status: 200, json: async () => ({}) };
    }
    if (opts.get === 'fail' || opts.get === undefined) throw new Error('network');
    return { ok: true, status: 200, json: async () => opts.get };
  }) as unknown as typeof fetch;
  return { f, calls };
}

describe('syncProxyTarget', () => {
  it('현재 대상과 같으면 POST 하지 않는다', async () => {
    const { f, calls } = fakeFetch({ get: { robot: '192.168.0.10', vision: '192.168.0.10' } });
    const r = await syncProxyTarget('192.168.0.10', '192.168.0.10', f);
    expect(r).toBe('unchanged');
    expect(calls.filter((c) => c.method === 'POST')).toHaveLength(0);
  });

  it('대상이 다르면 JSON 으로 POST 한다', async () => {
    const { f, calls } = fakeFetch({ get: { robot: '127.0.0.1', vision: '127.0.0.1' }, post: 'ok' });
    const r = await syncProxyTarget('192.168.0.10', '', f);
    expect(r).toBe('switched');
    const post = calls.find((c) => c.method === 'POST');
    expect(post?.url).toContain('/proxy/target');
    expect(post?.ct).toBe('application/json');
    expect(JSON.parse(post!.body!)).toEqual({ robot: '192.168.0.10', vision: '192.168.0.10' });
  });

  it('visionIp 가 비면 Motion IP 를 상속해 비교한다 — 같으면 POST 없음', async () => {
    const { f, calls } = fakeFetch({ get: { robot: '10.0.0.5', vision: '10.0.0.5' } });
    const r = await syncProxyTarget('10.0.0.5', '', f);
    expect(r).toBe('unchanged');
    expect(calls.filter((c) => c.method === 'POST')).toHaveLength(0);
  });

  it('GET 이 실패해도 throw 하지 않는다 — 연결 흐름을 막지 않는다', async () => {
    const { f } = fakeFetch({ get: 'fail' });
    await expect(syncProxyTarget('192.168.0.10', '', f)).resolves.toBe('failed');
  });

  it('POST 가 403(loopback 아님)이어도 throw 하지 않는다', async () => {
    const { f } = fakeFetch({ get: { robot: '127.0.0.1', vision: '127.0.0.1' }, post: 'deny' });
    await expect(syncProxyTarget('192.168.0.10', '', f)).resolves.toBe('failed');
  });
});

describe('네이티브 짝(proxyTarget.ts)', () => {
  it('no-op — 네트워크를 건드리지 않고 unchanged 를 준다', async () => {
    const native = await import('./proxyTarget');
    let called = 0;
    const spy = (async () => { called++; throw new Error('불려선 안 된다'); }) as unknown as typeof fetch;
    await expect(native.syncProxyTarget('192.168.0.10', '', spy)).resolves.toBe('unchanged');
    expect(called).toBe(0);
  });
});

describe('동시 호출 합치기(in-flight coalescing)', () => {
  it('같은 대상으로 동시에 두 번 불러도 POST 는 한 번만 나간다', async () => {
    const calls: string[] = [];
    const slow = (async (url: unknown, init?: Record<string, any>) => {
      const method = String(init?.method ?? 'GET').toUpperCase();
      calls.push(method);
      await new Promise((r) => setTimeout(r, 30));
      if (method === 'POST') return { ok: true, status: 200, json: async () => ({}) };
      return { ok: true, status: 200, json: async () => ({ robot: '127.0.0.1', vision: '127.0.0.1' }) };
    }) as unknown as typeof fetch;

    const [a, b] = await Promise.all([
      syncProxyTarget('192.168.0.50', '10.9.9.9', slow),
      syncProxyTarget('192.168.0.50', '10.9.9.9', slow),
    ]);
    expect(calls.filter((m) => m === 'POST')).toHaveLength(1);
    expect([a, b]).toEqual(['switched', 'switched']);
  });
});

describe('데모 모드', () => {
  it('데모에서는 네트워크를 아예 건드리지 않는다', async () => {
    demoOn = true;
    let calls = 0;
    const spy = (async () => { calls++; throw new Error('불려선 안 된다'); }) as unknown as typeof fetch;
    const r = await syncProxyTarget('demo', 'demo', spy);
    demoOn = false;
    expect(calls).toBe(0);
    expect(r).toBe('unchanged');
  });
});

describe('나중 요청이 이긴다(superseded 무효화)', () => {
  it('대상이 다른 요청이 겹치면 먼저 시작한 쪽은 POST 하지 않는다', async () => {
    const posted: string[] = [];
    const mk = (delay: number) => (async (url: unknown, init?: Record<string, any>) => {
      const method = String(init?.method ?? 'GET').toUpperCase();
      await new Promise((r) => setTimeout(r, delay));
      if (method === 'POST') { posted.push(JSON.parse(init!.body).robot); return { ok: true, status: 200, json: async () => ({}) }; }
      return { ok: true, status: 200, json: async () => ({ robot: '127.0.0.1', vision: '127.0.0.1' }) };
    }) as unknown as typeof fetch;

    const slow = syncProxyTarget('10.0.0.1', '', mk(60));
    await new Promise((r) => setTimeout(r, 5));
    const fast = syncProxyTarget('10.0.0.2', '', mk(5));
    await Promise.all([slow, fast]);

    expect(posted).toEqual(['10.0.0.2']);
  });
});

describe('대상 검증', () => {
  it("데모 센티넬 'demo' 는 데모가 꺼진 뒤에도 거부한다", async () => {
    let calls = 0;
    const spy = (async () => { calls++; throw new Error('불려선 안 된다'); }) as unknown as typeof fetch;
    await expect(syncProxyTarget('demo', 'demo', spy)).resolves.toBe('failed');
    expect(calls).toBe(0);
  });

  it('스킴·포트·경로가 섞인 값은 거부한다(프록시 host() 규칙 미러)', async () => {
    let calls = 0;
    const spy = (async () => { calls++; throw new Error('불려선 안 된다'); }) as unknown as typeof fetch;
    for (const bad of ['http://1.2.3.4', '1.2.3.4:8080', 'a/b', 'user@host', '', '  ']) {
      await expect(syncProxyTarget(bad, '', spy)).resolves.toBe('failed');
    }
    expect(calls).toBe(0);
  });

  it('점 없는 호스트명은 허용한다(--robot r-pc 구성)', async () => {
    const seen: string[] = [];
    const f = (async (_u: unknown, init?: Record<string, any>) => {
      const m = String(init?.method ?? 'GET').toUpperCase();
      if (m === 'POST') { seen.push(JSON.parse(init!.body).robot); return { ok: true, status: 200, json: async () => ({}) }; }
      return { ok: true, status: 200, json: async () => ({ robot: '127.0.0.1', vision: '127.0.0.1' }) };
    }) as unknown as typeof fetch;
    await expect(syncProxyTarget('r-pc', '', f)).resolves.toBe('switched');
    expect(seen).toEqual(['r-pc']);
  });
});

describe('세대 관리', () => {
  it('key1 → key2 → key1 로 겹쳐도 옛 요청은 POST 하지 않는다', async () => {
    const posted: string[] = [];
    const mk = (delay: number) => (async (_u: unknown, init?: Record<string, any>) => {
      const m = String(init?.method ?? 'GET').toUpperCase();
      await new Promise((r) => setTimeout(r, delay));
      if (m === 'POST') { posted.push(JSON.parse(init!.body).robot); return { ok: true, status: 200, json: async () => ({}) }; }
      return { ok: true, status: 200, json: async () => ({ robot: '127.0.0.1', vision: '127.0.0.1' }) };
    }) as unknown as typeof fetch;
    const a = syncProxyTarget('10.0.0.1', '', mk(80));
    await new Promise((r) => setTimeout(r, 5));
    const b = syncProxyTarget('10.0.0.2', '', mk(80));
    await new Promise((r) => setTimeout(r, 5));
    const c = syncProxyTarget('10.0.0.1', '', mk(5));
    await Promise.all([a, b, c]);
    expect(posted).toEqual(['10.0.0.1']);
  });

  it('무효화된 요청은 failed 를 준다 — 호출부가 성공으로 오인해 커밋하지 않도록', async () => {
    const mk = (delay: number) => (async (_u: unknown, init?: Record<string, any>) => {
      const m = String(init?.method ?? 'GET').toUpperCase();
      await new Promise((r) => setTimeout(r, delay));
      if (m === 'POST') return { ok: true, status: 200, json: async () => ({}) };
      return { ok: true, status: 200, json: async () => ({ robot: '127.0.0.1', vision: '127.0.0.1' }) };
    }) as unknown as typeof fetch;
    const old = syncProxyTarget('10.0.0.8', '', mk(60));
    await new Promise((r) => setTimeout(r, 5));
    const fresh = syncProxyTarget('10.0.0.9', '', mk(5));
    expect(await old).toBe('failed');
    expect(await fresh).toBe('switched');
  });
});
