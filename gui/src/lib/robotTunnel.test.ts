import { describe, expect, it, vi } from 'vitest';
import { attachPortTunnel, attachTunnel, STREAM_IDLE_MS, targetOf, tunnelDownload, tunnelFetch, tunnelReady, TunnelWebSocket } from './robotTunnel';

function fakeRobot(respond: (req: string) => Uint8Array) {
  const sent: string[] = [];
  const dc: any = { readyState: 'open', onmessage: null };
  let reqBuf = '';
  dc.send = (d: string | ArrayBuffer) => {
    if (typeof d === 'string') {
      const m = JSON.parse(d);
      sent.push(d);
      if (m.t === 'open') queueMicrotask(() => dc.onmessage({ data: JSON.stringify({ t: 'opened', id: m.id }) }));
      return;
    }
    const b = new Uint8Array(d);
    const id = (b[0] << 24) | (b[1] << 16) | (b[2] << 8) | b[3];
    reqBuf += new TextDecoder().decode(b.subarray(4));
    const end = reqBuf.indexOf('\r\n\r\n');
    const len = Number(/content-length: (\d+)/i.exec(reqBuf)?.[1] ?? 0);
    if (end < 0 || reqBuf.length < end + 4 + len) return;
    const out = respond(reqBuf);
    reqBuf = '';
    queueMicrotask(() => {
      for (let o = 0; o < out.length; o += 16384) {
        const part = out.subarray(o, o + 16384);
        const f = new Uint8Array(4 + part.length);
        f[3] = id; f.set(part, 4);
        dc.onmessage({ data: f.buffer });
      }
      dc.onmessage({ data: JSON.stringify({ t: 'close', id }) });
    });
  };
  attachTunnel(dc);
  dc.onmessage({ data: JSON.stringify({ t: 'hello', v: 1 }) });
  return { dc, sent };
}

const http = (status: number, body: Uint8Array | string, extra = '') => {
  const b = typeof body === 'string' ? new TextEncoder().encode(body) : body;
  const h = new TextEncoder().encode(`HTTP/1.1 ${status} X\r\nContent-Type: application/json\r\nContent-Length: ${b.length}\r\n${extra}Connection: close\r\n\r\n`);
  const o = new Uint8Array(h.length + b.length); o.set(h); o.set(b, h.length); return o;
};

describe('robotTunnel', () => {
  it('URL → 로봇 포트·경로 (네이티브·웹 둘 다)', () => {
    expect(targetOf('http://192.168.0.10:8080/api/version')).toEqual({ port: 8080, path: '/api/version' });
    expect(targetOf('http://10.0.0.2:8081/api/x?a=1')).toEqual({ port: 8081, path: '/api/x?a=1' });
    expect(targetOf('/api/version')).toEqual({ port: 8080, path: '/api/version' });
    expect(targetOf('/streamer/api/snapshot')).toEqual({ port: 8081, path: '/api/snapshot' });
    expect(targetOf('/slamnav/api/map/cloud')).toEqual({ port: 8082, path: '/api/map/cloud' });
  });

  it('hello 전엔 준비 안 됨(옛 로봇)', () => {
    attachTunnel({ readyState: 'open', send: () => {}, onmessage: null } as any);
    expect(tunnelReady()).toBe(false);
  });

  it('GET — 직결과 같은 요청 줄·인증 헤더를 보내고 JSON 을 받는다', async () => {
    let seen = '';
    const { sent } = fakeRobot((req) => { seen = req; return http(200, '{"ok":true}'); });
    const res = await tunnelFetch('http://192.168.0.10:8080/api/version', { headers: { Authorization: 'Basic abc' } });
    expect(JSON.parse(sent[0])).toMatchObject({ t: 'open', port: 8080 });
    expect(seen.startsWith('GET /api/version HTTP/1.1\r\n')).toBe(true);
    expect(seen).toMatch(/authorization: Basic abc\r\n/i);
    expect(res.status).toBe(200);
    expect(await res.json()).toEqual({ ok: true });
  });

  it('큰 바이너리(여러 청크)와 POST 본문이 바이트 그대로', async () => {
    const big = new Uint8Array(100_000).map((_, i) => i % 251);
    let seen = '';
    fakeRobot((req) => { seen = req; return http(200, big); });
    const res = await tunnelFetch('/api/blackbox/file?path=a', { method: 'POST', body: '{"x":1}' });
    expect(seen).toMatch(/^POST \/api\/blackbox\/file\?path=a HTTP\/1.1/);
    expect(seen.endsWith('\r\n\r\n{"x":1}')).toBe(true);
    expect(new Uint8Array(await res.arrayBuffer())).toEqual(big);
  });

  it('에러 상태도 그대로 — 401 은 호출부가 직결 때처럼 처리', async () => {
    fakeRobot(() => http(401, '{"status":"error"}'));
    const res = await tunnelFetch('/api/version');
    expect(res.ok).toBe(false);
    expect(res.status).toBe(401);
  });

  it('다운로드 — 본문을 모으지 않고 청크마다 넘긴다(헤더가 청크 경계에 걸려도)', async () => {
    const big = new Uint8Array(70_000).map((_, i) => (i * 7) % 256);
    fakeRobot(() => http(200, big));
    const got: Uint8Array[] = [];
    const res = await tunnelDownload('/api/blackbox/zip?path=a', {}, (b) => got.push(b.slice()));
    expect(res.status).toBe(200);
    expect(got.length).toBeGreaterThan(3);
    const all = new Uint8Array(got.reduce((n, b) => n + b.length, 0));
    let o = 0; for (const b of got) { all.set(b, o); o += b.length; }
    expect(all).toEqual(big);
  });

  it('다운로드 — 쓰기가 실패하면(디스크 가득) 그 에러로 끝난다', async () => {
    fakeRobot(() => http(200, new Uint8Array(40_000)));
    await expect(tunnelDownload('/api/x', {}, () => { throw new Error('ENOSPC'); })).rejects.toThrow('ENOSPC');
  });

  it('로봇이 연결을 거부하면 에러', async () => {
    const dc: any = { readyState: 'open', onmessage: null };
    dc.send = (d: any) => {
      if (typeof d !== 'string') return;
      const m = JSON.parse(d);
      if (m.t === 'open') queueMicrotask(() => dc.onmessage({ data: JSON.stringify({ t: 'close', id: m.id, err: 'port not allowed' }) }));
    };
    attachTunnel(dc);
    dc.onmessage({ data: JSON.stringify({ t: 'hello', v: 1 }) });
    await expect(tunnelFetch('http://x:22/')).rejects.toThrow('port not allowed');
  });
});

describe('포트별 피어(2PC)', () => {
  const peer = (name: string) => {
    const sent: string[] = [];
    const dc: any = { readyState: 'open', onmessage: null, send: (d: any) => { if (typeof d === 'string') sent.push(name + JSON.parse(d).t); } };
    return { dc, sent };
  };
  it('8082 는 SLAMNAV 피어가 준비돼 있으면 그쪽, 아니면 모션 피어', async () => {
    const tick = () => new Promise((r) => setTimeout(r, 0));
    const m = peer('m'), sl = peer('s');
    attachTunnel(m.dc); m.dc.onmessage({ data: JSON.stringify({ t: 'hello', v: 1 }) });
    attachPortTunnel(8082, sl.dc);
    tunnelFetch('/slamnav/api/map/topo').catch(() => {});
    await tick();
    sl.dc.onmessage({ data: JSON.stringify({ t: 'hello', v: 1 }) });
    tunnelFetch('/slamnav/api/map/topo').catch(() => {});
    tunnelFetch('/api/version').catch(() => {});
    await tick();
    expect(m.sent).toEqual(['mopen', 'mopen']);
    expect(sl.sent).toEqual(['sopen']);
    attachPortTunnel(8082, null);
  });
});

describe('robotTunnel — 끊김', () => {
  const short = () => { const r = http(200, new Uint8Array(100)); return r.subarray(0, r.length - 40); };

  it('다운로드: Content-Length 보다 덜 받고 닫히면 성공으로 넘기지 않는다', async () => {
    fakeRobot(short);
    let got = 0;
    await expect(tunnelDownload('/api/blackbox/zip', {}, (b) => { got += b.length; })).rejects.toThrow('잘렸습니다');
    expect(got).toBe(60);
  });

  it('REST: 덜 온 본문은 오류', async () => {
    fakeRobot(short);
    await expect(tunnelFetch('/api/version')).rejects.toThrow('잘렸습니다');
  });

  it('스트림이 열린 뒤 아무것도 안 오면 STREAM_IDLE_MS 뒤 끊는다(피어는 살아 있어도)', async () => {
    vi.useFakeTimers();
    try {
      const dc: any = { readyState: 'open', onmessage: null };
      dc.send = (d: any) => { if (typeof d === 'string') { const m = JSON.parse(d); if (m.t === 'open') queueMicrotask(() => dc.onmessage({ data: JSON.stringify({ t: 'opened', id: m.id }) })); } };
      attachTunnel(dc);
      dc.onmessage({ data: JSON.stringify({ t: 'hello', v: 1 }) });
      const p = tunnelDownload('/api/blackbox/zip', {}, () => {});
      const res = expect(p).rejects.toThrow('응답하지 않습니다');
      await vi.advanceTimersByTimeAsync(STREAM_IDLE_MS + 10);
      await res;
    } finally {
      vi.useRealTimers();
    }
  });
});

describe('TunnelWebSocket', () => {
  it('업그레이드 → 서버 텍스트 프레임 수신, 클라 프레임은 마스킹되어 원문 복원', async () => {
    const dc: any = { readyState: 'open', onmessage: null };
    let id = 0;
    const got: string[] = [];
    let stage = 0;
    const frame = (s: string) => { const p = new TextEncoder().encode(s); const f = new Uint8Array(2 + p.length); f[0] = 0x81; f[1] = p.length; f.set(p, 2); return f; };
    const push = (b: Uint8Array) => { const f = new Uint8Array(4 + b.length); f[3] = id; f.set(b, 4); dc.onmessage({ data: f.buffer }); };
    dc.send = (d: any) => {
      if (typeof d === 'string') { const m = JSON.parse(d); if (m.t === 'open') { id = m.id; queueMicrotask(() => dc.onmessage({ data: JSON.stringify({ t: 'opened', id }) })); } return; }
      const b = new Uint8Array(d).subarray(4);
      if (stage === 0) {
        stage = 1;
        expect(new TextDecoder().decode(b)).toMatch(/^GET \/ HTTP\/1.1\r\n[\s\S]*Upgrade: websocket/);
        queueMicrotask(() => { push(new TextEncoder().encode('HTTP/1.1 101 Switching Protocols\r\n\r\n')); push(frame('{"type":"connected","protocol":2}')); });
        return;
      }
      expect(b[1] & 0x80).toBe(0x80);
      const n = b[1] & 0x7f; const mask = b.subarray(2, 6);
      got.push(new TextDecoder().decode(b.subarray(6, 6 + n).map((x, i) => x ^ mask[i & 3])));
    };
    attachTunnel(dc);
    dc.onmessage({ data: JSON.stringify({ t: 'hello', v: 1 }) });
    const ws = new TunnelWebSocket(56785);
    const first = await new Promise<string>((r) => { ws.onmessage = (e) => r(String(e.data)); });
    expect(JSON.parse(first)).toEqual({ type: 'connected', protocol: 2 });
    ws.send('{"type":"start"}');
    expect(got).toEqual(['{"type":"start"}']);
    const big = new Uint8Array(65_536 + 3).map((_, i) => (i * 13) & 255);
    let raw: Uint8Array | null = null;
    dc.send = (d: any) => { if (typeof d !== 'string') raw = raw ? (() => { const c = new Uint8Array(raw!.length + d.byteLength - 4); c.set(raw!); c.set(new Uint8Array(d).subarray(4), raw!.length); return c; })() : new Uint8Array(d).subarray(4); };
    ws.send(big);
    const b = raw!;
    expect(b[1] & 0x7f).toBe(127);
    const mk = b.subarray(10, 14);
    expect(b.subarray(14).map((x, i) => x ^ mk[i & 3])).toEqual(big);
  });
});
