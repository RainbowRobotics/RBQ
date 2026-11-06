import { afterEach, beforeEach, describe, expect, it, vi } from 'vitest';

const robot = vi.hoisted(() => ({
  part: 0,
  total: 0,
  down: false,
  dropAt: 0,
  dropOnApply: false,
  dropAtEnd: false,
  watcher: null as null | { reply: (m: object) => void },
  stage: 'idle' as 'idle' | 'receiving' | 'deploying',
  last: null as null | { ok: boolean; fileName: string; message?: string; at: number },
  starts: [] as number[],
  bytes: 0,
}));

vi.mock('./robotTunnel', () => ({
  viaRemote: () => false, tunnelReady: () => true, TunnelWebSocket: class {}, RemoteUnsupportedError: class extends Error {},
}));
vi.mock('./demoFlag', () => ({ isDemo: () => false }));
vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
vi.mock('./firmwareRelease', () => ({
  coreSystemOf: (n: string) => /-(u\d\d-\w+)\.rbq$/.exec(n)?.[1] ?? null,
  fileSidecars: async () => ({ sha256: 'ab'.repeat(32) }),
}));
vi.mock('./fileStream', () => ({
  openFileStream: () => {
    let done = false;
    return {
      read: async () => (done ? { done: true, value: undefined } : (done = true, { done: false, value: new Uint8Array(robot.total) })),
      cancel: async () => {},
    };
  },
}));

class FakeWS {
  onmessage: ((e: { data: string }) => void) | null = null;
  onerror: (() => void) | null = null;
  onclose: (() => void) | null = null;
  binaryType = '';
  private open = true;
  constructor() {
    queueMicrotask(() => {
      if (robot.down) { this.shut(); return; }
      const busy = robot.stage === 'idle' ? undefined : { stage: robot.stage, fileName: NAME };
      if (robot.stage === 'deploying') robot.watcher = this;
      this.reply({ type: 'connected', protocol: 2, system: 'u22-x86_64', ...(busy ? { busy } : {}), ...(robot.last ? { last: robot.last } : {}) });
    });
  }
  reply(m: object) { if (this.open) queueMicrotask(() => this.open && this.onmessage?.({ data: JSON.stringify(m) })); }
  private shut() { if (!this.open) return; this.open = false; queueMicrotask(() => this.onclose?.()); }
  send(d: string | ArrayBuffer) {
    if (!this.open) return;
    if (typeof d === 'string') {
      const m = JSON.parse(d);
      if (m.type === 'start_transfer') { robot.stage = 'receiving'; robot.starts.push(robot.part); this.reply({ type: 'ready', resumeFrom: robot.part }); }
      return;
    }
    robot.part += d.byteLength;
    robot.bytes += d.byteLength;
    this.reply({ type: 'progress', received: robot.part, percentage: Math.floor((robot.part * 100) / robot.total) });
    if (robot.dropAt && robot.part >= robot.dropAt) { robot.dropAt = 0; robot.stage = 'idle'; this.shut(); return; }
    if (robot.part === robot.total) {
      robot.stage = 'deploying';
      if (robot.dropAtEnd) { robot.dropAtEnd = false; this.shut(); return; }
      this.reply({ type: 'complete' });
      this.reply({ type: 'applying', step: 'deploy' });
      if (robot.dropOnApply) { robot.dropOnApply = false; queueMicrotask(() => queueMicrotask(() => this.shut())); return; }
      robot.stage = 'idle';
      this.reply({ type: 'deployed' });
    }
  }
  close() { this.shut(); }
}

const NAME = 'core-v1.2609.0-1019-u22-x86_64.rbq';
const file = () => ({ uri: 'file:///pkg', name: NAME, size: robot.total });

import { otaTransfer, type OtaProgress } from './ota';

beforeEach(() => {
  Object.assign(robot, { part: 0, total: 5 * 64 * 1024, down: false, dropAt: 0, dropOnApply: false, stage: 'idle', last: null, starts: [], bytes: 0 });
  vi.stubGlobal('WebSocket', FakeWS);
  vi.useFakeTimers({ toFake: ['setTimeout', 'clearTimeout', 'Date'] });
});
afterEach(() => { vi.useRealTimers(); vi.unstubAllGlobals(); });

async function settle(p: Promise<void>) {
  let out: unknown = 'pending';
  p.then(() => { out = 'ok'; }, (e) => { out = e; });
  for (let i = 0; i < 200 && out === 'pending'; i++) await vi.advanceTimersByTimeAsync(1000);
  return out;
}

describe('otaTransfer — 끊김', () => {
  it('전송 중 끊기면 다시 붙어 받은 데부터 이어 보낸다(같은 바이트를 두 번 보내지 않는다)', async () => {
    robot.dropAt = 2 * 64 * 1024;
    const phases: OtaProgress['phase'][] = [];
    const h = otaTransfer('10.0.0.1', file(), (p) => phases.push(p.phase));
    expect(await settle(h.done)).toBe('ok');
    expect(robot.starts).toEqual([0, 2 * 64 * 1024]);
    expect(robot.bytes).toBe(robot.total);
    expect(phases).toContain('reconnecting');
    expect(phases.at(-1)).toBe('complete');
  });

  it('설치 중 끊기면 다시 붙어 결과를 묻는다 — 아직 설치 중이면 기다리고, 끝났으면 last 로 판정', async () => {
    robot.dropOnApply = true;
    const phases: OtaProgress['phase'][] = [];
    const h = otaTransfer('10.0.0.1', file(), (p) => phases.push(p.phase));
    const out = settle(h.done);
    await vi.advanceTimersByTimeAsync(6000);
    expect(robot.watcher).not.toBeNull();
    expect(phases.at(-1)).toBe('applying');
    robot.stage = 'idle';
    robot.watcher!.reply({ type: 'deployed' });
    expect(await out).toBe('ok');
    expect(robot.bytes).toBe(robot.total);
  });

  it('설치가 실패했다고 남아 있으면 그 사유로 실패', async () => {
    robot.dropOnApply = true;
    const h = otaTransfer('10.0.0.1', file(), () => {});
    const out = settle(h.done);
    await vi.advanceTimersByTimeAsync(10);
    robot.stage = 'idle';
    robot.last = { ok: false, fileName: NAME, message: 'deploy.bash failed — the previous version was restored', at: Date.now() / 1000 };
    const e = await out;
    expect(String(e)).toMatch('previous version was restored');
  });

  it('다 보낸 직후(complete 전) 끊겼는데 로봇이 이미 끝냈으면 다시 보내지 않고 결과로 끝낸다', async () => {
    robot.dropAtEnd = true;
    const h = otaTransfer('10.0.0.1', file(), () => {});
    const out = settle(h.done);
    await vi.advanceTimersByTimeAsync(10);
    robot.stage = 'idle';
    robot.last = { ok: true, fileName: NAME, at: Date.now() / 1000 };
    expect(await out).toBe('ok');
    expect(robot.bytes).toBe(robot.total);
  });

  it('한 번도 못 붙으면 오래 매달리지 않고 대상이 없다고 알린다', async () => {
    robot.down = true;
    const h = otaTransfer('10.0.0.1', file(), () => {});
    const e = await settle(h.done);
    expect(String(e)).toMatch('연결 실패');
  });
});
