import { describe, it, expect, vi, afterEach } from 'vitest';

const store = (v: string | null) => {
  const m = new Map<string, string>(v ? [['rbq-install-id', v]] : []);
  return { getItem: (k: string) => m.get(k) ?? null, setItem: (k: string, x: string) => { m.set(k, x); } };
};
const load = async (v: string | null) => {
  vi.resetModules();
  (globalThis as any).sessionStorage = store(v);
  const m = await import('./installId');
  await m.installIdReady;
  return m.installId();
};
let other: BroadcastChannel | null = null;
afterEach(() => { other?.close(); other = null; delete (globalThis as any).sessionStorage; });

describe('installId — 웹 탭', () => {
  it('새로 고침(같은 id 를 쓰는 탭 없음)은 id 를 유지한다', async () => {
    expect(await load('abc12345')).toBe('abc12345');
  });
  it('복제된 탭(원본이 같은 id 로 살아 있음)은 새 id', async () => {
    other = new BroadcastChannel('rbq-install-id');
    other.onmessage = (e) => { if (e.data?.ask === 'abc12345') other!.postMessage({ mine: 'abc12345' }); };
    const id = await load('abc12345');
    expect(id).not.toBe('abc12345');
    expect(id).toMatch(/^[0-9a-f]+$/);
  });
});
