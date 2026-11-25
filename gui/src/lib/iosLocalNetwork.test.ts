import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';

const rn = vi.hoisted(() => ({ Platform: { OS: 'ios' } }));
const store = vi.hoisted(() => new Map<string, string>());
vi.mock('react-native', () => ({ Platform: rn.Platform }));
vi.mock('@react-native-async-storage/async-storage', () => ({
  default: {
    getItem: async (k: string) => store.get(k) ?? null,
    setItem: async (k: string, v: string) => { store.set(k, v); },
  },
}));
vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
import { probeThroughLocalNetworkPrompt, localNetworkHint } from './iosLocalNetwork';

beforeEach(() => { rn.Platform.OS = 'ios'; store.clear(); vi.useFakeTimers(); });
afterEach(() => { vi.useRealTimers(); });

describe('probeThroughLocalNetworkPrompt', () => {
  it('처음 쓰는 기기 — 허용 창 때문에 실패해도 다시 물어 [허용] 뒤 붙는다(다시 누르지 않아도)', async () => {
    let allowed = false;
    const probe = vi.fn(async () => (allowed ? 'RBQ10-001' : null));
    const p = probeThroughLocalNetworkPrompt(probe);
    await vi.advanceTimersByTimeAsync(4_000);
    allowed = true;
    await vi.advanceTimersByTimeAsync(1_600);
    expect(await p).toBe('RBQ10-001');
    expect(probe.mock.calls.length).toBeGreaterThan(2);
  });

  it('처음인데 끝내 안 닿으면 기다림 상한 뒤 실패 — 그동안 계속 묻는다', async () => {
    const probe = vi.fn(async () => null);
    const p = probeThroughLocalNetworkPrompt(probe);
    await vi.advanceTimersByTimeAsync(21_000);
    expect(await p).toBeNull();
    expect(probe.mock.calls.length).toBeGreaterThan(10);
  });

  it('한 번 닿은 기기는 기다리지 않는다(그냥 로봇이 없음)', async () => {
    const ok = await probeThroughLocalNetworkPrompt(async () => 'RBQ10-001');
    expect(ok).toBe('RBQ10-001');
    const probe = vi.fn(async () => null);
    expect(await probeThroughLocalNetworkPrompt(probe)).toBeNull();
    expect(probe).toHaveBeenCalledTimes(1);
  });

  it('iOS 가 아니면 그대로', async () => {
    rn.Platform.OS = 'android';
    const probe = vi.fn(async () => null);
    expect(await probeThroughLocalNetworkPrompt(probe)).toBeNull();
    expect(probe).toHaveBeenCalledTimes(1);
    expect(localNetworkHint()).toBe('');
  });
});
