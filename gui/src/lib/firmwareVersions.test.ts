import { beforeEach, describe, expect, it, vi } from 'vitest';

const cfg = vi.hoisted(() => ({ channel: 'release' }));
vi.mock('react-native', () => ({ Platform: { OS: 'android' } }));
vi.mock('expo-constants', () => ({ default: { expoConfig: { extra: cfg } } }));
vi.mock('@/lib/desktopBridge', () => ({ isDesktop: () => false }));
vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
vi.mock('@/lib/fileDownload', () => ({ downloadToFile: vi.fn() }));
vi.mock('expo-file-system/legacy', () => ({}));
vi.mock('expo-intent-launcher', () => ({}));
vi.mock('@/store/robot', () => ({ useRobot: {} }));
vi.mock('@/store/telemetry', () => ({ useTelemetry: {} }));

const serve = (body: string, json = true) =>
  vi.stubGlobal('fetch', vi.fn(async () => ({
    ok: true, status: 200, json: async () => JSON.parse(body), text: async () => body,
  })));

async function load(channel: string) {
  cfg.channel = channel;
  vi.resetModules();
  return import('./firmwareRelease');
}

beforeEach(() => vi.unstubAllGlobals());

describe('HTTPS 이전 릴리스 숨김', () => {
  it('hides_pre_https_releases', async () => {
    serve(JSON.stringify({ latest: 'v1.2701.0', versions: ['v1.2701.0', 'v1.2609.1', 'v1.2609.0'] }));
    const m = await load('release');
    const list = await m.fetchFirmwareVersions();
    expect(list.map((r) => r.version)).toEqual(['1.2701.0']);
    expect(list[0].latest).toBe(true);
  });

  it('nightly_not_filtered', async () => {
    serve('version: v1.2608.0-nightly.2c9dc452\nbuilt_at: 2026-11-10T00:00:00Z\n');
    const m = await load('nightly');
    const list = await m.fetchFirmwareVersions();
    expect(list.map((r) => r.version)).toEqual(['1.2608.0-nightly.2c9dc452']);
  });
});
