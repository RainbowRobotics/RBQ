import { describe, it, expect, vi, beforeEach } from 'vitest';

vi.mock('@/lib/desktopBridge', () => ({
  wifiScan: vi.fn(),
  wifiConnect: vi.fn(),
  wifiCurrent: vi.fn(),
}));

import { wifiScan, wifiConnect, wifiCurrent } from '@/lib/desktopBridge';
import { useWifi } from './wifi';

beforeEach(() => {
  vi.clearAllMocks();
  useWifi.setState({ current: null, networks: [], scanning: false, connecting: false, error: null });
});

describe('useWifi', () => {
  it('scan() 성공 시 networks 채우고 scanning false', async () => {
    (wifiScan as any).mockResolvedValue([{ ssid: 'A', signal: 50, secured: true, active: false }]);
    await useWifi.getState().scan();
    expect(useWifi.getState().networks).toHaveLength(1);
    expect(useWifi.getState().scanning).toBe(false);
    expect(useWifi.getState().error).toBeNull();
  });

  it('scan() 실패 시 error 설정', async () => {
    (wifiScan as any).mockRejectedValue(new Error('스캔 실패함'));
    await useWifi.getState().scan();
    expect(useWifi.getState().error).toBe('스캔 실패함');
    expect(useWifi.getState().scanning).toBe(false);
  });

  it('connect() 성공 시 true 반환 + refreshCurrent 호출', async () => {
    (wifiConnect as any).mockResolvedValue(undefined);
    (wifiCurrent as any).mockResolvedValue({ ssid: 'A', signal: 50, secured: true, active: true });
    const ok = await useWifi.getState().connect('A', 'pw');
    expect(ok).toBe(true);
    expect(wifiConnect).toHaveBeenCalledWith('A', 'pw');
    expect(useWifi.getState().current?.ssid).toBe('A');
  });

  it('connect() 실패 시 false 반환 + error 설정', async () => {
    (wifiConnect as any).mockRejectedValue(new Error('비번 틀림'));
    const ok = await useWifi.getState().connect('A', 'bad');
    expect(ok).toBe(false);
    expect(useWifi.getState().error).toBe('비번 틀림');
  });
});
