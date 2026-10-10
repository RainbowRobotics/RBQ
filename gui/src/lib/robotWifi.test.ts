import { describe, it, expect, vi, beforeEach } from 'vitest';

const bridge = { desktop: true, current: null as any, nets: [] as any[], connect: vi.fn(async () => {}), scans: 0 };
vi.mock('@/lib/desktopBridge', () => ({
  isDesktop: () => bridge.desktop,
  wifiCurrent: async () => bridge.current,
  wifiScan: async () => { bridge.scans++; return bridge.nets; },
  wifiConnect: (ssid: string) => { bridge.current = { ssid, ip: '192.168.0.13' }; return bridge.connect(); },
}));
import { switchToRobotWifi } from './robotWifi';

beforeEach(() => { bridge.desktop = true; bridge.current = { ssid: 'Office', ip: '192.168.201.5' }; bridge.nets = []; bridge.connect.mockClear(); bridge.scans = 0; });

describe('switchToRobotWifi — 로봇을 고르면 그 로봇 WiFi 로', () => {
  it('모바일·웹은 바꾸지 않는다(unsupported)', async () => {
    bridge.desktop = false;
    expect(await switchToRobotWifi('RBQ_EXAMPLE', () => {})).toBe('unsupported');
  });
  it('이미 그 WiFi 면 그대로', async () => {
    bridge.current = { ssid: 'RBQ_EXAMPLE', ip: '192.168.0.13' };
    expect(await switchToRobotWifi('RBQ_EXAMPLE', () => {})).toBe('already');
  });
  it('주변에 없으면 not_visible, 비밀번호 저장 안 된 망이면 not_saved — 둘 다 붙지 않는다', async () => {
    expect(await switchToRobotWifi('RBQ_EXAMPLE', () => {})).toBe('not_visible');
    bridge.nets = [{ ssid: 'RBQ_EXAMPLE', secured: true, saved: false }];
    expect(await switchToRobotWifi('RBQ_EXAMPLE', () => {})).toBe('not_saved');
    expect(bridge.connect).not.toHaveBeenCalled();
  });
  it('방금 스캔한 목록을 주면 다시 스캔하지 않는다(덱 rescan 7~15초)', async () => {
    const start = vi.fn();
    expect(await switchToRobotWifi('RBQ_EXAMPLE', start, 2000, [{ ssid: 'RBQ_EXAMPLE', secured: true, saved: true } as any])).toBe('switched');
    expect(bridge.scans).toBe(0);
  });
  it('낡은 목록에 있던 AP 가 사라졌으면(nmcli No network) not_visible', async () => {
    bridge.connect.mockRejectedValueOnce(new Error("Error: No network with SSID 'RBQ_EXAMPLE' found."));
    expect(await switchToRobotWifi('RBQ_EXAMPLE', () => {}, 2000, [{ ssid: 'RBQ_EXAMPLE', secured: true, saved: true } as any])).toBe('not_visible');
  });
  it('보이고 저장된 망이면 붙고 로봇망 IP 를 받으면 switched', async () => {
    bridge.nets = [{ ssid: 'RBQ_EXAMPLE', secured: true, saved: true }];
    const start = vi.fn();
    expect(await switchToRobotWifi('RBQ_EXAMPLE', start, 2000)).toBe('switched');
    expect(start).toHaveBeenCalled();
  });
});
