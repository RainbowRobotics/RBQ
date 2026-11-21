import { isDesktop, wifiConnect, wifiCurrent, wifiPin, wifiScan, type WifiNetwork } from '@/lib/desktopBridge';

export type WifiSwitch = 'already' | 'switched' | 'not_visible' | 'not_saved' | 'failed' | 'unsupported';

const onRobotNet = (ip?: string | null) => !!ip && ip.startsWith('192.168.0.');

export async function switchToRobotWifi(ssid: string, onStart: () => void, waitMs = 15000, known?: WifiNetwork[]): Promise<WifiSwitch> {
  if (!isDesktop()) return 'unsupported';
  try {
    const cur = await wifiCurrent();
    if (cur?.ssid === ssid) return 'already';
    onStart();
    let net = (known?.length ? known : await wifiScan()).find((n) => n.ssid === ssid);
    if (known?.length && net?.secured && !net.saved) net = (await wifiScan()).find((n) => n.ssid === ssid);
    if (!net) return 'not_visible';
    if (net.secured && !net.saved) return 'not_saved';
    try { await wifiConnect(ssid); } catch (e) {
      if (/No network with SSID/i.test(e instanceof Error ? e.message : String(e))) return 'not_visible';
      throw e;
    }
    const until = Date.now() + waitMs;
    while (Date.now() < until) {
      const w = await wifiCurrent().catch(() => null);
      if (w?.ssid === ssid && onRobotNet(w.ip)) return 'switched';
      await new Promise((r) => setTimeout(r, 1000));
    }
    return 'failed';
  } catch { return 'failed'; }
}

export function otherRobotSsids(profiles: { lan?: { ssid?: string } }[], keep: string): string[] {
  return [...new Set(profiles.map((p) => p.lan?.ssid).filter((x): x is string => !!x && x !== keep))];
}
export function pinRobotWifi(keep: string, profiles: { lan?: { ssid?: string } }[]): void {
  if (!isDesktop() || !keep) return;
  void wifiPin(keep, otherRobotSsids(profiles, keep)).catch(() => {});
}
