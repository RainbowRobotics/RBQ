import { Platform } from 'react-native';
import { isDesktop, wifiCurrent } from '@/lib/desktopBridge';

const onRobotNet = (ip?: string | null) => !ip || ip.startsWith('192.168.0.');

export async function currentSsid(): Promise<string | null> {
  try {
    if (isDesktop()) { const w = await wifiCurrent(); return w?.ssid && onRobotNet(w.ip) ? w.ssid : null; }
    if (Platform.OS === 'android' || Platform.OS === 'ios') {
      const NetInfo = (await import('@react-native-community/netinfo')).default;
      const st = await NetInfo.fetch();
      const d: any = st.type === 'wifi' ? st.details : null;
      const ssid = d?.ssid;
      return ssid && ssid !== '<unknown ssid>' && onRobotNet(d?.ipAddress) ? String(ssid) : null;
    }
  } catch { }
  return null;
}
