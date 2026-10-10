import { Platform } from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { isDesktop } from './desktopBridge';
import { reconnectCurrent } from './connectNow';
import { isDemo } from './demoFlag';
import { useWifi } from '@/store/wifi';
import { useRobot } from '@/store/robot';

const POLL_MS = 2000;

let fingerprint: string | null = null;

function robotFacing(id: string): string {
  const ip = useRobot.getState().ip;
  const net = ip ? ip.split('.').slice(0, 3).join('.') : '';
  if (!net) return id;
  return id.split(',').filter((e) => e.split(':')[1]?.startsWith(`${net}.`)).join(',');
}

function onFingerprint(next: string) {
  if (next === fingerprint) return;
  const first = fingerprint === null;
  fingerprint = next;
  if (first || isDemo()) return;
  console.log('[netWatch] 기기 망이 바뀌었다 — 즉시 재연결');
  try { void useWifi.getState().refreshCurrent(); } catch { }
  try { reconnectCurrent(); } catch { }
}

export function installNetWatch(): () => void {
  if (isDesktop()) {
    const tick = () => {
      fetch('/netid')
        .then((r) => (r.ok ? r.json() : null))
        .then((j) => { if (j && typeof j.id === 'string') onFingerprint(robotFacing(j.id)); })
        .catch(() => { });
    };
    tick();
    const id = setInterval(tick, POLL_MS);
    return () => clearInterval(id);
  }
  if (Platform.OS === 'android' || Platform.OS === 'ios') {
    return NetInfo.addEventListener((st) => {
      const d: any = st.details ?? {};
      if (!st.isConnected) return;
      onFingerprint(`${st.type}:${d.ipAddress ?? ''}:${d.ssid ?? ''}`);
    });
  }
  return () => {};
}
