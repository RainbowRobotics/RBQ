import NetInfo from '@react-native-community/netinfo';
import { useRobots } from '@/store/robots';

const SETTLE_MS = 2000;

export function startLanUpgradeWatch(): () => void {
  let key = '';
  let timer: ReturnType<typeof setTimeout> | null = null;
  const unsub = NetInfo.addEventListener((st) => {
    const d: any = st.details;
    const next = `${st.type}|${d?.ipAddress ?? ''}|${d?.ssid ?? ''}`;
    if (next === key) return;
    const first = key === '';
    key = next;
    if (first || !st.isConnected) return;
    if (timer) clearTimeout(timer);
    timer = setTimeout(() => { void useRobots.getState().adoptLanRobot(); }, SETTLE_MS);
  });
  return () => { unsub(); if (timer) clearTimeout(timer); };
}
