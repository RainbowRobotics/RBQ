import { useEffect, useState } from 'react';
import { Platform } from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { isDesktop } from '@/lib/desktopBridge';
import { useWifi } from '@/store/wifi';
import { useRobot } from '@/store/robot';
import { connection } from '@/lib/connection';
import { linkQualityPct } from '@/lib/connectionRoute';

const POLL_MS = 5000;

export function useSignalStrength(): number | null {
  const desktop = isDesktop();
  const wifiSignal = useWifi((s) => (s.current && s.current.signal > 0 ? s.current.signal : null));
  const refreshCurrent = useWifi((s) => s.refreshCurrent);
  const [androidPct, setAndroidPct] = useState<number | null>(null);

  useEffect(() => {
    if (!desktop) return;
    refreshCurrent();
    const t = setInterval(() => refreshCurrent(), POLL_MS);
    return () => clearInterval(t);
  }, [desktop, refreshCurrent]);

  useEffect(() => {
    if (Platform.OS !== 'android') return;
    const unsub = NetInfo.addEventListener((st) => {
      const d: any = st.type === 'wifi' ? st.details : null;
      setAndroidPct(typeof d?.strength === 'number' ? d.strength : null);
    });
    return unsub;
  }, []);

  const remote = useRobot((s) => s.via === 'rendezvous' && s.conn === 'connected');
  const [linkPct, setLinkPct] = useState<number | null>(null);
  useEffect(() => {
    if (!remote) { setLinkPct(null); return; }
    const tick = () => connection.stats().then((st) => setLinkPct(linkQualityPct(st))).catch(() => {});
    tick();
    const t = setInterval(tick, POLL_MS);
    return () => clearInterval(t);
  }, [remote]);

  if (remote) return linkPct;
  if (desktop) return wifiSignal;
  if (Platform.OS === 'android') return androidPct;
  return null;
}

export function signalBars(percent: number): number {
  if (percent > 90) return 5;
  if (percent > 80) return 4;
  if (percent > 55) return 3;
  if (percent > 30) return 2;
  if (percent > 0) return 1;
  return 0;
}
