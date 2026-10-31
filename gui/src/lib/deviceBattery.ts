import { useEffect, useState } from 'react';
import { useBatteryLevel, useBatteryState, BatteryState, getBatteryLevelAsync } from 'expo-battery';

export function useDeviceBattery(): { pct: number; charging: boolean } | null {
  const level = useBatteryLevel();
  const state = useBatteryState();
  const [polled, setPolled] = useState<number | null>(null);
  useEffect(() => {
    let alive = true;
    const tick = () => { getBatteryLevelAsync().then((l) => { if (alive && typeof l === 'number') setPolled(l); }).catch(() => {}); };
    const id = setInterval(tick, 60_000);
    tick();
    return () => { alive = false; clearInterval(id); };
  }, []);
  const eff = polled != null && polled >= 0 ? polled : level;
  if (eff == null || eff < 0) return null;
  return { pct: Math.round(eff * 100), charging: state === BatteryState.CHARGING };
}
