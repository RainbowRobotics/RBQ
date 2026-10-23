import { useEffect, useState } from 'react';

type Batt = { pct: number; charging: boolean } | null;

export function useDeviceBattery(): Batt {
  const [batt, setBatt] = useState<Batt>(null);
  useEffect(() => {
    const getBattery = (navigator as any)?.getBattery?.bind(navigator);
    if (!getBattery) return;
    let alive = true;
    let mgr: any = null;
    const sync = () => {
      if (!alive || !mgr) return;
      setBatt({ pct: Math.round((mgr.level ?? 0) * 100), charging: !!mgr.charging });
    };
    getBattery().then((m: any) => {
      mgr = m;
      sync();
      m.addEventListener('levelchange', sync);
      m.addEventListener('chargingchange', sync);
    }).catch(() => {});
    return () => {
      alive = false;
      if (mgr) {
        mgr.removeEventListener('levelchange', sync);
        mgr.removeEventListener('chargingchange', sync);
      }
    };
  }, []);
  return batt;
}
