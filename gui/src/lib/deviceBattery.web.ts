import { useEffect, useState } from 'react';
import { isDesktop } from '@/lib/desktopBridge';

type Batt = { pct: number; charging: boolean } | null;

export function useDeviceBattery(): Batt {
  const [batt, setBatt] = useState<Batt>(null);
  useEffect(() => {
    const getBattery = (navigator as any)?.getBattery?.bind(navigator);
    if (!getBattery) return isDesktop() ? pollProxy(setBatt) : undefined;
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

function pollProxy(setBatt: (b: Batt) => void): () => void {
  let alive = true;
  const tick = () => {
    fetch('/host-battery')
      .then((r) => (r.ok ? r.json() : null))
      .then((j) => {
        if (!alive) return;
        setBatt(j?.present && typeof j.percent === 'number'
          ? { pct: j.percent, charging: !!j.charging } : null);
      })
      .catch(() => {});
  };
  const id = setInterval(tick, 60_000);
  tick();
  return () => { alive = false; clearInterval(id); };
}
