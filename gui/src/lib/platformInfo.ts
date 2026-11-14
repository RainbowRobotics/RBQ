import { useEffect, useState } from 'react';
import { Platform } from 'react-native';

export type PlatformInfo = { steamos: boolean; gaming: boolean };
let cached: PlatformInfo | null = null;
let pending: Promise<PlatformInfo> | null = null;

export function getPlatformInfo(): Promise<PlatformInfo> {
  if (cached) return Promise.resolve(cached);
  if (Platform.OS !== 'web') return Promise.resolve((cached = { steamos: false, gaming: false }));
  if (!pending) {
    pending = fetch('/platform')
      .then((r) => (r.ok ? r.json() : { steamos: false, gaming: false }))
      .catch(() => ({ steamos: false, gaming: false }))
      .then((p) => (cached = { steamos: !!p?.steamos, gaming: !!p?.gaming }));
  }
  return pending;
}

export function getSteamOSCached(): boolean {
  return cached?.steamos ?? false;
}

export function useSteamOS(): boolean {
  const [v, setV] = useState(cached?.steamos ?? false);
  useEffect(() => { let on = true; getPlatformInfo().then((p) => { if (on) setV(p.steamos); }); return () => { on = false; }; }, []);
  return v;
}

export function getDeckGamingCached(): boolean {
  return !!(cached?.steamos && cached?.gaming);
}

export function useDeckGaming(): boolean {
  const [v, setV] = useState(getDeckGamingCached());
  useEffect(() => { let on = true; getPlatformInfo().then((p) => { if (on) setV(p.steamos && p.gaming); }); return () => { on = false; }; }, []);
  return v;
}
