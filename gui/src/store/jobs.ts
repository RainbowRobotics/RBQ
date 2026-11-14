import { useEffect } from 'react';
import { create, type StoreApi, type UseBoundStore } from 'zustand';
import type { IconName } from '@/components/Icon';

export type JobBase = { open: number; unseen: boolean };

export type JobStore<T> = UseBoundStore<StoreApi<T & JobBase>> & {
  use<K extends keyof T>(k: K): [(T & JobBase)[K], (v: (T & JobBase)[K]) => void];
  setter<K extends keyof T>(k: K): (v: (T & JobBase)[K]) => void;
  useScreen(): void;
};

export function screenState<T extends object>(init: T, busy?: (s: T) => boolean): JobStore<T> {
  const store = create<T & JobBase>(() => ({ ...init, open: 0, unseen: false }));
  if (busy) store.subscribe((s, p) => { if (s.open === 0 && busy(p) && !busy(s)) store.setState({ unseen: true } as Partial<T & JobBase>); });
  const setters = new Map<PropertyKey, (v: never) => void>();
  const setter = <K extends keyof T>(k: K) => {
    let f = setters.get(k);
    if (!f) { f = (v: never) => store.setState({ [k]: v } as unknown as Partial<T & JobBase>); setters.set(k, f); }
    return f as (v: (T & JobBase)[K]) => void;
  };
  const use = <K extends keyof T>(k: K): [(T & JobBase)[K], (v: (T & JobBase)[K]) => void] => [useField(store, k), setter(k)];
  const useScreen = () => useJobScreen(store);
  return Object.assign(store, { use, setter, useScreen });
}

function useField<T, K extends keyof T>(store: UseBoundStore<StoreApi<T>>, k: K): T[K] {
  return store((s) => s[k]);
}
function useJobScreen(store: StoreApi<JobBase>) {
  useEffect(() => {
    store.setState((s) => ({ open: s.open + 1, unseen: false }));
    return () => store.setState((s) => ({ open: s.open - 1 }));
  }, [store]);
}

export type JobTone = 'run' | 'ok' | 'bad';
export type JobLine = { text: string; tone: JobTone; closable: boolean };

export function jobLine(s: JobBase, t: { run?: string | null; bad?: string | null; ok?: string | null }): JobLine | null {
  if (s.open > 0) return null;
  if (t.run) return { text: t.run, tone: 'run', closable: false };
  if (!s.unseen) return null;
  if (t.bad) return { text: t.bad, tone: 'bad', closable: true };
  if (t.ok) return { text: t.ok, tone: 'ok', closable: true };
  return null;
}

export type JobSource = {
  key: string;
  route: string;
  icon: IconName;
  label: () => string;
  useLine: () => JobLine | null;
  dismiss: () => void;
};
