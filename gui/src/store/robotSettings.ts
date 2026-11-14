import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';

type Values = Record<string, unknown>;

type RobotSettingsStore = {
  serial: string;
  bySerial: Record<string, Values>;
  setSerial: (s: string) => void;
  put: (values: Values) => void;
};

export const useRobotSettings = create<RobotSettingsStore>()(
  persist(
    (set, get) => ({
      serial: '',
      bySerial: {},
      setSerial: (serial) => { if (get().serial !== serial) set({ serial }); },
      put: (values) => {
        const { serial, bySerial } = get();
        if (!serial) return;
        const cur = bySerial[serial] ?? {};
        if (Object.keys(values).every((k) => Object.is(cur[k], values[k]))) return;
        set({ bySerial: { ...bySerial, [serial]: { ...cur, ...values } } });
      },
    }),
    {
      name: 'rbq-robot-settings',
      storage: createJSONStorage(() => AsyncStorage),
      partialize: (s) => ({ bySerial: s.bySerial }),
    },
  ),
);

export function cachedFor(serial: string): Values | undefined {
  return serial ? useRobotSettings.getState().bySerial[serial] : undefined;
}

export function bindRobotCache<S extends object>(
  store: { getState: () => S; setState: (p: Partial<S>) => void; subscribe: (fn: (s: S, prev: S) => void) => () => void },
  keys: (keyof S & string)[],
  prefix: string,
): () => void {
  const defaults = Object.fromEntries(keys.map((k) => [k, store.getState()[k]])) as Partial<S>;
  let hydrating = false;
  const unsaved = new Set<string>();

  const hydrate = (serial: string) => {
    const cached = cachedFor(serial) ?? {};
    const patch: Partial<S> = {};
    const keep: Values = {};
    for (const k of keys) {
      const ck = `${prefix}.${k}`;
      if (unsaved.has(k)) keep[ck] = store.getState()[k];
      else (patch as Record<string, unknown>)[k] = ck in cached ? cached[ck] : defaults[k];
    }
    unsaved.clear();
    hydrating = true;
    store.setState(patch);
    hydrating = false;
    if (Object.keys(keep).length) useRobotSettings.getState().put(keep);
  };

  const offStore = store.subscribe((s, prev) => {
    if (hydrating) return;
    const changed: Values = {};
    for (const k of keys) {
      if (Object.is(s[k], prev[k])) continue;
      changed[`${prefix}.${k}`] = s[k];
      if (!useRobotSettings.getState().serial) unsaved.add(k);
    }
    if (Object.keys(changed).length) useRobotSettings.getState().put(changed);
  });

  const offSerial = useRobotSettings.subscribe((s, prev) => {
    if (s.serial && s.serial !== prev.serial) hydrate(s.serial);
  });
  const offHydrated = useRobotSettings.persist.onFinishHydration((s) => { if (s.serial) hydrate(s.serial); });
  if (useRobotSettings.getState().serial) hydrate(useRobotSettings.getState().serial);

  return () => { offStore(); offSerial(); offHydrated(); };
}
