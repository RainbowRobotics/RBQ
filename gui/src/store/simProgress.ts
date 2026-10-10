import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';

export type LevelRecord = { best: number; at: string; count: number };

type SimProgressStore = {
  cleared: Record<string, LevelRecord>;
  recordClear: (id: string, sec: number) => boolean;
  resetAll: () => void;
};

export const useSimProgress = create<SimProgressStore>()(
  persist(
    (set, get) => ({
      cleared: {},
      recordClear: (id, sec) => {
        const prev = get().cleared[id];
        const better = !prev || sec < prev.best;
        set({ cleared: { ...get().cleared, [id]: {
          best: better ? sec : prev.best, at: new Date().toISOString(), count: (prev?.count ?? 0) + 1,
        } } });
        return better;
      },
      resetAll: () => set({ cleared: {} }),
    }),
    { name: 'rbq-sim-progress', storage: createJSONStorage(() => AsyncStorage), version: 1 },
  ),
);
