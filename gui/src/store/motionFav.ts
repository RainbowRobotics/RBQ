import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';
import type { MotionName } from '@/types/robot';

export const FAV_SLOTS = 2;
export const FAV_SLOTS_FAN = 2;

type MotionFavState = {
  slots: (MotionName | null)[];
  setSlot: (i: number, m: MotionName) => void;
};

const pad = (arr: (MotionName | null)[]) =>
  [...arr, ...Array(FAV_SLOTS).fill(null)].slice(0, FAV_SLOTS);

export const useMotionFav = create<MotionFavState>()(
  persist(
    (set) => ({
      slots: Array(FAV_SLOTS).fill(null),
      setSlot: (i, m) =>
        set((s) => {
          const slots = pad(s.slots);
          slots[i] = m;
          return { slots };
        }),
    }),
    {
      name: 'rbq-motion-fav',
      storage: createJSONStorage(() => AsyncStorage),
      merge: (persisted: any, current) => ({
        ...current,
        ...(persisted ?? {}),
        slots: pad((persisted as MotionFavState | undefined)?.slots ?? []),
      }),
    },
  ),
);
