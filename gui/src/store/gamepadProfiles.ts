import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';
import type { GamepadProfile } from '@/lib/gamepad/profiles';

type ProfilesStore = {
  profiles: Record<string, GamepadProfile>;
  setProfile: (descriptor: string, p: GamepadProfile) => void;
  removeProfile: (descriptor: string) => void;
};

export const useGamepadProfiles = create<ProfilesStore>()(
  persist(
    (set) => ({
      profiles: {},
      setProfile: (descriptor, p) => set((s) => ({ profiles: { ...s.profiles, [descriptor]: p } })),
      removeProfile: (descriptor) =>
        set((s) => { const { [descriptor]: _drop, ...rest } = s.profiles; return { profiles: rest }; }),
    }),
    { name: 'rbq-gamepad-profiles', storage: createJSONStorage(() => AsyncStorage) },
  ),
);
