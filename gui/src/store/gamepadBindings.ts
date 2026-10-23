import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';
import type { MotionName } from '@/types/robot';

export type BindingAction = { type: 'motion'; motion: MotionName };

export type GamepadBinding = {
  keyCode: number;
  label: string;
  action: BindingAction;
};

type BindingsState = {
  bindings: GamepadBinding[];
  setBinding: (b: GamepadBinding) => void;
  removeBinding: (keyCode: number) => void;
};

export const useGamepadBindings = create<BindingsState>()(
  persist(
    (set) => ({
      bindings: [],
      setBinding: (b) =>
        set((s) => ({ bindings: [...s.bindings.filter((x) => x.keyCode !== b.keyCode), b] })),
      removeBinding: (keyCode) =>
        set((s) => ({ bindings: s.bindings.filter((x) => x.keyCode !== keyCode) })),
    }),
    { name: 'rbq-gamepad-bindings', storage: createJSONStorage(() => AsyncStorage) },
  ),
);
