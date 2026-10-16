import { create } from 'zustand';
import type { GamepadDeviceInfo } from '@/lib/gamepad/input';
import { useSettings } from '@/store/settings';
import { useDeckGaming } from '@/lib/platformInfo';
import { resolveHasPhysicalGamepad } from '@/lib/keyboard/mode';
export { resolveHasPhysicalGamepad };

type ActiveGamepad = { name: string; profileLabel: string };

type GamepadState = {
  devices: GamepadDeviceInfo[];
  active: ActiveGamepad | null;
  setDevices: (devices: GamepadDeviceInfo[]) => void;
  setActive: (active: ActiveGamepad | null) => void;
};

export const useGamepad = create<GamepadState>()((set) => ({
  devices: [],
  active: null,
  setDevices: (devices) => set({ devices }),
  setActive: (active) => set({ active }),
}));


export const useHasPhysicalGamepad = () => {
  const count = useGamepad((s) => s.devices.length);
  const gpUiMode = useSettings((s) => s.gpUiMode);
  const deckGaming = useDeckGaming();
  return resolveHasPhysicalGamepad(count, gpUiMode, deckGaming);
};
