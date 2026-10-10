import { useHasPhysicalGamepad, useGamepad, resolveHasPhysicalGamepad } from '@/store/gamepad';
import { pickInputMode, keyboardDrive, isPcEnvironment } from '@/lib/keyboard/mode';
import { getSteamOSCached, getDeckGamingCached } from '@/lib/platformInfo';
import { useSettings } from '@/store/settings';

export function useInputMode(): 'gamepad' | 'touch' {
  return pickInputMode(useHasPhysicalGamepad());
}

export function getInputModeNow(): 'gamepad' | 'touch' {
  const { gpUiMode } = useSettings.getState();
  return pickInputMode(resolveHasPhysicalGamepad(useGamepad.getState().devices.length, gpUiMode, getDeckGamingCached()));
}

let controlScreen = false;
export const setControlScreenActive = (on: boolean) => { controlScreen = on; };

export function getKeyboardDriveNow(): boolean {
  if (!controlScreen) return false;
  const hasGamepad = resolveHasPhysicalGamepad(useGamepad.getState().devices.length, 'auto', getDeckGamingCached());
  return keyboardDrive(hasGamepad, isPcEnvironment(getSteamOSCached()));
}

