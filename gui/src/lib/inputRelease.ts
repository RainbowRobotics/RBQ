import { AppState, type AppStateStatus } from 'react-native';
import { clearGamepadOutputs } from '@/lib/gamepad/manager';

let installed = false;

export function installInputRelease() {
  if (installed) return;
  installed = true;
  let prev: AppStateStatus = AppState.currentState;
  AppState.addEventListener('change', (next) => {
    if (prev === 'active' && next !== 'active') clearGamepadOutputs();
    prev = next;
  });
}
