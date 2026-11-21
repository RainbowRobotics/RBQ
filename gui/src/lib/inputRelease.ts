import { AppState, Platform, type AppStateStatus } from 'react-native';
import { clearGamepadOutputs } from '@/lib/gamepad/manager';
import { connection } from '@/lib/connection';

let installed = false;

export function installInputRelease() {
  if (installed) return;
  installed = true;
  let prev: AppStateStatus = AppState.currentState;
  AppState.addEventListener('change', (next) => {
    if (prev === 'active' && next !== 'active') { clearGamepadOutputs(); connection.releaseAll(); }
    if (next === 'background' && Platform.OS === 'ios') connection.suspendRelays();
    if (prev !== 'active' && next === 'active') connection.resecure();
    prev = next;
  });
}
