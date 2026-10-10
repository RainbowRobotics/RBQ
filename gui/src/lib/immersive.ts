import { AppState, Platform } from 'react-native';
import * as NavigationBar from 'expo-navigation-bar';

async function apply(): Promise<void> {
  if (Platform.OS !== 'android') return;
  try {
    await NavigationBar.setVisibilityAsync('hidden');
  } catch {
  }
}

export function immersiveNavBar(): () => void {
  if (Platform.OS !== 'android') return () => {};
  void apply();
  const sub = AppState.addEventListener('change', (st) => {
    if (st === 'active') void apply();
  });
  return () => sub.remove();
}
