import { Platform } from 'react-native';
import { Paths } from 'expo-file-system';

export function simWorkerDir(): string {
  if (Platform.OS === 'android') return 'file:///android_asset/mujoco';
  const root = Paths.bundle.uri.replace(/\/$/, '');
  return `${root}/mujoco`;
}

export function simWorkerUrl(): string {
  return `${simWorkerDir()}/worker.html`;
}
