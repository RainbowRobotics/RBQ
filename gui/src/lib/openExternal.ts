import { Linking } from 'react-native';
import { isDesktop } from '@/lib/desktopBridge';

export function openExternal(url: string): void {
  if (isDesktop()) { fetch(`/open-url?u=${encodeURIComponent(url)}`).catch(() => {}); return; }
  Linking.openURL(url).catch(() => {});
}
