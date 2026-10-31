import { isDesktop } from '@/lib/desktopBridge';

export function installDeckTouchPassthrough(): void {
  if (typeof window === 'undefined' || !isDesktop()) return;
  const apply = () => { fetch('/touch-native').catch(() => {}); };
  apply();
  window.addEventListener('focus', apply);
  document.addEventListener('visibilitychange', () => { if (document.visibilityState === 'visible') apply(); });
  let last = 0;
  document.addEventListener('pointerdown', (e) => {
    if (e.pointerType !== 'mouse' || Date.now() - last < 5000) return;
    last = Date.now();
    apply();
  }, true);
}
