import { useEffect, useState } from 'react';
import { BackHandler, Platform } from 'react-native';
import { router, usePathname } from 'expo-router';
import { backAction, padIsInput, type BackInput } from '@/lib/backNav';
import { goTop } from '@/lib/nav';
import { setPadBack, setPadNav } from '@/lib/gamepad/manager';
import { padNav } from '@/lib/focusNav';
import { isDesktop } from '@/lib/desktopBridge';
import { QuitConfirm } from '@/components/QuitButton';

let padHold = 0;
export function usePadBackOff() {
  useEffect(() => { padHold++; return () => { padHold--; }; }, []);
}

const canQuit = Platform.OS === 'android' || isDesktop();

export function BackNav({ standalone }: { standalone: boolean }) {
  const path = usePathname();
  const [ask, setAsk] = useState(false);
  useEffect(() => {
    const run = (input: BackInput) => {
      const a = backAction(input, path, router.canGoBack(), canQuit, standalone);
      if (a === 'pop') router.back();
      else if (a === 'hub') goTop('/hub');
      else if (a === 'quit') setAsk(true);
      return a !== null;
    };
    if (Platform.OS === 'android') {
      const sub = BackHandler.addEventListener('hardwareBackPress', () => !router.canGoBack() && run('hw'));
      return () => sub.remove();
    }
    if (Platform.OS !== 'web') return;
    const modalOpen = () => !!document.querySelector('[aria-modal="true"]');
    const onKey = (e: KeyboardEvent) => {
      if (e.key !== 'Escape' || e.defaultPrevented || modalOpen()) return;
      const el = e.target as HTMLElement | null;
      if (el?.isContentEditable || /^(INPUT|TEXTAREA|SELECT)$/.test(el?.tagName ?? '')) return;
      run('esc');
    };
    window.addEventListener('keydown', onKey);
    setPadBack(() => { if (!padHold && !modalOpen()) run('pad'); });
    setPadNav(padIsInput(path) ? null : (cmd) => { if (!padHold) padNav(cmd); });
    return () => { window.removeEventListener('keydown', onKey); setPadBack(null); setPadNav(null); };
  }, [path, standalone]);
  return ask ? <QuitConfirm onClose={() => setAsk(false)} /> : null;
}
