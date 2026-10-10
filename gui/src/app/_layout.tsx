import '@/modules/installed';
import { LocalFilePicker } from '@/components/LocalFilePicker';
import { installDeckTouchPassthrough } from '@/lib/deckTouch';
import { useEffect, useRef, useState } from 'react';
import { View, Text, Pressable, Platform } from 'react-native';
import { Stack, usePathname } from 'expo-router';
import { StatusBar } from 'expo-status-bar';
import { GestureHandlerRootView } from 'react-native-gesture-handler';
import { SafeAreaProvider } from 'react-native-safe-area-context';

import { activateKeepAwakeAsync, deactivateKeepAwake } from 'expo-keep-awake';
import { useFonts } from 'expo-font';
import { RB_FONTS, RB_FONT_FACE_CSS } from '@/rb/fonts';
import { RbProvider } from '@/rb/theme';
import { immersiveNavBar } from '@/lib/immersive';

import { ThemeProvider, useTheme } from '@/theme';
import { connection } from '@/lib/connection';
import { restoreLastTarget, reconnectCurrent, disconnectAll } from '@/lib/connectNow';
import { restoreAccountSession } from '@/lib/accountSession';
import { decideEntry } from '@/lib/entryGate';
import { isDemo } from '@/lib/demoFlag';
import { useRobots } from '@/store/robots';
import { startLanUpgradeWatch } from '@/lib/lanUpgrade';
import { initPush, syncPush } from '@/lib/push';
import { useAccount } from '@/store/account';
import { startGamepadManager } from '@/lib/gamepad/manager';
import { installInputRelease } from '@/lib/inputRelease';
import { installTouchPointerPolyfill, installMouseDragScrollShim, installDeckCursorHide } from '@/lib/touchPointerPolyfill';
import { installOsk } from '@/lib/osk/install';
import { installRobotIdentity } from '@/lib/robotIdentity';
import { installRobotSettingsSync } from '@/lib/robotSettingsSync';
import { installNetWatch } from '@/lib/netWatch';
import { getPlatformInfo } from '@/lib/platformInfo';
import { startKeyboardManager } from '@/lib/keyboard/manager';
import { installGlTeardownGuard } from '@/lib/glTeardownGuard';
import { useRobot } from '@/store/robot';
import { useSpectating, useSpectatingDriver } from '@/lib/spectating';
import { useViewport } from '@/store/viewport';
import { useSettings, useSettingsHydrated } from '@/store/settings';
import { useLang } from '@/store/lang';
import { t } from '@/lib/i18n';
import { useWebrtcStore } from '@/lib/webrtcClient';
import { ProxyStatusBadge } from '@/components/ProxyStatusBadge';
import { RobotAuthPrompt } from '@/components/RobotAuthPrompt';
import { cloudConfigured } from '@/lib/logUploadCommon';
import { warnBeep } from '@/components/SignalStrength';
import { BootScreen, BOOT_BG } from '@/components/BootScreen';
import { isDesktop, restartProxy, setUiZoom, installFullscreenHotkey } from '@/lib/desktopBridge';
import { syncProxyTarget } from '@/lib/proxyTarget';
import { goTop } from '@/lib/nav';

installGlTeardownGuard();
installRobotIdentity();
installRobotSettingsSync();

function ConnErrorBanner() {
  const err = useRobot((s) => s.connError);
  const clear = useRobot((s) => s.setConnError);
  if (!err) return null;
  return (
    <View pointerEvents="box-none" style={{ position: 'absolute', top: 10, left: 0, right: 0, alignItems: 'center', zIndex: 999 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 10, maxWidth: 640, paddingVertical: 10, paddingHorizontal: 14,
        backgroundColor: 'rgba(231,51,28,0.92)', borderRadius: 10, borderWidth: 1, borderColor: 'rgba(255,255,255,0.35)' }}>
        <Text style={{ color: '#fff', fontSize: 12, fontWeight: '600', flexShrink: 1 }}>⚠ {t(err)}</Text>
        <Pressable onPress={() => clear(null)} hitSlop={8}>
          <Text style={{ color: '#fff', fontSize: 14, fontWeight: '800' }}>✕</Text>
        </Pressable>
      </View>
    </View>
  );
}

function TakeoverConflictModal() {
  const open = useRobot((s) => s.takeoverConflict);
  const ip = useRobot((s) => s.ip);
  const visionIp = useRobot((s) => s.visionIp);
  if (!open) return null;
  const close = () => useRobot.getState().setTakeoverConflict(false);
  return (
    <View style={{ position: 'absolute', inset: 0 as any, alignItems: 'center', justifyContent: 'center', backgroundColor: 'rgba(0,0,0,0.45)', zIndex: 1000 }}>
      <View style={{ width: 340, borderRadius: 14, padding: 18, backgroundColor: '#1d2126', borderWidth: 1, borderColor: 'rgba(255,255,255,0.15)' }}>
        <Text style={{ color: '#fff', fontSize: 14, fontWeight: '700', marginBottom: 8 }}>{t('연결 충돌이 반복되고 있습니다')}</Text>
        <Text style={{ color: 'rgba(255,255,255,0.7)', fontSize: 12, lineHeight: 17, marginBottom: 14 }}>
          {t('같은 로봇에 다른 조종기(앱/RBQGUI)가 접속해 연결을 서로 뺏고 있습니다. 연결을 가져오면 상대가 끊깁니다.')}
        </Text>
        <View style={{ flexDirection: 'row', gap: 10 }}>
          <Pressable onPress={() => { close(); }}
            style={{ flex: 1, height: 40, borderRadius: 10, alignItems: 'center', justifyContent: 'center', borderWidth: 1, borderColor: 'rgba(255,255,255,0.25)' }}>
            <Text style={{ color: '#fff', fontSize: 13, fontWeight: '600' }}>{t('대기 (연결하지 않음)')}</Text>
          </Pressable>
          <Pressable onPress={() => { close(); reconnectCurrent(); }}
            style={{ flex: 1, height: 40, borderRadius: 10, alignItems: 'center', justifyContent: 'center', backgroundColor: '#e7331c' }}>
            <Text style={{ color: '#fff', fontSize: 13, fontWeight: '700' }}>{t('다시 가져오기')}</Text>
          </Pressable>
        </View>
      </View>
    </View>
  );
}

function SpectatorMode() {
  const show = useSpectating();
  const noSlot = useWebrtcStore((s) => s.status === '영상 자리 없음');
  const ddOpen = useViewport((s) => s.ddOpen || s.barPopOpen);
  const path = usePathname();
  const pillOnly = path === '/hub' || path === '/settings' || path === '/log';
  const [confirm, setConfirm] = useState<null | 'menu' | 'auto'>(null);
  useEffect(() => { if (!show) setConfirm(null); }, [show]);
  if (!show) return null;
  const btn = (bg: string, border: string): any => ({
    minWidth: 220, height: 46, borderRadius: 12, alignItems: 'center', justifyContent: 'center',
    backgroundColor: bg, borderWidth: 1, borderColor: border,
  });
  if (confirm === 'menu') {
    return (
      <View style={{ position: 'absolute', left: 0, right: 0, bottom: 0, top: SPECTATE_TOP, backgroundColor: 'rgba(0,0,0,0.6)', zIndex: 950, alignItems: 'center', justifyContent: 'center', gap: 12 }}>
        <Text style={{ color: '#fff', fontSize: 16, fontWeight: '700' }}>{t('제어권을 가져올까요?')}</Text>
        <Text style={{ color: 'rgba(255,255,255,0.75)', fontSize: 12, marginBottom: 6 }}>{t('지금 조종하는 기기는 관전으로 바뀝니다')}</Text>
        <Pressable onPress={() => { setConfirm(null); connection.claimOwnership(); }} style={btn('rgba(63,185,80,0.25)', 'rgba(63,185,80,0.8)')}>
          <Text style={{ color: '#7ee787', fontSize: 16, fontWeight: '800' }}>{t('제어권 가져오기')}</Text>
        </Pressable>
        <Pressable onPress={() => setConfirm('auto')} style={btn('rgba(255,255,255,0.08)', 'rgba(255,255,255,0.35)')}>
          <Text style={{ color: '#fff', fontSize: 13, fontWeight: '700' }}>▶ {t('자동 기동')}</Text>
        </Pressable>
        <Pressable onPress={() => setConfirm(null)} style={btn('transparent', 'rgba(255,255,255,0.3)')}>
          <Text style={{ color: '#fff', fontSize: 13, fontWeight: '600' }}>{t('계속 관전')}</Text>
        </Pressable>
      </View>
    );
  }
  if (confirm === 'auto') {
    return (
      <View style={{ position: 'absolute', left: 0, right: 0, bottom: 0, top: SPECTATE_TOP, backgroundColor: 'rgba(0,0,0,0.7)', zIndex: 950, alignItems: 'center', justifyContent: 'center' }}>
        <View style={{ width: 340, borderRadius: 14, padding: 20, backgroundColor: '#1d2126', borderWidth: 1, borderColor: 'rgba(63,185,80,0.6)' }}>
          <Text style={{ color: '#fff', fontSize: 15, fontWeight: '800', marginBottom: 8 }}>{t('자동 기동을 실행할까요?')}</Text>
          <Text style={{ color: 'rgba(255,255,255,0.75)', fontSize: 12, lineHeight: 18, marginBottom: 16 }}>
            {t('로봇이 앉은(SIT) 상태로 평평한 바닥에 있고, 발과 무릎이 지면에 닿아 있는지 확인하세요. 실행 시 이 기기가 제어권을 가져옵니다.')}
          </Text>
          <View style={{ flexDirection: 'row', gap: 10 }}>
            <Pressable onPress={() => setConfirm(null)}
              style={{ flex: 1, height: 42, borderRadius: 10, alignItems: 'center', justifyContent: 'center', borderWidth: 1, borderColor: 'rgba(255,255,255,0.3)' }}>
              <Text style={{ color: '#fff', fontSize: 13, fontWeight: '600' }}>{t('취소')}</Text>
            </Pressable>
            <Pressable
              onPress={() => { setConfirm(null); connection.claimOwnership().then(() => connection.sendMotion('auto_start')); }}
              style={{ flex: 1, height: 42, borderRadius: 10, alignItems: 'center', justifyContent: 'center', backgroundColor: 'rgba(63,185,80,0.35)', borderWidth: 1, borderColor: 'rgba(63,185,80,0.8)' }}>
              <Text style={{ color: '#7ee787', fontSize: 13, fontWeight: '800' }}>{t('자동 기동')}</Text>
            </Pressable>
          </View>
        </View>
      </View>
    );
  }
  const pill = (
      <Pressable onPress={() => setConfirm('menu')} accessibilityLabel={t('관전')}
        style={{ marginTop: 8, flexDirection: 'row', alignItems: 'center', gap: 8, height: 40, paddingHorizontal: 16, borderRadius: 20,
          backgroundColor: 'rgba(20,24,28,0.88)', borderWidth: 1, borderColor: 'rgba(242,177,74,0.8)' }}>
        <Text style={{ color: '#f2b14a', fontSize: 14, fontWeight: '800' }}>{t('관전')}</Text>
        <Text style={{ color: 'rgba(255,255,255,0.8)', fontSize: 12 }}>{t(noSlot ? '보는 기기가 많아 영상 대기 중 · 눌러서 제어권 가져오기' : '다른 기기가 조종 중 · 눌러서 제어권 가져오기')}</Text>
      </Pressable>
  );
  if (ddOpen) return null;
  if (pillOnly) return (
    <View pointerEvents="box-none" style={{ position: 'absolute', left: 0, right: 0, top: SPECTATE_TOP, zIndex: 950, alignItems: 'center' }}>
      {pill}
    </View>
  );
  return (
    <View
      style={{ position: 'absolute', left: 0, right: 0, bottom: 0, top: SPECTATE_TOP, zIndex: 950, alignItems: 'center' }}>
      {pill}
    </View>
  );
}
const SPECTATE_TOP = 60;

function BatteryLowBeep() {
  const battPct = useRobot((s) => s.battPct);
  const conn = useRobot((s) => s.conn);
  const enabled = useSettings((s) => s.notifyBeep && s.batteryLowBeep);
  const wasLow = useRef(false);
  useEffect(() => {
    const low = conn === 'connected' && battPct != null && battPct < 15;
    if (!low) { wasLow.current = false; return; }
    if (!wasLow.current) { wasLow.current = true; if (enabled) warnBeep(); }
  }, [battPct, conn, enabled]);
  return null;
}

function RbBridge({ children }: { children: React.ReactNode }) {
  const { name } = useTheme();
  useEffect(() => {
    if (Platform.OS !== 'web' || document.getElementById('rb-app-fonts')) return;
    const el = document.createElement('style'); el.id = 'rb-app-fonts'; el.textContent = RB_FONT_FACE_CSS; document.head.appendChild(el);
  }, []);
  return <RbProvider theme={name}>{children}</RbProvider>;
}

export default function RootLayout() {
  useFonts(RB_FONTS);
  useSpectatingDriver();
  const ip = useRobot((s) => s.ip);
  const visionIp = useRobot((s) => s.visionIp);
  const lang = useLang((s) => s.lang);
  const storeReady = useSettingsHydrated();
  const pushAlerts = useSettings((st) => st.pushAlerts);
  const hasAccount = useAccount((st) => !!st.account);
  const accountRestored = useAccount((st) => st.restored);
  useEffect(() => {
    if (storeReady && accountRestored) void syncPush(pushAlerts, hasAccount);
  }, [storeReady, accountRestored, pushAlerts, hasAccount]);
  const [minElapsed, setMinElapsed] = useState(false);
  useEffect(() => { const t = setTimeout(() => setMinElapsed(true), 900); return () => clearTimeout(t); }, []);
  const hydrated = storeReady && minElapsed;
  const rvKey = useSettings((s) =>
    isDesktop() ? [s.connProfile, s.rendezvousUrl, s.robotId, s.webrtcToken].join('|') : '');
  const pathname = usePathname();
  const isGallery = pathname === '/widgets' || pathname === '/rb-mirror' || pathname === '/rb-gallery' || pathname === '/rb-compare';
  useEffect(() => {
    if (isGallery) return;
    let cancelled = false;
    (async () => {
      if (isDesktop()) {
        const st = useSettings.getState();
        const rv = st.connProfile === 'wan' && (st.rendezvousUrl ?? '').trim() !== ''
                   && (st.robotId ?? '').trim() !== ''
          ? { rendezvousUrl: st.rendezvousUrl, robotId: st.robotId, webrtcToken: st.webrtcToken }
          : undefined;
        await restartProxy(ip, visionIp, rv).catch((e) => console.warn('proxy restart 실패', e));
      } else {
        await syncProxyTarget(ip, visionIp);
      }
      if (cancelled) return;
    })();
    return () => { cancelled = true; };
  }, [ip, visionIp, rvKey, isGallery]);

  useEffect(() => {
    void restoreAccountSession();
    initPush().catch(() => {});
    if (!isGallery && isDemo()) restoreLastTarget();
    return () => { disconnectAll(); };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const gated = useRef(false);
  useEffect(() => {
    if (gated.current || !hydrated || !accountRestored || isGallery) return;
    gated.current = true;
    const go = () => {
      const st = useSettings.getState();
      useRobots.getState().migrate(st.lanIp, st.wanIp, useRobot.getState().ip, st.webrtcToken);
      const forceEntry = typeof location !== 'undefined' && new URLSearchParams(location.search).get('entry') === '1';
      const to = decideEntry({ hasRobots: useRobots.getState().local.length > 0, hasAccount: !!useAccount.getState().account, isDemo: isDemo() && !forceEntry, cloud: cloudConfigured });
      if (to !== '/') goTop(to);
    };
    if (useRobots.persist.hasHydrated()) go(); else useRobots.persist.onFinishHydration(go);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [hydrated, accountRestored, isGallery]);

  useEffect(() => {
    const z = useSettings.getState().uiZoom;
    if (isDesktop() && z != null) setUiZoom(z).catch(() => {});
  }, []);

  useEffect(() => (isDemo() ? undefined : startLanUpgradeWatch()), []);

  useEffect(() => { startGamepadManager(); installInputRelease(); }, []);

  const rbRoute = pathname.startsWith('/rb-');
  useEffect(() => {
    if (typeof document === 'undefined' || rbRoute) return;
    const style = document.createElement('style');
    style.textContent =
      '::-webkit-scrollbar{width:4px;height:4px}' +
      '::-webkit-scrollbar-thumb{background:rgba(128,128,128,0.45);border-radius:2px}' +
      '::-webkit-scrollbar-track{background:transparent}';
    document.head.appendChild(style);
    return () => style.remove();
  }, [rbRoute]);

  useEffect(() => { startKeyboardManager(); }, []);

  useEffect(() => {
    if (typeof document === 'undefined') return;
    const block = (e: Event) => e.preventDefault();
    document.addEventListener('contextmenu', block);
    const style = document.createElement('style');
    style.textContent =
      "@font-face{font-family:Pretendard;src:url('/fonts/Pretendard-Regular.woff2') format('woff2');font-weight:400;font-display:swap}" +
      "@font-face{font-family:Pretendard;src:url('/fonts/Pretendard-Bold.woff2') format('woff2');font-weight:700;font-display:swap}" +
      ':root{--font-display:Pretendard,system-ui,sans-serif}';
    document.head.appendChild(style);
    getPlatformInfo();
    try {
      for (const sheet of Array.from(document.styleSheets)) {
        let rules: CSSRuleList;
        try { rules = sheet.cssRules; } catch { continue; }
        for (const r of Array.from(rules)) {
          const st = (r as CSSStyleRule).style;
          if (st?.fontFamily && st.fontFamily.includes('-apple-system') && !st.fontFamily.includes('Pretendard')) {
            st.fontFamily = `Pretendard, ${st.fontFamily}`;
          }
        }
      }
    } catch { }
    installTouchPointerPolyfill();
    installDeckTouchPassthrough();
    installMouseDragScrollShim();
    installOsk();
    installDeckCursorHide();
    const offFs = installFullscreenHotkey();
    const offNet = installNetWatch();
    return () => { document.removeEventListener('contextmenu', block); offFs(); offNet(); };
  }, []);

  const wakeLock = useSettings((s) => s.wakeLock);
  useEffect(() => {
    if (wakeLock) activateKeepAwakeAsync().catch(() => {});
    else deactivateKeepAwake().catch(() => {});
  }, [wakeLock]);

  const gpPublishHz = useSettings((s) => s.gpPublishHz);
  useEffect(() => { connection.setPublishHz(gpPublishHz); }, [gpPublishHz]);

  useEffect(() => immersiveNavBar(), []);

  return (
    <GestureHandlerRootView style={{ flex: 1, backgroundColor: BOOT_BG }}>
      <SafeAreaProvider>
        <ThemeProvider initial="light">
        <RbBridge>
          <StatusBar hidden />
          {!hydrated ? <BootScreen /> : (
          <Stack key={lang} screenOptions={{ headerShown: false, animation: 'none', contentStyle: { backgroundColor: '#000' } }}>
            <Stack.Screen name="index" options={{ gestureEnabled: false }} />
            <Stack.Screen name="hub" />
            <Stack.Screen name="pin" />
            <Stack.Screen name="add-robot" />
            <Stack.Screen name="dashboard" />
            <Stack.Screen name="log" />
            <Stack.Screen name="media" />
            <Stack.Screen name="settings" />
            <Stack.Screen name="maintenance" />
            <Stack.Screen name="arm" />
            <Stack.Screen name="arm-door" />
            <Stack.Screen name="gamepad" />
          </Stack>
          )}
          <SpectatorMode />
          <TakeoverConflictModal />
          <LocalFilePicker />
          <ProxyStatusBadge />
          <RobotAuthPrompt />
          <BatteryLowBeep />
          <ConnErrorBanner />
        </RbBridge>
        </ThemeProvider>
      </SafeAreaProvider>
    </GestureHandlerRootView>
  );
}
