import { useEffect, useRef, useState } from 'react';
import { View, StyleSheet, Platform, Pressable, Modal as RNModal, ScrollView, useWindowDimensions } from 'react-native';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { Text } from 'react-native';
import NetInfo from '@react-native-community/netinfo';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { WifiStatus } from '@/components/WifiStatus';
import { useSignalStrength, signalBars } from '@/lib/useSignalStrength';
import { useNetRate } from '@/lib/useNetRate';
import { fmtRate } from '@/lib/netMeter';
import { useCompactW, useTinyW } from '@/lib/layout';
import { useSettings } from '@/store/settings';
import { useRobot } from '@/store/robot';
import { useWifi } from '@/store/wifi';
import { isDesktop } from '@/lib/desktopBridge';
import { t } from '@/lib/i18n';
import { connection } from '@/lib/connection';
import { linkRttMs, routeLabel } from '@/lib/connectionRoute';

function RemoteLinkCard() {
  const { c, fonts, radius } = useTheme();
  const route = useRobot((s) => s.route);
  const proto = useRobot((s) => s.relayProto);
  const [rtt, setRtt] = useState<number | null>(null);
  useEffect(() => {
    const tick = () => connection.stats().then((st) => setRtt(linkRttMs(st))).catch(() => {});
    tick();
    const id = setInterval(tick, 2000);
    return () => clearInterval(id);
  }, []);
  const rl = routeLabel(route, proto);
  const parts = [t('원격'), rl && t(rl), rtt != null ? `${t('지연')} ${rtt}ms` : null].filter(Boolean);
  return (
    <View style={{ marginBottom: 14 }}>
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginBottom: 8 }}>{t('로봇 연결')}</Text>
      <View style={{ padding: 12, borderWidth: 1, backgroundColor: c.bg, borderColor: c.line, borderRadius: radius.md }}>
        <Text style={{ color: c.greenTx, fontSize: 13, fontWeight: '600' }}>{t('원격으로 연결됨')}</Text>
        <Text style={{ color: c.dim, fontSize: 10, marginTop: 2, fontFamily: fonts.mono }}>{parts.join(' · ')}</Text>
      </View>
    </View>
  );
}

const BAR_H = [4, 7, 10, 13, 16];

const cs = StyleSheet.create({
  chipTx: { fontSize: 10, fontWeight: '600', lineHeight: 12, includeFontPadding: false },
});

function useCurrentSsid(): string | null {
  const desktopSsid = useWifi((s) => s.current?.ssid ?? null);
  const [droidSsid, setDroidSsid] = useState<string | null>(null);
  useEffect(() => {
    if (Platform.OS !== 'android') { useWifi.getState().refreshCurrent(); return; }
    return NetInfo.addEventListener((st) => {
      const d: any = st.type === 'wifi' ? st.details : null;
      setDroidSsid(d?.ssid && d.ssid !== '<unknown ssid>' ? d.ssid : null);
    });
  }, []);
  return Platform.OS === 'android' ? droidSsid : desktopSsid;
}

export function SignalStrength({ showSsid = true, showPct = true }: { showSsid?: boolean; showPct?: boolean } = {}) {
  const { c, fonts, radius } = useTheme();
  const pct = useSignalStrength();
  const compact = useCompactW();
  const tiny = useTinyW();
  const bars = pct == null ? 0 : signalBars(pct);
  const tone = pct == null ? c.dim : pct < 20 ? c.redbright : pct < 40 ? c.amber : c.greenTx2;
  const ssid = useCurrentSsid();
  const conn = useRobot((s) => s.conn);
  const remote = useRobot((s) => s.via === 'rendezvous' && s.conn === 'connected');
  const rate = useNetRate(conn === 'connected');
  const [open, setOpen] = useState(false);
  const wifiSupported = isDesktop() || Platform.OS === 'android';
  const { width: winW, height: winH } = useWindowDimensions();
  const chipRef = useRef<View>(null);
  const [anchorX, setAnchorX] = useState(8);
  const PANEL_W = Math.min(420, winW - 16);
  const openPanel = () => {
    const el = chipRef.current as unknown as { measureInWindow?: (cb: (x: number) => void) => void } | null;
    if (el?.measureInWindow) el.measureInWindow((x) => { setAnchorX(Math.max(8, Math.min(x, winW - PANEL_W - 8))); setOpen(true); });
    else setOpen(true);
  };
  return (
    <>
      <View ref={chipRef} collapsable={false}>
      <Tappable onPress={openPanel}
        style={[styles.chip, { backgroundColor: 'transparent', borderColor: pct != null && pct < 20 ? 'rgba(231,51,28,0.55)' : c.glassLine, borderRadius: radius.sm }]}>
        {showSsid && ssid ? (
          <Text style={[cs.chipTx, { color: c.muted, maxWidth: 110 }]} numberOfLines={1}>{ssid}</Text>
        ) : null}
        {!tiny && <View style={styles.bars}>
          {BAR_H.map((h, i) => (
            <View key={i} style={{ width: 3, height: h, borderRadius: 1, backgroundColor: i < bars ? tone : c.line }} />
          ))}
        </View>}
        {showPct && !compact && (
          <Text style={[cs.chipTx, { color: tone, fontFamily: fonts.mono }]}>
            {pct == null ? '—' : `${Math.round(pct)}%`}
          </Text>
        )}
        {rate != null && (
          <Text style={[cs.chipTx, { color: c.muted, fontFamily: fonts.mono }]}>
            {fmtRate(rate)}
          </Text>
        )}
      </Tappable>
      </View>
      <RNModal supportedOrientations={MODAL_ORIENTATIONS} visible={open} transparent animationType="fade" onRequestClose={() => setOpen(false)}>
        <Pressable style={StyleSheet.absoluteFill} onPress={() => setOpen(false)}>
          <Pressable onPress={() => {}}
            style={[styles.wifiPanel, { left: anchorX, width: PANEL_W, maxHeight: winH - 60 - 12, backgroundColor: c.panel, borderColor: c.line }]}>
            <ScrollView showsVerticalScrollIndicator style={{ flexShrink: 1 }} contentContainerStyle={{ padding: 14 }}>
              {remote && <RemoteLinkCard />}
              {wifiSupported ? (
                <WifiStatus embedded />
              ) : (
                <Text style={{ color: c.dim, fontSize: 11, lineHeight: 16 }}>
                  {t('이 기기에서는 앱 안에서 Wi-Fi 를 검색하거나 연결할 수 없습니다. 시스템 설정에서 로봇 네트워크에 연결한 뒤 돌아와 주세요.')}
                </Text>
              )}
            </ScrollView>
          </Pressable>
        </Pressable>
      </RNModal>
    </>
  );
}

let _audioCtx: AudioContext | null = null;
export function warnBeep() {
  if (Platform.OS !== 'web' || typeof AudioContext === 'undefined') return;
  try {
    _audioCtx ??= new AudioContext();
    const ctx = _audioCtx;
    for (let i = 0; i < 2; i++) {
      const osc = ctx.createOscillator();
      const gain = ctx.createGain();
      osc.type = 'square';
      osc.frequency.value = 880;
      gain.gain.setValueAtTime(0.08, ctx.currentTime + i * 0.22);
      gain.gain.exponentialRampToValueAtTime(0.001, ctx.currentTime + i * 0.22 + 0.15);
      osc.connect(gain).connect(ctx.destination);
      osc.start(ctx.currentTime + i * 0.22);
      osc.stop(ctx.currentTime + i * 0.22 + 0.16);
    }
  } catch { }
}

export function LowSignalBanner() {
  const { c, radius } = useTheme();
  const pct = useSignalStrength();
  const enabled = useSettings((s) => s.lowConnWarnEnabled);
  const threshold = useSettings((s) => s.lowConnThreshold);
  const beep = useSettings((s) => s.notifyBeep);
  const [dismissed, setDismissed] = useState(false);
  const low = enabled && pct != null && pct < threshold;
  const wasLow = useRef(false);

  useEffect(() => {
    if (!low) { wasLow.current = false; setDismissed(false); return; }
    if (!wasLow.current) { wasLow.current = true; if (beep) warnBeep(); }
  }, [low, beep]);

  if (!low || dismissed) return null;
  return (
    <View style={[styles.banner, { backgroundColor: 'rgba(231,51,28,0.14)', borderColor: 'rgba(231,51,28,0.55)', borderRadius: radius.md }]}>
      <Icon name="wifi" size={14} color={c.redbright} />
      <Text style={{ color: c.redbright, fontSize: 11.5, fontWeight: '700' }}>{t('네트워크 신호 약함')}</Text>
      <Text style={{ color: c.muted, fontSize: 10.5 }}>
        {Math.round(pct!)}% ({t('임계')} {threshold}%) — {t('로봇 제어가 지연되거나 끊길 수 있습니다')}
      </Text>
      <Pressable onPress={() => setDismissed(true)} hitSlop={8} style={{ marginLeft: 4 }}>
        <Icon name="x" size={13} color={c.muted} />
      </Pressable>
    </View>
  );
}

const styles = StyleSheet.create({
  chip: { flexDirection: 'row', alignItems: 'center', gap: 5, height: 32, paddingHorizontal: 8, borderWidth: 1 },
  wifiPanel: { position: 'absolute', top: 60, borderWidth: 1, borderRadius: 14, overflow: 'hidden' },
  bars: { flexDirection: 'row', alignItems: 'flex-end', gap: 2, height: 16 },
  banner: {
    flexDirection: 'row', alignItems: 'center', gap: 8,
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
  },
});
