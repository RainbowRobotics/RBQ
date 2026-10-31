import { useEffect, useState } from 'react';
import { View, Text, Image, StyleSheet, Animated, Easing, useWindowDimensions, type StyleProp, type ViewStyle } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import { useRobotKind } from '@/modules/registry';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Battery, DeviceBattery, BatteryPopover } from '@/components/Battery';
import { SignalStrength, LowSignalBanner } from '@/components/SignalStrength';
import { useViewport } from '@/store/viewport';
import { topBarButtons } from '@/modules/registry';
import { QuitButton } from '@/components/QuitButton';
import { useTinyW, EDGE, SOURCE_DD_W } from '@/lib/layout';
import { DockingBanner } from '@/components/DockingBanner';
import { BoardFirmwareBanner } from '@/components/BoardFirmwareBanner';
import { AutoUpdateBanner } from '@/components/AutoUpdate';
import { CameraCalibBanner } from '@/components/CameraCalib';
import { LogToasts } from '@/components/LogToasts';
import { HealthBanner } from '@/components/HealthBanner';
import { LogBadge } from '@/components/LogBadge';
import { EStopModal } from '@/components/control/overlays';
import { Tappable } from '@/components/anim';
import { useRobot } from '@/store/robot';
import { actions } from '@/lib/rest';
import { useTelemetry } from '@/store/telemetry';
import { robotCharging } from '@/lib/charge';
import { useFeatureWheel, useFeatureCanFd } from '@/store/capability';
import { connection } from '@/lib/connection';
import { wheelGaitName } from '@/lib/wheelGait';
import { gaitTone, type GaitTone } from '@/lib/gaitTone';
import { getLang } from '@/store/lang';
import { t } from '@/lib/i18n';
import { LogoMenu } from '@/components/LogoMenu';


const ESTOP_GRACE_MS = 8000;

export function EStop({ onPress }: { onPress?: () => void }) {
  const { c, radius } = useTheme();
  const [ask, setAsk] = useState(false);
  const conn = useRobot((s) => s.conn);
  const droppedAt = useRobot((s) => s.droppedAt);
  const [, tick] = useState(0);
  const inGrace = conn === 'connecting' && droppedAt != null && Date.now() - droppedAt < ESTOP_GRACE_MS;
  useEffect(() => {
    if (!inGrace) return;
    const t = setTimeout(() => tick((n) => n + 1), ESTOP_GRACE_MS - (Date.now() - (droppedAt ?? 0)) + 50);
    return () => clearTimeout(t);
  }, [inGrace, droppedAt]);
  const enabled = conn === 'connected' || inGrace;
  const kind = useRobotKind();
  const tiny = useTinyW();
  return (
    <>
    {ask && <EStopModal onClose={() => setAsk(false)} />}
    <Tappable disabled={!enabled} onPress={onPress ?? (() => setAsk(true))} style={!enabled && { opacity: 0.4 }}
      accessibilityLabel={kind ? t(kind.estop.label) : 'E-STOP'}>
      <LinearGradient
        colors={[c.dangerA, c.dangerB]}
        style={[styles.estop, { borderColor: c.dangerLine, borderRadius: radius.md }, tiny && { paddingHorizontal: 9 }]}
      >
        <Icon name="estop" size={18} color={c.onAccent} />
        {(!tiny || kind) && <Text style={[styles.estopTxt, { color: c.onAccent }]}>{kind ? t(kind.estop.label) : 'E-STOP'}</Text>}
      </LinearGradient>
    </Tappable>
    </>
  );
}

export function Logo({beforeNavigate}:{beforeNavigate?:(next:()=>void)=>void} = {}) {
  const [open, setOpen] = useState(false);
  return (
    <>
      <Tappable onPress={() => setOpen(true)} accessibilityLabel={t('메뉴')}>
        <Image source={require('@/assets/images/rb-logo.png')} style={styles.logo} />
      </Tappable>
      {open && <LogoMenu onClose={() => setOpen(false)} beforeNavigate={beforeNavigate} />}
    </>
  );
}

export function Pod({ children, style }: { children: React.ReactNode; style?: StyleProp<ViewStyle> }) {
  const { c, radius } = useTheme();
  return <View style={[styles.pod, { backgroundColor: c.glass, borderColor: c.glassLine, borderRadius: radius.md }, style]}>{children}</View>;
}

export function TopRight({ onEStop, hideRobot, onWidth }: { onEStop?: () => void; hideRobot?: boolean; onWidth?: (w: number) => void }) {
  const [batt, setBatt] = useState(false);
  const conn = useRobot((s) => s.conn);
  const battPct = useRobot((s) => s.battPct);
  const fd = useFeatureCanFd();
  const robotChg = useTelemetry((s) => robotCharging(fd, s.packAvg, s.chargeAvg));
  return (
    <View style={styles.right} onLayout={onWidth ? (e) => onWidth(e.nativeEvent.layout.width) : undefined}>
      <Pod style={styles.noShrink}>
        <LogBadge />
        {!hideRobot && (
          <Tappable onPress={() => setBatt(true)} accessibilityLabel={t('배터리 상세')} style={styles.batt}>
            <Battery pct={conn === 'connected' ? battPct : null} label={t('로봇')} charging={robotChg} />
          </Tappable>
        )}
        <DeviceBattery />
        {topBarButtons.map((B, i) => <B key={i} />)}
      </Pod>
      <EStop onPress={onEStop} />
      <QuitButton />
      {batt && <BatteryPopover onClose={() => setBatt(false)} />}
    </View>
  );
}

export function HlcChip() {
  const { c, fonts, radius } = useTheme();
  const extJoy = useTelemetry((s) => !!s.robot?.extJoy);
  const ip = useRobot((s) => s.ip);
  const isMine = useRobot((s) => s.isMine);
  if (!extJoy) return null;
  return (
    <Tappable disabled={!isMine} onPress={() => actions.gamepadExternal(ip, false).catch(() => {})}
      accessibilityLabel={t('외부 조종 끄기')}
      style={[styles.chip, {
        borderRadius: radius.sm,
        backgroundColor: 'rgba(163,113,247,0.12)', borderColor: 'rgba(163,113,247,0.5)',
      }, !isMine && { opacity: 0.4 }]}>
      <Icon name="route" size={13} color={c.accent2} />
      <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 10.5, fontWeight: '600' }}>SLAM</Text>
      <Icon name="x" size={11} color={c.accent2} />
    </Tappable>
  );
}


type Health = 'ok' | 'warn' | 'danger' | 'off';
function StatusLight({ color, pulse }: { color: string; pulse: 'none' | 'slow' | 'fast' }) {
  const [av] = useState(() => new Animated.Value(1));
  useEffect(() => {
    if (pulse === 'none') { av.setValue(1); return; }
    const d = pulse === 'fast' ? 600 : 1500;
    const loop = Animated.loop(Animated.sequence([
      Animated.timing(av, { toValue: pulse === 'fast' ? 0.25 : 0.5, duration: d, easing: Easing.inOut(Easing.quad), useNativeDriver: true }),
      Animated.timing(av, { toValue: 1, duration: d, easing: Easing.inOut(Easing.quad), useNativeDriver: true }),
    ]));
    loop.start();
    return () => loop.stop();
  }, [pulse, av]);
  return (
    <View style={styles.light}>
      <Animated.View style={[styles.halo, { backgroundColor: color, opacity: Animated.multiply(av, 0.3) }]} />
      <Animated.View style={[styles.dot, { backgroundColor: color, opacity: av }]} />
    </View>
  );
}

type CommonProps = { onEStop?: () => void };

export function TopBarHome({ onEStop, onSignal, onRightWidth, ddInBar = true }: CommonProps & { onSignal?: () => void; onRightWidth?: (w: number) => void; ddInBar?: boolean }) {
  const { c, fonts, radius } = useTheme();
  const agg = useRobot((s) => s.agg);
  const gait = useRobot((s) => s.gait);
  const gaitId = useRobot((s) => s.robot?.gait_id);
  const featureWheel = useFeatureWheel();
  const isMine = useRobot((s) => s.isMine);
  const conn = useRobot((s) => s.conn);
  const visionConn = useTelemetry((s) => s.visionConn);
  const isSim = useViewport((v) => v.key) === 'sim';
  const { width: winW } = useWindowDimensions();
  const insetL = useSafeAreaInsets().left;
  const leftMax = ddInBar ? winW / 2 - SOURCE_DD_W / 2 - 8 - EDGE - insetL : undefined;
  const health: Health = conn !== 'connected' ? 'off'
    : agg === 'red' ? 'danger'
    : agg === 'amber' || visionConn !== 'connected' || !isMine ? 'warn'
    : 'ok';
  const healthC = { ok: c.green, warn: c.amber, danger: c.red, off: c.red }[health];
  const tone: GaitTone = gaitTone(gaitId, gait);
  const toneC = { fault: c.red, idle: c.accent, active: c.green, unknown: c.dim }[tone];
  const toneTx = { fault: c.redTx, idle: c.accent2, active: c.greenTx, unknown: c.muted }[tone];
  const wheelName = featureWheel ? wheelGaitName(gaitId) : null;
  const gaitLabel = wheelName ? (getLang() === 'en' ? wheelName.en : wheelName.ko) : gait;
  return (
    <Bar>
      <Pod style={{ maxWidth: leftMax }}>
      <Logo />
      {!isSim && <Tappable
        onPress={onSignal} accessibilityLabel={t('시스템 상태')}
        style={[styles.chip, { backgroundColor: 'transparent', borderColor: c.glassLine, borderRadius: radius.sm, paddingHorizontal: 4, gap: 2 }]}
      >
        <StatusLight color={healthC} pulse={health === 'off' ? 'none' : health === 'danger' ? 'fast' : 'slow'} />
        <Icon name="caret" size={13} color={c.dim} />
      </Tappable>}
      {gait ? (
        <View
          style={[
            styles.chip,
            {
              borderRadius: radius.sm,
              flexShrink: 1,
              height: 32,
              paddingLeft: 0,
              overflow: 'hidden',
              backgroundColor: 'transparent',
              borderColor: c.glassLine,
            },
          ]}
        >
          <View style={[styles.rail, { backgroundColor: toneC }]} />
          <Text numberOfLines={1} style={{ color: toneTx, fontFamily: fonts.mono, fontSize: 13, fontWeight: '700', letterSpacing: 0.4 }}>
            {gaitLabel}
          </Text>
        </View>
      ) : null}
      {!isSim && <HlcChip />}
      {!isSim && <SignalStrength showSsid={false} showPct={false} />}
      </Pod>
      <View pointerEvents="none" style={{ flex: 1 }} />
      <TopRight onEStop={onEStop} hideRobot={isSim} onWidth={onRightWidth} />
    </Bar>
  );
}

export function SubBanners({ top = 60 }: { top?: number }) {
  const isSim = useViewport((v) => v.key) === 'sim';
  if (isSim) return null;
  return (
    <View pointerEvents="box-none" style={[styles.bannerStack, { top }]}>
      <HealthBanner />
      <LowSignalBanner />
      <CameraCalibBanner />
      <DockingBanner inline />
      <BoardFirmwareBanner />
      <AutoUpdateBanner />
      <LogToasts />
    </View>
  );
}

function Bar({ children }: { children: React.ReactNode }) {
  const { c } = useTheme();
  const insets = useSafeAreaInsets();
  const isSim = useViewport((v) => v.key) === 'sim';
  return (
    <>
      <View pointerEvents="box-none" style={[styles.bar, { paddingLeft: EDGE + insets.left, paddingRight: EDGE + insets.right }]}>
        {children}
      </View>
      <View pointerEvents="box-none" style={[styles.bannerStack, styles.bannerStackHome,
        { left: 112 + insets.left, right: 112 + insets.right }]}>
        {!isSim && <HealthBanner />}
        {!isSim && <LowSignalBanner />}
        {!isSim && <CameraCalibBanner />}
        {!isSim && <LogToasts />}
      </View>
      {!isSim && <DockingBanner openable />}
    </>
  );
}

const styles = StyleSheet.create({
  bannerStack: { position: 'absolute', top: 60, left: 0, right: 0, alignItems: 'center', gap: 8, zIndex: 90 },
  bannerStackHome: { top: 104 },
  bar: {
    height: 56, flexDirection: 'row', alignItems: 'center', gap: 8,
    paddingTop: EDGE, paddingBottom: 8,
    zIndex: 20,
  },
  pod: { flexDirection: 'row', alignItems: 'center', gap: 8, height: 44, paddingHorizontal: 8, borderWidth: 1, flexShrink: 1 },
  logo: { width: 32, height: 32, borderRadius: 9 },
  chip: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 32, paddingHorizontal: 8, borderWidth: 1 },
  rail: { width: 4, alignSelf: 'stretch' },
  light: { width: 22, height: 22, alignItems: 'center', justifyContent: 'center' },
  halo: { position: 'absolute', width: 20, height: 20, borderRadius: 10 },
  dot: { width: 9, height: 9, borderRadius: 4.5 },
  estop: { flexDirection: 'row', alignItems: 'center', gap: 7, height: 44, paddingHorizontal: 14, borderWidth: 1 },
  right: { flexDirection: 'row', alignItems: 'center', gap: 8, flexShrink: 0 },
  noShrink: { flexShrink: 0 },
  batt: { flexDirection: 'row', alignItems: 'center', height: 30 },
  estopTxt: { fontWeight: '700', fontSize: 13, letterSpacing: 0.3 },
});
