import { useEffect, useState } from 'react';
import { View, Text, Image, StyleSheet, Animated, Easing } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import { usePathname } from 'expo-router';
import { useRobotKind } from '@/modules/registry';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Battery, DeviceBattery } from '@/components/Battery';
import { SignalStrength, LowSignalBanner } from '@/components/SignalStrength';
import { useViewport } from '@/store/viewport';
import { IssueReportButton } from '@/components/IssueReport';
import { QuitButton } from '@/components/QuitButton';
import { useTinyW } from '@/lib/layout';
import { DockingBanner } from '@/components/DockingBanner';
import { BoardFirmwareBanner } from '@/components/BoardFirmwareBanner';
import { CameraCalibBanner } from '@/components/CameraCalib';
import { LogToasts } from '@/components/LogToasts';
import { LogBadge } from '@/components/LogBadge';
import { EStopModal } from '@/components/control/overlays';
import { Tappable } from '@/components/anim';
import { useRobot } from '@/store/robot';
import { actions } from '@/lib/rest';
import { useTelemetry } from '@/store/telemetry';
import { isRobotCharging } from '@/lib/robotState';
import { useFeatureWheel } from '@/store/capability';
import { connection } from '@/lib/connection';
import { wheelGaitName } from '@/lib/wheelGait';
import { gaitTone, type GaitTone } from '@/lib/gaitTone';
import { getLang } from '@/store/lang';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';


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

export function Logo() {
  const path = usePathname();
  return (
    <Tappable onPress={() => { if (path !== '/hub') goTop('/hub'); }} accessibilityLabel={t('허브')}>
      <Image source={require('@/assets/images/rb-logo.png')} style={styles.logo} />
    </Tappable>
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

export function TopBarHome({ onEStop, onSignal }: CommonProps & { onSignal?: () => void }) {
  const { c, fonts, radius } = useTheme();
  const agg = useRobot((s) => s.agg);
  const gait = useRobot((s) => s.gait);
  const gaitId = useRobot((s) => s.robot?.gait_id);
  const featureWheel = useFeatureWheel();
  const isMine = useRobot((s) => s.isMine);
  const conn = useRobot((s) => s.conn);
  const battPct = useRobot((s) => s.battPct);
  const robotChg = useTelemetry((s) => isRobotCharging(s.chargeAvg));
  const visionConn = useTelemetry((s) => s.visionConn);
  const isSim = useViewport((v) => v.key) === 'sim';
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
      <View style={[styles.pod, { backgroundColor: c.glass, borderColor: c.glassLine, borderRadius: radius.md }]}>
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
              height: 30,
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
      </View>
      <View pointerEvents="none" style={{ flex: 1 }} />
      <View style={[styles.pod, { backgroundColor: c.glass, borderColor: c.glassLine, borderRadius: radius.md }]}>
        <LogBadge />
        {!isSim && <Battery pct={conn === 'connected' ? battPct : null} label={t('로봇')} charging={robotChg} />}
        <DeviceBattery />
        <IssueReportButton />
      </View>
      <EStop onPress={onEStop} />
      <QuitButton />
    </Bar>
  );
}

export function SubBanners({ top = 60 }: { top?: number }) {
  const isSim = useViewport((v) => v.key) === 'sim';
  if (isSim) return null;
  return (
    <View pointerEvents="box-none" style={[styles.bannerStack, { top }]}>
      <LowSignalBanner />
      <CameraCalibBanner />
      <DockingBanner inline />
      <BoardFirmwareBanner />
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
      <View pointerEvents="box-none" style={[styles.bar, { paddingLeft: 4 + insets.left, paddingRight: 4 + insets.right }]}>
        {children}
      </View>
      <View pointerEvents="box-none" style={[styles.bannerStack, styles.bannerStackHome,
        { left: 112 + insets.left, right: 112 + insets.right }]}>
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
    paddingRight: 12,
    zIndex: 20,
  },
  pod: { flexDirection: 'row', alignItems: 'center', gap: 8, height: 44, paddingHorizontal: 8, borderWidth: 1, flexShrink: 1 },
  logo: { width: 32, height: 32, borderRadius: 9 },
  chip: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 32, paddingHorizontal: 8, borderWidth: 1 },
  rail: { width: 4, alignSelf: 'stretch' },
  light: { width: 22, height: 22, alignItems: 'center', justifyContent: 'center' },
  halo: { position: 'absolute', width: 20, height: 20, borderRadius: 10 },
  dot: { width: 9, height: 9, borderRadius: 4.5 },
  estop: { flexDirection: 'row', alignItems: 'center', gap: 7, height: 36, paddingHorizontal: 13, borderWidth: 1 },
  estopTxt: { fontWeight: '700', fontSize: 13, letterSpacing: 0.3 },
});
