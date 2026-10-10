import { useEffect, useState, useRef } from 'react';
import { useRouter, type Href } from 'expo-router';
import { isDemo } from '@/lib/demoFlag';
import { View, Text, StyleSheet, useWindowDimensions, Modal as RNModal } from 'react-native';
import { useTheme } from '@/theme';
import { t } from '@/lib/i18n';
import { Tappable } from '@/components/anim';
import { AttitudeDial } from '@/components/AttitudeDial';
import { useTelemetry } from '@/store/telemetry';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { Screen } from '@/components/Screen';
import { TopBarHome } from '@/components/TopBar';
import { Joystick, JOY_SIZE } from '@/components/Joystick';
import { Viewport, SourceDropdownList } from '@/components/Viewport';
import { DrivePopover, SignalPopover, EStopModal, AutoStartModal, MotionGridModal, useActiveMotion } from '@/components/control/overlays';
import { ZmpCalibModal } from '@/components/ZmpCalibModal';
import { AudioPopover, SoundPopover } from '@/components/control/AudioPopover';
import { SlamModal } from '@/components/control/SlamModal';
import { RightPanel, ViewRow } from '@/components/control/RightPanel';
import { isPtzSourceKey } from '@/components/control/PtzOverlay';
import { useMedia } from '@/lib/media';
import { ShotToast } from '@/components/control/ShotToast';
import { ControlPopover } from '@/components/control/ControlPopover';
import { VisionPopover } from '@/components/control/VisionPopover';
import { ViewSettingsPopover } from '@/components/control/ViewSettingsPopover';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { ArmPopover } from '@/components/control/ArmPopover';
import { usePtt } from '@/lib/ptt';
import { MotionButtons, usePadExtra } from '@/components/control/CompactControls';
import { setRailCardW, setDockAnchor } from '@/components/control/ToolRail';
import { ToolDock, PttButton, MediaDock, rightDockWidth, DOCK_BTN_H, DOCK_GAP, TINY_BTN } from '@/components/control/ToolDock';
import { LeftPanel, LEFT_X, LEFT_W } from '@/components/control/LeftPanel';
import { useCompactH, useCompactW, useTinyW } from '@/lib/layout';
import { SpeedHud } from '@/components/SpeedHud';
import { GaitHud } from '@/components/GaitHud';
import { connection } from '@/lib/connection';
import { useDockingBannerShown } from '@/components/DockingBanner';
import { useDockView } from '@/store/dockView';
import { resolveTouchAxes } from '@/lib/touchAxes';
import { useRobot } from '@/store/robot';
import { useNotMine } from '@/lib/spectating';
import { useRobots } from '@/store/robots';
import type { MotionName } from '@/types/robot';
import { useInputMode, setControlScreenActive } from '@/store/inputMode';
import { useHasArm, useHasLidar, useHasPtz } from '@/store/capability';
import { useSettings, useSettingsHydrated } from '@/store/settings';
import { robotVersionDef, useRobotKind } from '@/modules/registry';

const TOPBAR_H = 56;
import { useMotionFav } from '@/store/motionFav';
import { useViewport } from '@/store/viewport';
import { simEngine, useSimCourse } from '@/lib/simEngine';
import { SimLevelPicker } from '@/components/SimLevelPicker';
import { goTop } from '@/lib/nav';

type Overlay = null | 'ctrl' | 'vision' | 'drive' | 'audio' | 'sound' | 'signal' | 'motion' | 'estop' | 'autostart' | 'slam' | 'arm' | 'zmpcalib';






function AutoStart({ onPress, disabled }: { onPress: () => void; disabled?: boolean }) {
  const { c, radius } = useTheme();
  return (
    <Tappable onPress={disabled ? undefined : onPress} accessibilityLabel={t('자동 기동')}
      style={[styles.auto, { backgroundColor: c.glass, borderRadius: radius.md, borderColor: disabled ? c.glassLine : 'rgba(63,185,80,0.5)' }]}>
      <View style={[styles.playTri, { borderLeftColor: disabled ? c.muted : c.green }]} />
      <Text style={[styles.autoTxt, { color: disabled ? c.muted : c.greenTx }]}>{t('자동 기동')}</Text>
    </Tappable>
  );
}

function ModernGyro() {
  const { c, fonts, radius } = useTheme();
  const tiny = useTinyW();
  const rpy = useTelemetry((s) => s.robot?.imu.rpy);
  const [big, setBig] = useState(false);
  const roll = rpy ? (rpy[0] * 180) / Math.PI : 0;
  const pitch = rpy ? (rpy[1] * 180) / Math.PI : 0;
  const size = big ? 104 : tiny ? 30 : 44;
  return (
    <Tappable onPress={() => setBig((v) => !v)}
      style={{ alignItems: 'center', gap: 3,
        backgroundColor: c.glass, borderColor: c.glassLine, borderWidth: 1, borderRadius: radius.md, padding: tiny && !big ? 3 : 5 }}>
      <AttitudeDial size={size} roll={roll} pitch={pitch}
        colors={{ sky: c.legacyHorizonSky, ground: c.legacyHorizonGround, line: '#FFFFFF', cross: c.accent, border: c.line }} />
      {big && (
        <Text style={{ color: c.muted, fontSize: 10, fontFamily: fonts.mono, fontWeight: '700' }}>
          R {roll.toFixed(2)}°  P {pitch.toFixed(2)}°
        </Text>
      )}
    </Tappable>
  );
}

export default function Home() {
  const hydrated = useSettingsHydrated();
  if (!hydrated) return null;
  return <ControlHome />;
}

function ControlHome() {
  const { c } = useTheme();
  const [open, setOpen] = useState<Overlay>(null);
  const hasArm = useHasArm();
  const hasLidar = useHasLidar();
  const robotVersion = useSettings((s) => s.robotVersion);
  const slamAvail = hasLidar || !!robotVersionDef(robotVersion)?.slamWithoutLidar;
  const insets = useSafeAreaInsets();
  const [vpVoid, setVpVoid] = useState({ side: 0, bottom: 0 });
  const room = vpVoid.side - Math.max(insets.left, insets.right) - LEFT_X - 8;
  const narrow = useCompactW();
  const cardW = room >= LEFT_W ? LEFT_W : room >= 44 ? Math.floor(room) : narrow ? 60 : LEFT_W;
  useEffect(() => { setRailCardW(cardW); }, [cardW]);
  const cardsOverVideo = room < 44;
  const vpKey = useViewport((s) => s.key);
  const isSim = vpKey === 'sim';
  const setVpKey = useViewport((s) => s.setKey);
  const hasPtz = useHasPtz();
  const prevVpKeyRef = useRef<string>('front');
  const vpOpen = useViewport((s) => s.ddOpen);
  const setVpOpen = (v: boolean | ((p: boolean) => boolean)) =>
    useViewport.getState().setDdOpen(typeof v === 'function' ? v(useViewport.getState().ddOpen) : v);
  useEffect(() => () => { useViewport.getState().setDdOpen(false); useViewport.getState().setBarPopOpen(false); }, []);
  useEffect(() => { useViewport.getState().setBarPopOpen(open === 'signal'); }, [open]);
  const [favTarget, setFavTarget] = useState<number | null>(null);
  const controlOn = useRobot((s) => !!s.robot?.control_started);
  const isMine = useRobot((s) => s.isMine);
  const notMine = useNotMine();
  useEffect(() => { if (notMine) setOpen((o) => (o === 'autostart' ? null : o)); }, [notMine]);
  const conn = useRobot((s) => s.conn);
  const activeMotion = useActiveMotion();
  const padExtra = usePadExtra();
  const inputMode = useInputMode();
  const panelMode = inputMode !== 'touch';
  const compactH = useCompactH();
  const tiny = useTinyW();
  const { width: winW, height: winH } = useWindowDimensions();
  const stageH = Math.max(0, winH - TOPBAR_H);
  const gutter = Math.max(0, Math.round((winW - Math.min(winW, (stageH * 16) / 9)) / 2));
  const joySize = Math.max(96, Math.min(128, gutter > 40 ? gutter - 16 : 128));
  const joyInset = gutter > 40 ? Math.max(6, Math.round((gutter - joySize) / 2)) : 14;
  const fullBleed = true;
  const router = useRouter();
  const kindRoute = useRobotKind()?.route;
  useEffect(() => { if (kindRoute) goTop(kindRoute as Href); }, [kindRoute, router]);
  const pttListen = usePtt((s) => s.listen);
  const extJoyOn = useTelemetry((s) => !!s.robot?.extJoy);

  const [sceneOpenG, setSceneOpenG] = useState(false);
  const recording = useMedia((s) => s.recordingPath != null);
  const [gaugeW, setGaugeW] = useState(0);
  const TOOLS = [
    { key: 'drive', icon: 'sliders' as const, label: t('주행'), open: open === 'drive', onPress: () => setOpen(open === 'drive' ? null : 'drive') },
    ...(hasArm && !isSim ? [{ key: 'arm', icon: 'hand' as const, label: t('팔'), open: open === 'arm', onPress: () => setOpen(open === 'arm' ? null : 'arm') }] : []),
    { key: 'ctrl', icon: 'gamepad' as const, label: t('조종'), open: open === 'ctrl', dot: extJoyOn,
      onPress: () => setOpen(open === 'ctrl' ? null : 'ctrl') },
  ];
  const [levelsOpen, setLevelsOpen] = useState(false);
  const LEFT_TOOLS = isSim ? [
    ...TOOLS.filter((x) => x.key === 'drive'),
    { key: 'level', icon: 'route' as const, label: t('레벨'), open: levelsOpen, onPress: () => setLevelsOpen(true) },
    { key: 'reset', icon: 'recover' as const, label: t('리셋'), open: false, onPress: () => simEngine.reset() },
  ] : TOOLS;
  const dockPos = panelMode ? { left: 4 + insets.left, bottom: 30 + insets.bottom }
    : compactH ? { left: joyInset + joySize + 10, bottom: 12 }
    : { left: 24 + JOY_SIZE + 12, bottom: 22 };
  const pttPos = panelMode ? { right: 4 + insets.right, bottom: 30 + insets.bottom }
    : compactH ? { right: joyInset + joySize + 10, bottom: 12 }
    : { right: 24 + JOY_SIZE + 12, bottom: 22 };
  const mediaVertical = !tiny && winW - pttPos.right - rightDockWidth(compactH) < winW / 2 + gaugeW / 2 + 8;
  useEffect(() => { setDockAnchor({ left: dockPos.left, bottom: dockPos.bottom + DOCK_BTN_H + 8 }); }, [dockPos.left, dockPos.bottom]);


  const speedHud = useSettings((x) => x.speedHud);
  const gyroOn = useSettings((x) => x.gyroWidgetEnabled);
  const simCourse = useSimCourse();
  const viewRows = (
    <>
      <ViewRow icon="search" label={t('비전')} on={open === 'vision'}
        onPress={() => setOpen(open === 'vision' ? null : 'vision')} />
      {vpKey === 'pose3d' && <ViewRow icon="box" label={t('3D 씬')} on={sceneOpenG} onPress={() => setSceneOpenG(!sceneOpenG)} />}
      <ViewRow icon="mic" label={t('오디오')} on={open === 'audio'} dot={pttListen || recording}
        onPress={() => setOpen(open === 'audio' ? null : 'audio')} />
      {slamAvail && (
        <ViewRow icon="route" label="SLAM" on={open === 'slam'} onPress={() => setOpen(open === 'slam' ? null : 'slam')} />
      )}
      {hasPtz && (
        <ViewRow icon="ptz" label={t('PTZ')} on={isPtzSourceKey(vpKey)}
          onPress={() => {
            if (isPtzSourceKey(vpKey)) { setVpKey(prevVpKeyRef.current); return; }
            prevVpKeyRef.current = vpKey;
            setVpKey('cctv');
          }} />
      )}
    </>
  );

  const { show: dockingShown } = useDockingBannerShown();
  const dockViewOpen = useDockView((s) => s.open);
  const gpOneStick = useSettings((s) => s.gpOneStick);
  const padRef = useRef({ left: { x: 0, y: 0 }, right: { x: 0, y: 0 } });
  useEffect(() => () => {
    padRef.current = { left: { x: 0, y: 0 }, right: { x: 0, y: 0 } };
    connection.setAxes('L', 0, 0);
    connection.setAxes('R', 0, 0);
  }, [compactH, panelMode]);
  useEffect(() => () => { connection.setAxes('L', 0, 0); connection.setAxes('R', 0, 0); }, []);
  useEffect(() => { setControlScreenActive(true); return () => setControlScreenActive(false); }, []);
  useEffect(() => {
    if (isSim || isDemo() || conn === 'connected') return;
    const id = setTimeout(() => { if (useRobot.getState().conn !== 'connected') goTop('/hub'); }, 8000);
    return () => clearTimeout(id);
  }, [conn, isSim, router]);
  const serial = useRobots((st) => st.currentSerial);
  const serial0 = useRef(serial);
  useEffect(() => {
    if (isSim || isDemo() || serial === serial0.current) return;
    goTop('/hub');
  }, [serial, isSim, router]);
  const close = () => setOpen(null);
  useEffect(() => { if (vpKey !== 'pose3d') setSceneOpenG(false); }, [vpKey]);
  return (
    <Screen bleed>
      <View style={styles.stage}>
        <View
          style={fullBleed
            ? StyleSheet.absoluteFill
            : styles.centerCol}
          pointerEvents="box-none">
          <Viewport
            selectedKey={vpKey}
            onSelect={(k) => { setVpKey(k); setVpOpen(false); }}
            onToggleOpen={() => setVpOpen((v) => !v)}
            maximized={fullBleed}
            onVoid={setVpVoid}
            topInset={0}
            compact={compactH}
            sideReserve={undefined}
            bottomReserve={controlOn ? (speedHud ? 150 : 110) : 250}
            chrome
          />
        </View>
        <View style={[styles.centerDock, { bottom: 12 + insets.bottom }]} pointerEvents="box-none">
          {!controlOn && !isSim && (
            <AutoStart disabled={conn !== 'connected'} onPress={() => setOpen('autostart')} />
          )}
          <View style={styles.gauges} pointerEvents="box-none" onLayout={(e) => setGaugeW(e.nativeEvent.layout.width)}>
            {gyroOn && (conn === 'connected' || isSim) && <ModernGyro />}
            {speedHud && ((!dockingShown && !dockViewOpen) || isSim) && <SpeedHud dense={compactH} />}
          </View>
        </View>
        <GaitHud bottom={8 + insets.bottom} left={6} />
        {!isSim && (
          <RightPanel inset={insets.right} title="관측" overVideo={cardsOverVideo} width={cardW} topInset={56} bottomReserve={panelMode ? 30 : joySize + 24}>
            {viewRows}
          </RightPanel>
        )}
        {panelMode ? (
          compactH ? (
            <>
              {!isSim && <LeftPanel inset={insets.left} overVideo={cardsOverVideo} width={cardW} topInset={56}
                bottomReserve={30 + DOCK_BTN_H + 8}>
                <MotionButtons active={activeMotion} extra={padExtra}
                  onSelect={(m: MotionName) => connection.sendMotion(m)}
                  onMore={() => { setFavTarget(null); setOpen('motion'); }} />
              </LeftPanel>}
            </>
          ) : (
            <>
              {!isSim && <LeftPanel inset={insets.left} overVideo={cardsOverVideo} width={cardW} topInset={56} bottomReserve={30 + DOCK_BTN_H + 8}>
                <MotionButtons active={activeMotion} extra={padExtra}
                  onSelect={(m: MotionName) => connection.sendMotion(m)}
                  onMore={() => { setFavTarget(null); setOpen('motion'); }} />
              </LeftPanel>}
            </>
          )
        ) : compactH ? (
          <>
            <View
              style={[styles.joyAbs, styles.joyCompact, { left: joyInset }]}>
              <Joystick size={joySize}
                onMove={(nx, ny) => {
                  padRef.current.left = { x: nx, y: ny };
                  const { L, R } = resolveTouchAxes(gpOneStick, padRef.current.left, padRef.current.right);
                  connection.setAxes('L', L.x, L.y);
                  connection.setAxes('R', R.x, R.y);
                }}
              />
            </View>
            <View
              style={[styles.joyAbs, styles.joyCompact, { right: joyInset }]}>
              <Joystick size={joySize}
                onMove={(nx, ny) => {
                  padRef.current.right = { x: nx, y: ny };
                  const { L, R } = resolveTouchAxes(gpOneStick, padRef.current.left, padRef.current.right);
                  connection.setAxes('L', L.x, L.y);
                  connection.setAxes('R', R.x, R.y);
                }}
              />
            </View>
            {!isSim && <LeftPanel inset={insets.left} overVideo={cardsOverVideo} width={cardW} topInset={56}
              bottomReserve={joySize + 24}>
              <MotionButtons active={activeMotion} extra={padExtra}
   onSelect={(m: MotionName) => connection.sendMotion(m)}
   onMore={() => { setFavTarget(null); setOpen('motion'); }} />
            </LeftPanel>}
          </>
        ) : (
          <>
            <View
              style={[styles.joyAbs, { left: 24 }]}>
              <Joystick
                onMove={(nx, ny) => {
                  padRef.current.left = { x: nx, y: ny };
                  const { L, R } = resolveTouchAxes(gpOneStick, padRef.current.left, padRef.current.right);
                  connection.setAxes('L', L.x, L.y);
                  connection.setAxes('R', R.x, R.y);
                }}
              />
            </View>
            <View
              style={[styles.joyAbs, { right: 24 }]}>
              <Joystick
                onMove={(nx, ny) => {
                  padRef.current.right = { x: nx, y: ny };
                  const { L, R } = resolveTouchAxes(gpOneStick, padRef.current.left, padRef.current.right);
                  connection.setAxes('L', L.x, L.y);
                  connection.setAxes('R', R.x, R.y);
                }}
              />
            </View>
            {!isSim && <LeftPanel inset={insets.left} overVideo={cardsOverVideo} width={cardW} topInset={56}
              bottomReserve={182}>
              <MotionButtons active={activeMotion} extra={padExtra}
   onSelect={(m: MotionName) => connection.sendMotion(m)}
   onMore={() => { setFavTarget(null); setOpen('motion'); }} />
            </LeftPanel>}
          </>
        )}
        <ToolDock tools={LEFT_TOOLS} left={dockPos.left} bottom={dockPos.bottom} compact={compactH} tiny={tiny} />
        {!isSim && <PttButton right={pttPos.right} bottom={pttPos.bottom} compact={compactH} tiny={tiny} />}
        {!isSim && <MediaDock right={pttPos.right} bottom={pttPos.bottom} compact={compactH} tiny={tiny} vertical={mediaVertical}
          soundOpen={open === 'sound'} onSound={() => setOpen(open === 'sound' ? null : 'sound')} />}
      </View>
      <View style={styles.barOverlay} pointerEvents="box-none">
        <TopBarHome onSignal={() => setOpen('signal')} onEStop={() => setOpen('estop')} />
      </View>
      <SourceDropdownList open={vpOpen} selectedKey={vpKey}
        onSelect={(k) => { setVpKey(k); setVpOpen(false); }} onToggleOpen={() => setVpOpen((v) => !v)}
      />

      {open === 'ctrl' && <ControlPopover onClose={close} />}
      {open === 'vision' && <VisionPopover onClose={close} />}
      {open === 'drive' && <DrivePopover onClose={close} />}
      {open === 'audio' && <AudioPopover onClose={close} />}
      {open === 'sound' && <SoundPopover onClose={close} right={pttPos.right}
        bottom={pttPos.bottom + ((tiny ? TINY_BTN : compactH ? 40 : DOCK_BTN_H) + DOCK_GAP) * (mediaVertical ? 3 : 1) + 2} />}
      {levelsOpen && (
        <SimLevelPicker current={simCourse?.map ?? ''} onClose={() => setLevelsOpen(false)}
          onPick={(id) => { setLevelsOpen(false); simEngine.setMap(id); }} />
      )}
      <ShotToast />
      {sceneOpenG && (
        <RNModal supportedOrientations={MODAL_ORIENTATIONS} transparent visible animationType="fade" onRequestClose={() => setSceneOpenG(false)}>
          <ViewSettingsPopover onClose={() => setSceneOpenG(false)} />
        </RNModal>
      )}
      {open === 'slam' && <SlamModal onClose={close} />}
      {open === 'arm' && (
        <ArmPopover onClose={close}
          onOpenDoor={() => { close(); router.push('/arm-door'); }}
          onDetail={() => { close(); router.push('/arm'); }} />
      )}
      {open === 'signal' && <SignalPopover onClose={close} onAutoStart={isMine ? () => setOpen('autostart') : undefined} />}
      {open === 'estop' && <EStopModal onClose={close} />}
      {open === 'autostart' && <AutoStartModal onClose={close} />}
      {open === 'zmpcalib' && <ZmpCalibModal onClose={close} />}
      {open === 'motion' && (
        <MotionGridModal
          danger={favTarget == null}
          onClose={close}
          onZmpCalib={() => setOpen('zmpcalib')}
          onPick={(m) => {
            connection.sendMotion(m);
            if (favTarget != null) useMotionFav.getState().setSlot(favTarget, m);
          }}
        />
      )}
    </Screen>
  );
}

const styles = StyleSheet.create({
  stage: { flex: 1, position: 'relative' },
  barOverlay: { position: 'absolute', top: 0, left: 0, right: 0, zIndex: 30 },
  centerCol: { position: 'absolute', left: 0, right: 0, top: 18, alignItems: 'center' },
  joyAbs: { position: 'absolute', bottom: 22, zIndex: 2 },
  joyCompact: { bottom: 12 },
  centerDock: { position: 'absolute', left: 0, right: 0, bottom: 12, alignItems: 'center', gap: 8, zIndex: 5 },
  gauges: { flexDirection: 'row', alignItems: 'flex-end', gap: 8 },
  auto: { flexDirection: 'row', alignItems: 'center', gap: 9, height: 34, paddingHorizontal: 14, borderWidth: 1 },
  playTri: { width: 0, height: 0, borderLeftWidth: 9, borderTopWidth: 6, borderBottomWidth: 6, borderTopColor: 'transparent', borderBottomColor: 'transparent' },
  autoTxt: { fontWeight: '600', fontSize: 12 },
});
