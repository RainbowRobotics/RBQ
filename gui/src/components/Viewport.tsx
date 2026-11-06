import { EDGE, SOURCE_DD_W } from '@/lib/layout';
import { useEffect, useRef, useState, useMemo } from 'react';
import { View, Text, StyleSheet, Pressable, PanResponder, useWindowDimensions, type StyleProp, type ViewStyle } from 'react-native';
import { useTheme } from '@/theme';
import Animated from 'react-native-reanimated';
import { Tappable, unfoldDown, foldUp } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { RobotModel3D, OBSMAP_3D_W } from '@/components/RobotModel3D';
import { useSimScene } from '@/components/SimScene';
import { levelById, fmtTime } from '@/lib/simCourse';
import { SimHost } from '@/components/SimHost';
import { simEngine, useSimCourse, type SimCam } from '@/lib/simEngine';
import { revealOnKeyframe, REVEAL_CAP_MS } from '@/lib/videoKeyframe';
import { CameraView } from '@/components/CameraView';
import { WallMapView } from '@/components/control/WallMapView';
import { SlamMapView } from '@/components/control/SlamMapView';
import { useAvailableSources } from '@/lib/visionSources';
import { Platform } from 'react-native';
import { useViewport } from '@/store/viewport';
import { useView3d } from '@/store/view3d';
import { MODERN_VIEW3D_THEMES } from '@/lib/view3dThemes';
import { useRobot } from '@/store/robot';
import { useSpectating } from '@/lib/spectating';
import { streamerCommand } from '@/lib/commandBus';
import { useSettings } from '@/store/settings';
import { actions, restErrorText } from '@/lib/rest';
import { cctv } from '@/lib/vision';
import { ViewSettingsPopover } from '@/components/control/ViewSettingsPopover';
import { P2gTapMarker } from '@/components/P2gTapMarker';
import { PtzOverlay } from '@/components/control/PtzOverlay';
import { viewportOverlays } from '@/modules/registry';
import { DockScanOverlay, type DockViewKind } from '@/components/DockScanOverlay';
import { DockView } from '@/components/DockView';
import { useDockView } from '@/store/dockView';
import { DOCK_STREAM_ID, NO_STREAM_ID, STAIRS_STREAM_ID } from '@/lib/visionSources';
import { P2gOverlay } from '@/components/P2gOverlay';
import { DriveGuideOverlay } from '@/components/DriveGuideOverlay';
import { useVisionToggles } from '@/store/visionToggles';
import { t } from '@/lib/i18n';


const SIDE_RESERVE = 470;
const DD_W = SOURCE_DD_W;
const STAIRS_BLACK_W = 540 / 1280;

export function Viewport({
  selectedKey,
  onSelect,
  onToggleOpen,
  maximized = false,
  onVoid,
  topInset = 0,
  sideReserve = SIDE_RESERVE,
  bottomReserve = 240,
  chrome = true,
  compact = false,
  onBackgroundPress,
}: {
  selectedKey: string;
  onSelect: (k: string) => void;
  onToggleOpen: () => void;
  maximized?: boolean;
  onVoid?: (v: { side: number; bottom: number }) => void;
  topInset?: number;
  sideReserve?: number;
  bottomReserve?: number;
  chrome?: boolean;
  compact?: boolean;
  onBackgroundPress?: () => void;
}) {
  const { c, radius } = useTheme();
  const { width, height } = useWindowDimensions();
  const sources = useAvailableSources();
  const cur = sources.find((s) => s.key === selectedKey) ?? sources[0];
  const spectating = useSpectating();
  const simCamNow = useSimCourse()?.cam ?? 'third';

  const p2g = useViewport((s) => s.p2g);
  const camSize = useRef({ w: 0, h: 0 });
  const opLive = useOperatorLive(spectating && cur?.kind === 'camera');
  const opIn3d = spectating && cur?.kind === 'camera' && opLive === NO_STREAM_ID;
  const opStairs = spectating && cur?.kind === 'camera' && opLive === STAIRS_STREAM_ID;
  const isCamSource = cur?.kind === 'camera' && !opIn3d;
  const dockOpen = useDockView((s) => s.open);
  const dockPaused = useDockView((s) => s.paused);
  const [dockShown, setDockShown] = useState(false);
  useEffect(() => {
    if (!dockOpen) { setDockShown(false); return; }
    return revealOnKeyframe(Date.now(), () => setDockShown(true));
  }, [dockOpen]);
  const isCam = isCamSource || dockShown;
  const camActive = isCamSource || dockOpen;
  const dockClosedAt = useDockView((s) => s.closedAt);
  const [exitDoneFor, setExitDoneFor] = useState<number | null>(null);
  const exitHold = !dockOpen && isCamSource && dockClosedAt != null && exitDoneFor !== dockClosedAt
    && Date.now() - dockClosedAt < REVEAL_CAP_MS;
  useEffect(() => {
    if (!exitHold || dockClosedAt == null) return;
    return revealOnKeyframe(dockClosedAt, () => setExitDoneFor(dockClosedAt));
  }, [exitHold, dockClosedAt]);
  const srcKey = cur?.key;
  const prevSrcKey = useRef(srcKey);
  useEffect(() => {
    if (prevSrcKey.current === srcKey) return;
    prevSrcKey.current = srcKey;
    useDockView.getState().hide();
  }, [srcKey]);
  const dockViewKind: DockViewKind =
    cur?.key === 'front' ? 'front' : cur?.key === 'rear' ? 'rear' : cur?.key === 'stacked' ? 'stacked' : 'other';
  const p2gAvail = isCamSource && !dockOpen && cur?.streamId === 1;
  const guideOn = useVisionToggles((s) => s.guide);
  const guideCam = guideOn && isCamSource && !dockOpen && (cur?.streamId === 1 || cur?.streamId === 2) ? cur.streamId : null;
  const [taps, setTaps] = useState<{ id: number; x: number; y: number; h: number }[]>([]);
  const [aim, setAim] = useState<{ x: number; y: number } | null>(null);
  const tapSeq = useRef(0);
  const aimAt = useRef<{ x: number; y: number } | null>(null);

  const p2gPan = useMemo(() => PanResponder.create({
    onStartShouldSetPanResponder: () => true,
    onMoveShouldSetPanResponder: () => true,
    onPanResponderGrant: (e) => {
      const { locationX, locationY } = e.nativeEvent;
      aimAt.current = { x: locationX, y: locationY };
      setAim({ x: locationX, y: locationY });
    },
    onPanResponderMove: (e) => {
      const { locationX, locationY } = e.nativeEvent;
      aimAt.current = { x: locationX, y: locationY };
      setAim({ x: locationX, y: locationY });
    },
    onPanResponderRelease: () => {
      const p = aimAt.current;
      aimAt.current = null;
      setAim(null);
      const { w, h } = camSize.current;
      if (p == null || !(w > 0 && h > 0)) return;
      if (!Number.isFinite(p.x) || !Number.isFinite(p.y)) {
        useRobot.getState().pushLog({ ts: '', process: 'App', level: 'ERROR',
          msg: 'P2G 지점 전송 취소 — 조준 좌표를 얻지 못했습니다' });
        return;
      }
      if (p.x < 0 || p.y < 0 || p.x > w || p.y > h) return;
      actions.visionTouchClick(useRobot.getState().ip, p.x / w, p.y / h).catch((err: unknown) => {
        useRobot.getState().pushLog({ ts: '', process: 'App', level: 'ERROR',
          msg: `P2G 지점 전송 실패 — ${restErrorText(err)}` });
      });
      const id = ++tapSeq.current;
      setTaps((ts) => [...ts, { id, x: p.x, y: p.y, h }]);
    },
    onPanResponderTerminate: () => { aimAt.current = null; setAim(null); },
  }), []);
  const isCctv = cur?.key === 'cctv';

  const v3d = useView3d();
  const mt = MODERN_VIEW3D_THEMES[v3d.themeIndex] ?? MODERN_VIEW3D_THEMES[0];


  const maxH = Math.max(200, height - bottomReserve);
  const vw = Math.max(300, Math.min(width - sideReserve, (maxH * 16) / 9));
  const vh = (vw * 9) / 16;

  const [fullBox, setFullBox] = useState({ w: 0, h: 0 });
  const simFill = cur?.kind === 'sim' && !dockShown && simCamNow !== 'stairs';
  const fw = !(fullBox.w && fullBox.h) ? 0 : simFill ? fullBox.w : Math.min(fullBox.w, (fullBox.h * 16) / 9);
  const fh = simFill ? fullBox.h : fw / (16 / 9);
  const sideVoid = fw ? Math.max(0, (fullBox.w - fw) / 2) : 0;
  const bottomVoid = fw ? Math.max(0, fullBox.h - fh) : 0;
  useEffect(() => {
    if (!maximized || !fw) return;
    onVoid?.({ side: Math.round(sideVoid), bottom: Math.round(bottomVoid) });
  }, [maximized, fw, sideVoid, bottomVoid, onVoid]);

  const is3d = (cur?.kind === 'pose3d' || opIn3d) && !dockShown;
  const isSim = cur?.kind === 'sim' && !dockShown;
  const simScene = useSimScene(isSim);
  const isStairs = (spectating ? opStairs : cur?.key === 'stairs') && !dockShown;
  const isObsMap = cur?.kind === 'obsmap';
  const isSlam = cur?.kind === 'slam' && !dockShown;
  const obsAvoidOnForElev = useSettings((s) => s.obsAvoidEnabled);
  const mapGenOnForElev = useSettings((s) => s.mapGenOn);
  const mapGenShowCamFront = useSettings((s) => s.mapGenShowCamFront);
  const mapGenShowCamRear = useSettings((s) => s.mapGenShowCamRear);
  const mapGenShow3d = useSettings((s) => s.mapGenShow3d);
  const mapGenShow2d = useSettings((s) => s.mapGenShow2d);
  const elevationOn = obsAvoidOnForElev || (mapGenOnForElev && (mapGenShow3d || mapGenShow2d));
  const mapGenPanes = (mapGenOnForElev
    ? [(mapGenShowCamFront || mapGenShowCamRear) && 'cam', mapGenShow3d && '3d', mapGenShow2d && '2d']
    : []
  ).filter((x): x is 'cam' | '3d' | '2d' => !!x);
  const mapGenSplit = mapGenPanes.length >= 1;
  const otherPaneCount = mapGenPanes.filter((k) => k !== '2d').length;
  const paneWidth = (k: 'cam' | '3d' | '2d'): number => {
    if (!mapGenSplit) return 0;
    if (mapGenShow2d && otherPaneCount > 0) return k === '2d' ? 30 : 70 / otherPaneCount;
    return 100 / mapGenPanes.length;
  };
  const paneLeft = (k: 'cam' | '3d' | '2d'): number => {
    let left = 0;
    for (const p of mapGenPanes) { if (p === k) break; left += paneWidth(p); }
    return left;
  };
  const paneStyle = (k: 'cam' | '3d' | '2d'): ViewStyle => {
    const i = mapGenPanes.indexOf(k);
    return { position: 'absolute', top: 0, bottom: 0,
      left: `${paneLeft(k)}%` as `${number}%`, width: `${paneWidth(k)}%` as `${number}%`,
      borderLeftWidth: i > 0 ? 1 : 0, borderLeftColor: c.line };
  };
  const camPaneOn = mapGenSplit && (mapGenShowCamFront || mapGenShowCamRear);
  const pane3dOn = mapGenSplit && mapGenShow3d;
  const pane2dOn = mapGenSplit && mapGenShow2d;
  const camShown = mapGenSplit ? camPaneOn : isCam;
  const camActiveSplit = camActive || camPaneOn;
  const show2dContainer = mapGenSplit ? pane2dOn : isObsMap;
  const miniActive = pane2dOn;
  const miniSplitX = pane3dOn ? paneWidth('3d') / (paneWidth('3d') + paneWidth('2d')) : 0;
  const miniStyle = (): ViewStyle => {
    const leftKey = pane3dOn ? '3d' : '2d';
    const width = pane3dOn ? paneWidth('3d') + paneWidth('2d') : paneWidth('2d');
    const leftIdx = mapGenPanes.indexOf(leftKey);
    return { position: 'absolute', top: 0, bottom: 0,
      left: `${paneLeft(leftKey)}%` as `${number}%`, width: `${width}%` as `${number}%`,
      borderLeftWidth: leftIdx > 0 ? 1 : 0, borderLeftColor: c.line };
  };
  const show3dContainer = mapGenSplit ? (pane3dOn || miniActive) : (is3d || isSim || isStairs || isObsMap);
  const [camMounted, setCamMounted] = useState(false);
  const lastCamId = useRef<number>(1);
  const preDockCamId = useRef<number | null>(null);
  if (dockOpen) {
    if (preDockCamId.current == null) preDockCamId.current = lastCamId.current;
    lastCamId.current = dockPaused ? NO_STREAM_ID : DOCK_STREAM_ID;
  } else {
    if (preDockCamId.current != null) { lastCamId.current = preDockCamId.current; preDockCamId.current = null; }
    if (isCamSource && cur?.streamId != null) lastCamId.current = cur.streamId;
  }
  const camPaneCropRegion: 'frontRear' | 'front' | 'rear' | undefined = !camPaneOn ? undefined
    : mapGenShowCamFront && mapGenShowCamRear ? 'frontRear'
    : mapGenShowCamFront ? 'front'
    : 'rear';
  const camPaneStreamId = camPaneOn ? STAIRS_STREAM_ID : lastCamId.current;
  useEffect(() => { if (isCamSource || dockOpen || camPaneOn) setCamMounted(true); }, [isCamSource, dockOpen, camPaneOn]);
  const zHide = Platform.OS === 'ios';
  const body = (
    <View style={StyleSheet.absoluteFill}>
      {!zHide && camMounted && (
        <View style={camPaneOn ? [paneStyle('cam')] : [StyleSheet.absoluteFill, { opacity: camShown ? 1 : 0 }]}
          pointerEvents={camShown ? 'auto' : 'none'}>
          <CameraView streamId={camPaneStreamId} active={camActiveSplit} cropRegion={camPaneCropRegion} />
        </View>
      )}
      {isSim && <SimHost />}
      <View
        style={pane2dOn ? [paneStyle('2d'), { overflow: 'hidden' }, zHide && { zIndex: 2 }]
          : !mapGenSplit && isObsMap
            ? [styles.obsMapPanel, { overflow: 'hidden' }, zHide && { zIndex: 2 }, { width: `${(1 - OBSMAP_3D_W) * 100}%`, borderLeftColor: c.line, borderLeftWidth: 1 }]
            : [StyleSheet.absoluteFill, { opacity: 0 }]}
        pointerEvents={show2dContainer ? 'auto' : 'none'}>
        <WallMapView active={show2dContainer} />
      </View>
      <View
        style={miniActive ? [miniStyle(), zHide && { zIndex: 2 }]
          : pane3dOn ? [paneStyle('3d'), zHide && { zIndex: 2 }]
          : mapGenSplit ? [StyleSheet.absoluteFill, zHide ? { zIndex: 0 } : { opacity: 0 }]
          : is3d || isSim || isObsMap ? [StyleSheet.absoluteFill, zHide && { zIndex: 2 }]
          : isStairs ? [styles.stairsPose, zHide && { zIndex: 2 }, { width: `${STAIRS_BLACK_W * 100}%`, backgroundColor: c.panel2, borderLeftColor: c.line, borderLeftWidth: 1 }]
          : [StyleSheet.absoluteFill, zHide ? { zIndex: 0 } : { opacity: 0 }]}
        pointerEvents={show3dContainer ? 'box-none' : 'none'}>
        <RobotModel3D active={show3dContainer}
          controls={(mapGenSplit ? pane3dOn : (is3d || isSim || isObsMap)) ? chrome && !compact : false}
          listenPresets={mapGenSplit ? pane3dOn : is3d} onTap={onBackgroundPress}
          terrain={isSim ? simScene.terrain : undefined}
          gridVisible={v3d.gridVisible && !isSim} gridSize={v3d.gridSize}
          gridColor={mt.grid} gridCenterColor={mt.gridCenter} backgroundColor={mt.bg}
          viewMode={v3d.viewMode} lidarMode={v3d.lidarMode}
          avoidCamLock={mapGenOnForElev}
          elevationOn={elevationOn} topDownMini={miniActive || (isObsMap && !mapGenSplit)}
          miniSplit={miniActive ? miniSplitX : undefined}
          fpv={isSim && simScene.state?.cam !== 'third' ? simScene.state?.cam : undefined}
          driveGuide={guideOn && (is3d || isSim)}
          ledBottom />
        {isSim && (
          <View style={styles.simBar} pointerEvents="none">
            <View style={[styles.simPill, { backgroundColor: c.glass, borderColor: c.glassLine }]}>
            <Text style={[styles.simTxt, { color: simScene.state?.error ? c.redbright : simScene.state?.note ? c.amber : c.muted }]}>
              {simScene.state?.error
                ? `${t('시뮬레이터를 불러오지 못했습니다')} — ${simScene.state.error}`
                : !simScene.state?.ready
                  ? t('시뮬레이터 준비 중…')
                  : simScene.state.note
                    ? simScene.state.note
                    : courseLine(simScene.state)}
            </Text>
            </View>
          </View>
        )}
      </View>
      {isSlam && (
        <View style={[StyleSheet.absoluteFill, zHide && { zIndex: 2 }]}>
          <SlamMapView backgroundColor={mt.bg} />
        </View>
      )}
      {zHide && camMounted && (
        <View style={camPaneOn ? [paneStyle('cam'), { zIndex: 1 }] : [StyleSheet.absoluteFill, { zIndex: 1, opacity: camShown ? 1 : 0 }]}
          pointerEvents={camShown ? 'auto' : 'none'}>
          <CameraView streamId={camPaneStreamId} active={camActiveSplit} cropRegion={camPaneCropRegion} />
        </View>
      )}
      {exitHold && <View style={[StyleSheet.absoluteFill, { zIndex: 3, backgroundColor: '#000' }]} pointerEvents="none" />}
    </View>
  );

  const clipBox = (
    <View
      style={[StyleSheet.absoluteFill, { backgroundColor: c.panel2, borderColor: c.line, borderWidth: maximized ? 0 : 1, borderRadius: maximized ? 0 : radius.lg, overflow: 'hidden' }]}
      onLayout={(e) => { camSize.current = { w: e.nativeEvent.layout.width, h: e.nativeEvent.layout.height }; }}
    >
      {onBackgroundPress ? (
        <Pressable style={StyleSheet.absoluteFill} onPress={onBackgroundPress}>
          {body}
        </Pressable>
      ) : (
        body
      )}
      {p2gAvail && p2g && (
        <View style={StyleSheet.absoluteFill} {...p2gPan.panHandlers} />
      )}
      {p2gAvail && p2g && (
        <View pointerEvents="none"
          style={[StyleSheet.absoluteFill, { borderWidth: 2, borderColor: 'rgba(77,156,245,0.85)' }]} />
      )}
      {aim && <P2gTapMarker x={aim.x} y={aim.y} viewH={camSize.current.h} aiming onDone={() => {}} />}
      {taps.map((tp) => (
        <P2gTapMarker key={tp.id} x={tp.x} y={tp.y} viewH={tp.h}
          onDone={() => setTaps((ts) => ts.filter((v) => v.id !== tp.id))} />
      ))}
      <DockScanOverlay viewKind={dockViewKind} active={!dockOpen && (isCamSource || is3d)} variant="modern" />
      {dockShown && <DockView />}
      <P2gOverlay active={p2gAvail && p2g} />
      {guideCam != null && <DriveGuideOverlay streamId={guideCam} />}
      <PtzOverlay />
      {viewportOverlays.map((Overlay, i) => <Overlay key={i} />)}
    </View>
  );

  const controls = !chrome ? null : (
    <>
      {<View style={[styles.ddAnchor, topInset ? { top: EDGE + topInset } : null]} pointerEvents="box-none">
        <Tappable
          onPress={onToggleOpen}
          disabled={dockOpen}
          style={[styles.dd, { width: DD_W, backgroundColor: c.glass, borderColor: dockOpen ? c.amber : c.glassLine, borderRadius: radius.md }]}
        >
          <Text style={{ color: dockOpen ? c.amber : c.text, fontSize: 12, fontWeight: '700' }}>
            {dockOpen ? t('도킹') : isSim ? t(SIM_CAMS.find((x) => x.key === simCamNow)?.label ?? '3인칭') : spectating && cur?.kind === 'camera' ? t('조종자 화면') : cur ? t(cur.label) : '—'}
          </Text>
          {!dockOpen && <Text style={{ color: c.dim, fontSize: 10 }}>▾</Text>}
        </Tappable>
      </View>}
    </>
  );





  if (maximized) {
    return (
      <View
        style={[StyleSheet.absoluteFill, { alignItems: 'center', backgroundColor: c.bg }]}
        onLayout={(e) => {
          const { width: w, height: h } = e.nativeEvent.layout;
          setFullBox((p) => (Math.abs(p.w - w) < 1 && Math.abs(p.h - h) < 1 ? p : { w, h }));
        }}>
        <View style={fw ? { width: fw, height: fh } : StyleSheet.absoluteFill}>
          {clipBox}
          {controls}
        </View>
      </View>
    );
  }

  return (
    <View style={{ alignItems: 'center' }}>
      <View style={{ width: vw, height: vh }}>
        {clipBox}
        {controls}
      </View>
    </View>
  );
}


export function SourceDropdownList({ open, selectedKey, onSelect, onToggleOpen, style }: {
  open: boolean;
  selectedKey?: string;
  onSelect: (key: string) => void;
  onToggleOpen: () => void;
  style?: StyleProp<ViewStyle>;
}) {
  const { c, radius } = useTheme();
  const simCam = useSimCourse()?.cam ?? 'third';
  const sim = selectedKey === 'sim';
  const avail = useAvailableSources();
  const spectating = useSpectating();
  const driverKey = avail.find((s) => s.kind === 'camera')?.key ?? 'front';
  const sources = sim ? SIM_CAMS
    : spectating ? [{ key: 'pose3d', label: '3D' }, { key: driverKey, label: '조종자 화면' }]
    : avail.filter((s) => s.kind !== 'sim').map((s) => ({ key: s.key, label: s.label }));
  const curKey = sim ? simCam
    : spectating ? (selectedKey === 'pose3d' ? 'pose3d' : driverKey)
    : (sources.find((s) => s.key === selectedKey) ?? sources[0])?.key;
  const pick = (k: string) => {
    if (!sim) { onSelect(k); return; }
    simEngine.setCam(k as SimCam);
    onToggleOpen();
  };
  if (!open) return null;
  return (
    <View style={[StyleSheet.absoluteFill, { zIndex: 50, elevation: 50 }, style]} pointerEvents="box-none">
      <Pressable style={StyleSheet.absoluteFill} onPress={onToggleOpen} />
      <View style={styles.ddListAnchor} pointerEvents="box-none">
        <Animated.View entering={unfoldDown} exiting={foldUp} style={[styles.ddList, { transformOrigin: 'top center' } as any, { width: DD_W, backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
          {sources.map((s) => (
            <Pressable
              key={s.key}
              onPress={() => pick(s.key)}
              style={[styles.ddItem, s.key === curKey && { backgroundColor: 'rgba(77,156,245,0.22)' }]}
            >
              <Text style={{ color: s.key === curKey ? c.accent : c.text, fontSize: 12 }}>{t(s.label)}</Text>
            </Pressable>
          ))}
        </Animated.View>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  simBar: { position: 'absolute', top: 76, left: 0, right: 0, alignItems: 'center' },
  simPill: { paddingHorizontal: 12, paddingVertical: 4, borderRadius: 999, borderWidth: 1 },
  simTxt: { fontSize: 11, fontVariant: ['tabular-nums'] },
  ddAnchor: { position: 'absolute', top: EDGE, alignSelf: 'center', left: 0, right: 0, alignItems: 'center', zIndex: 95 },
  dd: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 44, borderWidth: 1 },
  ddListAnchor: { position: 'absolute', top: EDGE + 44 + 4, left: 0, right: 0, alignItems: 'center' },
  ddList: { borderWidth: 1, overflow: 'hidden' },
  ddItem: { paddingHorizontal: 12, paddingVertical: 9, alignItems: 'center' },
  vsBtn: { position: 'absolute', right: 10, width: 32, height: 32, alignItems: 'center', justifyContent: 'center', zIndex: 10 },
  stairsPose: { position: 'absolute', top: 0, bottom: 0, right: 0 },
  obsMapPanel: { position: 'absolute', top: 0, bottom: 0, right: 0 },
});

const SIM_CAMS: { key: SimCam; label: string }[] = [
  { key: 'third', label: '3인칭' }, { key: 'front', label: '1인칭 전방' },
  { key: 'back', label: '1인칭 후방' }, { key: 'stairs', label: '계단 뷰' },
];

function courseLine(st: { map: string; course: { passed: number; total: number; done: boolean; elapsed: number; falls: number }; rtf: number }) {
  const lv = levelById(st.map);
  const base = lv
    ? `${lv.no}. ${t(lv.name)} · ${st.course.done ? `${t('클리어')} ${fmtTime(st.course.elapsed)}` : `${t('게이트')} ${st.course.passed}/${st.course.total} · ${fmtTime(st.course.elapsed)}`}${st.course.falls ? ` · ${t('낙하 {n}회').replace('{n}', String(st.course.falls))}` : ''}`
    : st.map;
  return st.rtf > 0 && st.rtf < 0.9 ? `${base} · ${t('실시간의 {x}배').replace('{x}', st.rtf.toFixed(2))}` : base;
}

function useOperatorLive(on: boolean): number | null {
  const [id, setId] = useState<number | null>(null);
  useEffect(() => {
    if (!on) { setId(null); return; }
    let alive = true;
    const tick = () => streamerCommand('GET', '/api/vision/stream/live').then(
      (r) => { if (alive) setId(typeof r?.id === 'number' ? r.id : null); },
      () => { if (alive) setId(null); });
    tick();
    const h = setInterval(tick, 1000);
    return () => { alive = false; clearInterval(h); };
  }, [on]);
  return id;
}
