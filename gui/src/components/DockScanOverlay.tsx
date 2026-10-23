import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, Pressable, Image, type LayoutChangeEvent } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { useTelemetry } from '@/store/telemetry';
import { useVisionToggles } from '@/store/visionToggles';
import { useRobot } from '@/store/robot';
import { useCamCalibRunning } from '@/components/CameraCalib';
import { connection } from '@/lib/connection';
import { MARKER_CAM } from '@/lib/arucoState';
import { useInputMode } from '@/store/inputMode';
import { t } from '@/lib/i18n';

const BOX_FILL = 'rgba(29,148,81,0.3)';
const BOX_BORDER = '#0B6623';
const TRI_FILL = '#FBBA16';
const TRI_BORDER = '#00492C';
const BTN_TEXT = '#187B25';
const BTN_BG = '#EEEBE4';
const BTN_BG_ACTIVE = '#FBBA16';
const HOLD_MS = 1500;

const REF_IMAGES = {
  bottom: require('@/assets/images/aruco_marker/bottom_id.png'),
  id5: require('@/assets/images/aruco_marker/ID5_75mm_marker.png'),
  rear: require('@/assets/images/aruco_marker/rear_id.png'),
};

export type DockViewKind = 'front' | 'rear' | 'stacked' | 'other';

export function streamIdToViewKind(id: number): DockViewKind {
  return id === 1 ? 'front' : id === 2 ? 'rear' : id === 11 ? 'stacked' : 'other';
}

function UpTriangle() {
  return (
    <View style={{ width: 36, height: 24, alignItems: 'center', justifyContent: 'center' }}>
      <View style={[styles.tri, { borderLeftWidth: 18, borderRightWidth: 18, borderBottomWidth: 24, borderBottomColor: TRI_BORDER }]} />
      <View style={[styles.tri, { position: 'absolute', top: 2, borderLeftWidth: 15, borderRightWidth: 15, borderBottomWidth: 20, borderBottomColor: TRI_FILL }]} />
    </View>
  );
}

export function DockScanOverlay({ viewKind, active, variant = 'modern' }: { viewKind: DockViewKind; active: boolean; variant?: 'legacy' | 'modern' }) {
  const { radius } = useTheme();
  const marker = useTelemetry((s) => s.markerState);
  const dockScan = useVisionToggles((s) => s.dockScan);
  const dockOverride = useVisionToggles((s) => s.dockOverride);
  const dockingStatus = useRobot((s) => s.robot?.docking_status);
  const oakCalib = useCamCalibRunning();
  const touchJoy = useInputMode() === 'touch';
  const [size, setSize] = useState({ w: 0, h: 0 });
  const onLayout = (e: LayoutChangeEvent) => {
    const { width, height } = e.nativeEvent.layout;
    setSize((p) => (p.w === width && p.h === height ? p : { w: width, h: height }));
  };

  const dockingRunning = dockingStatus != null && dockingStatus >= 1 && dockingStatus <= 7 && !dockOverride;

  const detectedNow = !!(active && (dockScan || dockingRunning) && marker?.detected);
  const [held, setHeld] = useState(false);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  useEffect(() => {
    if (detectedNow) {
      if (timer.current) { clearTimeout(timer.current); timer.current = null; }
      setHeld(true);
    } else {
      if (timer.current) clearTimeout(timer.current);
      timer.current = setTimeout(() => { setHeld(false); timer.current = null; }, HOLD_MS);
    }
    return () => { if (timer.current) { clearTimeout(timer.current); timer.current = null; } };
  }, [detectedNow]);

  if (!active || !marker) return null;

  const markerFound = held && dockScan && !dockingRunning && !oakCalib;
  const sized = size.w > 0 && size.h > 0;
  const showAction = markerFound && sized;
  const showVis = held && !oakCalib && (dockScan || dockingRunning) && sized;

  const camIsFront = marker.cam === MARKER_CAM.front;
  const camIsRear = marker.cam === MARKER_CAM.rear;
  const camIsBottom = marker.cam === MARKER_CAM.bottom;
  const joyBottom = touchJoy ? JOY_RESERVE : 12;
  const isFrontView = viewKind === 'front';
  const isRearView = viewKind === 'rear';
  const isStackedView = viewKind === 'stacked';

  const showFrontBox = camIsFront && (isFrontView || isStackedView);
  const showRearFullBox = camIsRear && isRearView;
  const showRearPipBox = camIsRear && isStackedView;
  const showBox = showFrontBox || showRearFullBox || showRearPipBox;
  const showBottomIndicator = !showBox;

  const refSrc = camIsBottom ? REF_IMAGES.bottom : marker.ver === 2 ? REF_IMAGES.id5 : REF_IMAGES.rear;
  const refSize = Math.round(size.h * 0.22);

  const boxU0 = showRearPipBox ? 0.375 : 0;
  const boxScale = showRearPipBox ? 0.25 : 1;
  const startDock = () => connection.sendMotion('dock');

  const modern = variant === 'modern';
  const bbox = (
    <Pressable
      onPress={startDock}
      style={{
        position: 'absolute',
        left: (boxU0 + marker.uMin * boxScale) * size.w - 30 * boxScale,
        top: marker.vMin * boxScale * size.h - 30 * boxScale,
        width: (marker.uMax - marker.uMin) * boxScale * size.w + 60 * boxScale,
        height: (marker.vMax - marker.vMin) * boxScale * size.h + 60 * boxScale,
        backgroundColor: modern ? 'rgba(29,148,81,0.16)' : BOX_FILL,
        borderColor: modern ? '#1D9451' : BOX_BORDER,
        borderWidth: modern ? 2.5 : 6,
        borderRadius: modern ? radius.md : 0,
      }}
    />
  );

  return (
    <View style={StyleSheet.absoluteFill} pointerEvents="box-none" onLayout={onLayout}>
      {showVis && showBox && bbox}
      {showVis && showBottomIndicator && !modern && (
        <View style={styles.bottomInd} pointerEvents="none">
          <UpTriangle />
        </View>
      )}

      {showAction && !modern && (<>
      <View style={[styles.actionStack, { bottom: joyBottom }]}>
        <Pressable onPress={startDock}>
          <Image
            source={refSrc}
            style={{ width: refSize, height: refSize, borderColor: BOX_BORDER, borderWidth: Math.max(2, Math.round(size.h * 0.008)), borderRadius: radius.sm }}
            resizeMode="contain"
          />
        </Pressable>
        <Pressable
          onPress={startDock}
          style={({ pressed }) => [styles.btn, { width: refSize, backgroundColor: pressed ? BTN_BG_ACTIVE : BTN_BG, borderColor: TRI_BORDER }]}
        >
          <Text style={{ color: BTN_TEXT, fontSize: 13, fontWeight: '700' }}>START DOCKING</Text>
        </Pressable>
      </View>
      </>)}

      {showAction && modern && (<>
      <View pointerEvents="box-none" style={styles.pillWrap}>
        {showBottomIndicator && (
          <Pressable onPress={startDock} style={[styles.refCard, { borderRadius: radius.md }]}>
            <Image source={refSrc} style={{ width: refSize * 0.8, height: refSize * 0.8, borderRadius: radius.sm }} resizeMode="contain" />
          </Pressable>
        )}
        <Pressable
          onPress={startDock}
          style={({ pressed }) => [styles.pill, { borderRadius: radius.md, borderColor: '#00C853', backgroundColor: pressed ? 'rgba(0,200,83,0.25)' : 'rgba(0,0,0,0.55)' }]}
        >
          <Icon name="anchor" size={14} color="#00C853" />
          <Text style={{ color: '#00C853', fontSize: 11.5, fontWeight: '700' }}>{t('도킹 시작')}</Text>
        </Pressable>
      </View>
      </>)}
    </View>
  );
}

const JOY_RESERVE = 128 + 24;

const styles = StyleSheet.create({
  tri: { width: 0, height: 0, borderLeftColor: 'transparent', borderRightColor: 'transparent' },
  bottomInd: { position: 'absolute', left: 0, right: 0, bottom: 12, alignItems: 'center' },
  actionStack: { position: 'absolute', right: 12, bottom: 12, alignItems: 'center', gap: 6 },
  btn: { height: 34, alignItems: 'center', justifyContent: 'center', borderWidth: 2, paddingHorizontal: 8 },
  pillWrap: { position: 'absolute', bottom: 70, left: 0, right: 0, alignItems: 'center', gap: 8, zIndex: 30 },
  pill: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1 },
  refCard: { alignItems: 'center', padding: 8, backgroundColor: 'rgba(0,0,0,0.55)' },
});
