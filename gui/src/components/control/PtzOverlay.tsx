import { useEffect, useRef } from 'react';
import { View, Text, StyleSheet, Pressable, useWindowDimensions } from 'react-native';
import Animated, { useAnimatedStyle, useSharedValue } from 'react-native-reanimated';
import { Gesture, GestureDetector } from 'react-native-gesture-handler';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Joystick } from '@/components/Joystick';
import { Slider } from '@/components/ui/controls';
import { ptz } from '@/lib/vision';
import { useViewport } from '@/store/viewport';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { RIGHT_X, RIGHT_W } from '@/components/control/RightPanel';
import { t } from '@/lib/i18n';
import { useDeviceToggles } from '@/store/deviceToggles';

const ZOOMS = [1, 3, 8, 16, 32] as const;

const PANEL_EST_H = 210;

export const PTZ_SOURCE_KEYS = ['cctv', 'thermal', 'ptzmix'] as const;
export const isPtzSourceKey = (k: string) => (PTZ_SOURCE_KEYS as readonly string[]).includes(k);

function Btn({ label, on, disabled, onPress }: {
  label: string; on?: boolean; disabled?: boolean; onPress?: () => void;
}) {
  const { c, radius } = useTheme();
  return (
    <Pressable disabled={disabled} onPress={onPress}
      style={({ pressed }) => [styles.btn, {
        borderRadius: radius.sm, opacity: disabled ? 0.35 : 1,
        backgroundColor: on ? 'rgba(77,156,245,0.28)' : pressed ? 'rgba(255,255,255,0.18)' : 'rgba(255,255,255,0.06)',
        borderColor: on ? c.accent : 'rgba(255,255,255,0.2)',
      }]}>
      <Text style={{ color: '#fff', fontSize: 11, fontWeight: '700' }}>{label}</Text>
    </Pressable>
  );
}

function Row({ label, value, children }: { label: string; value: string; children: React.ReactNode }) {
  const { c } = useTheme();
  return (
    <View style={styles.row}>
      <Text style={[styles.rowLab, { color: c.dim }]}>{label}</Text>
      <View style={{ flex: 1 }}>{children}</View>
      <Text style={[styles.rowVal, { color: '#fff' }]}>{value}</Text>
    </View>
  );
}

export function PtzOverlay() {
  const { c, radius } = useTheme();
  const vpKey = useViewport((s) => s.key);
  const sens = useViewport((s) => s.ptzSens);
  const setSens = useViewport((s) => s.setPtzSens);
  const zi = Math.max(0, ZOOMS.indexOf(useDeviceToggles((s) => s.ptzZoom) as (typeof ZOOMS)[number]));
  const setZi = (i: number) => useDeviceToggles.setState({ ptzZoom: ZOOMS[i] });
  const faceTemp = useDeviceToggles((s) => s.ptzFaceTemp);
  const setFaceTemp = (v: boolean) => useDeviceToggles.setState({ ptzFaceTemp: v });

  const isMix = vpKey === 'ptzmix';
  const shown = isPtzSourceKey(vpKey);

  const win = useWindowDimensions();
  const insets = useSafeAreaInsets();
  const GAP = 12;
  const right = RIGHT_X + RIGHT_W + GAP + insets.right;
  const top = Math.min(58 + 3 * (50 + 6), Math.max(12, win.height - PANEL_EST_H - 12));

  const dx = useSharedValue(0);
  const dy = useSharedValue(0);
  const start = useSharedValue({ x: 0, y: 0 });
  const drag = Gesture.Pan()
    .onStart(() => { start.value = { x: dx.value, y: dy.value }; })
    .onUpdate((e) => { dx.value = start.value.x + e.translationX; dy.value = start.value.y + e.translationY; });
  const panelStyle = useAnimatedStyle(() => ({ transform: [{ translateX: dx.value }, { translateY: dy.value }] }));

  const axes = useRef({ x: 0, y: 0 });
  const wasMoving = useRef(false);
  useEffect(() => {
    if (!shown) return;
    const id = setInterval(() => {
      const { x, y } = axes.current;
      const moving = x !== 0 || y !== 0;
      if (moving) { ptz.velocity(-x * sens, -y * sens, 0); wasMoving.current = true; }
      else if (wasMoving.current) { ptz.velocity(0, 0, 0); wasMoving.current = false; }
    }, 40);
    return () => { clearInterval(id); if (wasMoving.current) { ptz.velocity(0, 0, 0); wasMoving.current = false; } };
  }, [shown, sens]);

  useEffect(() => {
    if (!isMix && faceTemp) { setFaceTemp(false); ptz.faceTemp(false); }
  }, [isMix]); // eslint-disable-line react-hooks/exhaustive-deps

  if (!shown) return null;

  const pickZoom = (idx: number) => {
    const i = Math.max(0, Math.min(ZOOMS.length - 1, idx));
    setZi(i);
    ptz.zoomX(ZOOMS[i]);
  };

  return (
    <View pointerEvents="box-none" style={[styles.wrap, { right: right, top: top }]}>
      <Animated.View style={[styles.panel, panelStyle, { backgroundColor: 'rgba(13,17,23,0.78)', borderColor: c.line, borderRadius: radius.md }]}>
        <GestureDetector gesture={drag}>
          <View style={styles.grip}>
            <Icon name="move" size={14} color={c.muted} />
          </View>
        </GestureDetector>

        <View style={{ alignItems: 'center' }}>
          <Joystick size={92} onMove={(nx, ny) => { axes.current = { x: nx, y: ny }; }} />
        </View>

        <Row label={t('줌')} value={`${ZOOMS[zi]}x`}>
          <Slider width="100%" value={(zi / (ZOOMS.length - 1)) * 100}
            onChange={(v) => pickZoom(Math.round((v / 100) * (ZOOMS.length - 1)))} />
        </Row>
        <View style={styles.zoomRow}>
          {ZOOMS.map((z, i) => (
            <Btn key={z} label={`${z}x`} on={zi === i} onPress={() => pickZoom(i)} />
          ))}
        </View>
        <Row label={t('감도')} value={`${Math.round(sens * 100)}%`}>
          <Slider width="100%" value={sens * 100}
            onChange={(v) => setSens(Math.max(10, v) / 100)} />
        </Row>

        <View style={styles.btnRow}>
          <Btn label="Center" onPress={() => ptz.returnCenter()} />
          {isMix && (
            <Btn label={`FaceTemp ${faceTemp ? 'ON' : 'OFF'}`} on={faceTemp}
              onPress={() => { const nv = !faceTemp; setFaceTemp(nv); ptz.faceTemp(nv); }} />
          )}
        </View>
      </Animated.View>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { position: 'absolute', zIndex: 20 },
  panel: { width: 236, paddingHorizontal: 10, paddingTop: 8, paddingBottom: 10, borderWidth: 1, gap: 6 },
  grip: { position: 'absolute', right: 0, top: -2, width: 30, height: 26, alignItems: 'center', justifyContent: 'center', zIndex: 2 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  rowLab: { fontSize: 9.5, fontWeight: '700', width: 30 },
  rowVal: { fontSize: 10.5, fontWeight: '700', width: 38, textAlign: 'right' },
  zoomRow: { flexDirection: 'row', gap: 4, justifyContent: 'space-between' },
  btnRow: { flexDirection: 'row', gap: 6, marginTop: 2 },
  btn: { height: 26, minWidth: 34, paddingHorizontal: 7, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
});
