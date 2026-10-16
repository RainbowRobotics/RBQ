import { useEffect, useRef } from 'react';
import { View, StyleSheet, Platform, type StyleProp, type ViewStyle } from 'react-native';
import { Gesture, GestureDetector } from 'react-native-gesture-handler';
import Animated, {
  useSharedValue, useAnimatedStyle, withTiming, runOnJS, cancelAnimation,
} from 'react-native-reanimated';
import { useTheme } from '@/theme';

const LONG_MS = 500;
const MAX_DIST = 48;

type Props = {
  onPress?: () => void;
  onLongPress?: () => void;
  style?: StyleProp<ViewStyle>;
  children: React.ReactNode;
  disabled?: boolean;
  gaugeColor?: string;
};

export function LongPressButton(props: Props) {
  return Platform.OS === 'web' ? <WebLongPress {...props} /> : <NativeLongPress {...props} />;
}

function WebLongPress({ onPress, onLongPress, style, children, disabled = false, gaugeColor }: Props) {
  const { c } = useTheme();
  const rootRef = useRef<any>(null);
  const gaugeRef = useRef<any>(null);
  const cb = useRef({ onPress, onLongPress, disabled });
  cb.current = { onPress, onLongPress, disabled };

  useEffect(() => {
    const el: any = rootRef.current;
    if (!el || typeof el.addEventListener !== 'function') return;

    const s = { active: false, fired: false, startX: 0, startY: 0, startT: 0, raf: 0 };

    const setGauge = (p: number) => {
      const g: any = gaugeRef.current;
      if (g && g.style) { g.style.width = `${p * 100}%`; g.style.opacity = p > 0 ? '1' : '0'; }
    };
    const setDim = (on: boolean) => { if (el.style) el.style.opacity = on ? '0.75' : '1'; };
    const reset = () => {
      s.active = false;
      if (s.raf) { cancelAnimationFrame(s.raf); s.raf = 0; }
      setGauge(0);
      setDim(false);
    };

    const onDown = (e: any) => {
      if (cb.current.disabled) return;
      s.active = true; s.fired = false;
      s.startX = e.clientX; s.startY = e.clientY; s.startT = Date.now();
      setDim(true);
      try { el.setPointerCapture?.(e.pointerId); } catch {}
      if (cb.current.onLongPress) {
        const loop = () => {
          if (!s.active) return;
          const p = Math.min(1, (Date.now() - s.startT) / LONG_MS);
          setGauge(p);
          if (p >= 1) {
            if (!s.fired) { s.fired = true; cb.current.onLongPress?.(); }
            return;
          }
          s.raf = requestAnimationFrame(loop);
        };
        s.raf = requestAnimationFrame(loop);
      }
    };
    const onMove = (e: any) => {
      if (!s.active) return;
      const dx = e.clientX - s.startX, dy = e.clientY - s.startY;
      if (dx * dx + dy * dy > MAX_DIST * MAX_DIST) {
        s.fired = true;
        reset();
      }
    };
    const onUp = () => {
      if (!s.active) { reset(); return; }
      const wasLong = s.fired;
      reset();
      if (!wasLong) cb.current.onPress?.();
    };
    const onCancel = () => { reset(); };
    const onCtx = (e: any) => { e.preventDefault(); e.stopPropagation(); };

    el.addEventListener('pointerdown', onDown);
    el.addEventListener('pointermove', onMove);
    el.addEventListener('pointerup', onUp);
    el.addEventListener('pointercancel', onCancel);
    el.addEventListener('pointerleave', onCancel);
    el.addEventListener('contextmenu', onCtx);
    return () => {
      el.removeEventListener('pointerdown', onDown);
      el.removeEventListener('pointermove', onMove);
      el.removeEventListener('pointerup', onUp);
      el.removeEventListener('pointercancel', onCancel);
      el.removeEventListener('pointerleave', onCancel);
      el.removeEventListener('contextmenu', onCtx);
      if (s.raf) cancelAnimationFrame(s.raf);
    };
  }, []);

  return (
    <View
      ref={rootRef}
      // @ts-expect-error
      style={[style, { overflow: 'hidden', touchAction: 'none', userSelect: 'none', WebkitTouchCallout: 'none', cursor: disabled ? 'default' : 'pointer' }]}
      pointerEvents={disabled ? 'none' : 'auto'}
    >
      {children}
      {onLongPress && (
        <View style={styles.gaugeTrack} pointerEvents="none">
          <View ref={gaugeRef} style={[styles.gauge, { backgroundColor: gaugeColor ?? c.accent, width: '0%', opacity: 0 }]} />
        </View>
      )}
    </View>
  );
}

function NativeLongPress({ onPress, onLongPress, style, children, disabled = false, gaugeColor }: Props) {
  const { c } = useTheme();
  const progress = useSharedValue(0);
  const pressed = useSharedValue(0);

  const long = Gesture.LongPress()
    .minDuration(LONG_MS)
    .maxDistance(MAX_DIST)
    .enabled(!disabled && !!onLongPress)
    .onBegin(() => {
      pressed.value = 1;
      progress.value = withTiming(1, { duration: LONG_MS });
    })
    .onStart(() => {
      if (onLongPress) runOnJS(onLongPress)();
    })
    .onFinalize(() => {
      pressed.value = 0;
      cancelAnimation(progress);
      progress.value = withTiming(0, { duration: 120 });
    });

  const tap = Gesture.Tap()
    .maxDuration(LONG_MS)
    .maxDistance(MAX_DIST)
    .enabled(!disabled)
    .onBegin(() => { pressed.value = 1; })
    .onEnd(() => {
      if (onPress) runOnJS(onPress)();
    })
    .onFinalize(() => { pressed.value = 0; });

  const gesture = onLongPress ? Gesture.Exclusive(long, tap) : tap;

  const gaugeStyle = useAnimatedStyle(() => ({
    width: `${progress.value * 100}%`,
    opacity: progress.value > 0 ? 1 : 0,
  }));
  const dimStyle = useAnimatedStyle(() => ({ opacity: pressed.value ? 0.75 : 1 }));

  return (
    <GestureDetector gesture={gesture}>
      <Animated.View style={[style, dimStyle, { overflow: 'hidden' }]}>
        {children}
        {onLongPress && (
          <View style={styles.gaugeTrack} pointerEvents="none">
            <Animated.View style={[styles.gauge, { backgroundColor: gaugeColor ?? c.accent }, gaugeStyle]} />
          </View>
        )}
      </Animated.View>
    </GestureDetector>
  );
}

const styles = StyleSheet.create({
  gaugeTrack: { position: 'absolute', left: 0, right: 0, bottom: 0, height: 3 },
  gauge: { height: 3 },
});
