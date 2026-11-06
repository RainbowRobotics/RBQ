import { useEffect } from 'react';
import { StyleSheet, View } from 'react-native';
import Animated, { useSharedValue, useAnimatedStyle, withTiming, Easing } from 'react-native-reanimated';

function flatten(y: number, viewH: number) {
  const t = viewH > 0 ? Math.max(0, Math.min(1, y / viewH)) : 0.5;
  return 0.28 + t * 0.34;
}

export function P2gTapMarker({ x, y, viewH = 0, color = '#ffc93c', aiming = false, onDone }: {
  x: number; y: number; viewH?: number; color?: string; aiming?: boolean; onDone: () => void;
}) {
  const p = useSharedValue(0);
  useEffect(() => {
    if (aiming) return;
    p.value = withTiming(1, { duration: 620, easing: Easing.out(Easing.cubic) });
    const t = setTimeout(onDone, 660);
    return () => clearTimeout(t);
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const W = 46;
  const H = W * flatten(y, viewH);
  const ring = useAnimatedStyle(() => (aiming
    ? { opacity: 0.95, transform: [{ scale: 1 }] }
    : { opacity: 1 - p.value, transform: [{ scale: 1 + p.value * 0.9 }] }));
  const core = useAnimatedStyle(() => (aiming
    ? { opacity: 0.5, transform: [{ scale: 1 }] }
    : { opacity: 1 - p.value * 0.9, transform: [{ scale: 1 - p.value * 0.35 }] }));

  return (
    <>
      <Animated.View pointerEvents="none" style={[styles.ellipse, ring, {
        left: x - W / 2, top: y - H / 2, width: W, height: H, borderRadius: W / 2, borderColor: color,
        borderWidth: aiming ? 2 : 3,
      }]} />
      <Animated.View pointerEvents="none" style={[styles.ellipse, core, {
        left: x - W / 4, top: y - H / 4, width: W / 2, height: H / 2, borderRadius: W / 4,
        backgroundColor: color, borderWidth: 0,
      }]} />
      {aiming && (
        <View pointerEvents="none"
          style={[styles.stem, { left: x - 1, top: y - H / 2 - 14, height: 14, backgroundColor: color }]} />
      )}
    </>
  );
}

const styles = StyleSheet.create({
  ellipse: { position: 'absolute' },
  stem: { position: 'absolute', width: 2, opacity: 0.85, borderRadius: 1 },
});
