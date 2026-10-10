import { useEffect } from 'react';
import { Pressable, View, type StyleProp, type ViewStyle, type PressableProps } from 'react-native';
import Animated, {
  useSharedValue, useAnimatedStyle, withSpring, withRepeat, withTiming, Keyframe, Easing,
} from 'react-native-reanimated';

const AnimatedPressable = Animated.createAnimatedComponent(Pressable);

const IN_MS = 180;
const OUT_MS = 120;
export const unfoldDown = new Keyframe({
  0: { opacity: 0, transform: [{ translateY: -10 }, { scale: 0.92 }] },
  100: { opacity: 1, transform: [{ translateY: 0 }, { scale: 1 }], easing: Easing.out(Easing.cubic) },
}).duration(IN_MS);
export const foldUp = new Keyframe({
  0: { opacity: 1, transform: [{ translateY: 0 }, { scale: 1 }] },
  100: { opacity: 0, transform: [{ translateY: -8 }, { scale: 0.94 }] },
}).duration(OUT_MS);
export const unfoldUp = new Keyframe({
  0: { opacity: 0, transform: [{ translateY: 10 }, { scale: 0.92 }] },
  100: { opacity: 1, transform: [{ translateY: 0 }, { scale: 1 }], easing: Easing.out(Easing.cubic) },
}).duration(IN_MS);
export const foldDown = new Keyframe({
  0: { opacity: 1, transform: [{ translateY: 0 }, { scale: 1 }] },
  100: { opacity: 0, transform: [{ translateY: 8 }, { scale: 0.94 }] },
}).duration(OUT_MS);
export const popIn = new Keyframe({
  0: { opacity: 0, transform: [{ scale: 0.94 }] },
  100: { opacity: 1, transform: [{ scale: 1 }], easing: Easing.out(Easing.cubic) },
}).duration(IN_MS);
export const popOut = new Keyframe({
  0: { opacity: 1, transform: [{ scale: 1 }] },
  100: { opacity: 0, transform: [{ scale: 0.96 }] },
}).duration(OUT_MS);

export function Tappable({
  children, onPress, onLongPress, delayLongPress, style, hitSlop, disabled, scaleTo = 0.97, accessibilityLabel,
}: {
  children?: React.ReactNode;
  onPress?: () => void;
  onLongPress?: () => void;
  delayLongPress?: number;
  style?: StyleProp<ViewStyle>;
  hitSlop?: PressableProps['hitSlop'];
  disabled?: boolean;
  scaleTo?: number;
  accessibilityLabel?: string;
}) {
  const s = useSharedValue(1);
  const a = useAnimatedStyle(() => ({ transform: [{ scale: s.value }] }));
  return (
    <AnimatedPressable
      accessibilityLabel={accessibilityLabel}
      onPress={onPress}
      onLongPress={onLongPress}
      delayLongPress={delayLongPress}
      disabled={disabled}
      hitSlop={hitSlop}
      onPressIn={() => { s.value = withTiming(scaleTo, { duration: 70 }); }}
      onPressOut={() => { s.value = withTiming(1, { duration: 110 }); }}
      style={[style, a]}
    >
      {children}
    </AnimatedPressable>
  );
}

export function Pulse({ color, size = 9, halo = 18 }: { color: string; size?: number; halo?: number }) {
  const p = useSharedValue(0);
  useEffect(() => {
    p.value = withRepeat(withTiming(1, { duration: 1500 }), -1, true);
  }, [p]);
  const haloStyle = useAnimatedStyle(() => ({
    opacity: 0.35 - p.value * 0.3,
    transform: [{ scale: 0.7 + p.value * 0.9 }],
  }));
  return (
    <View style={{ width: size, height: size, alignItems: 'center', justifyContent: 'center' }}>
      <Animated.View
        style={[
          { position: 'absolute', width: halo, height: halo, borderRadius: halo / 2, backgroundColor: color },
          haloStyle,
        ]}
      />
      <View style={{ width: size, height: size, borderRadius: size / 2, backgroundColor: color }} />
    </View>
  );
}
