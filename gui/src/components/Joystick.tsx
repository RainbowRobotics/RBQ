import { useEffect, useRef } from 'react';
import { View, StyleSheet } from 'react-native';
import Svg, { Defs, RadialGradient, Stop, Circle, Path } from 'react-native-svg';
import { Gesture, GestureDetector } from 'react-native-gesture-handler';
import Animated, { useSharedValue, useAnimatedStyle, withTiming, runOnJS } from 'react-native-reanimated';
import { useTheme } from '@/theme';
import { useSettings } from '@/store/settings';
import { applyStickCurve } from '@/lib/gamepad/curve';
import { createStickGate, type StickGate } from '@/lib/stickGate';

export const JOY_SIZE = 158;
const SIZE = JOY_SIZE;
const KNOB = 62;
const MAX_R = 46;

export function Joystick({ onMove, tone = 'default', size = SIZE }: {
  onMove?: (nx: number, ny: number) => void;
  tone?: 'default' | 'legacy';
  size?: number;
}) {
  const { c } = useTheme();
  const knobSz = Math.round((size * KNOB) / SIZE);
  const maxR = (size * MAX_R) / SIZE;
  const ringIn = Math.round((size * 26) / SIZE);
  const tickIn = Math.round((size * 10) / SIZE);
  const tickLen = Math.round((size * 11) / SIZE);
  const legacy = tone === 'legacy';
  const dish = legacy ? { a: c.legacyJoyA, b: c.legacyJoyB, c3: c.legacyJoyC, line: c.legacyKnobLine } : { a: c.joyA, b: c.joyB, c3: c.joyC, line: c.line };
  const knob = legacy ? { a: c.legacyKnobA, b: c.legacyKnobB, line: c.legacyKnobLine } : { a: c.knobA, b: c.knobB, line: c.knobLine };
  const tx = useSharedValue(0);
  const ty = useSharedValue(0);

  const onMoveRef = useRef(onMove);
  onMoveRef.current = onMove;
  const gateRef = useRef<StickGate | null>(null);
  if (!gateRef.current) gateRef.current = createStickGate((x, y) => onMoveRef.current?.(x, y));
  const gate = gateRef.current;
  useEffect(() => { gate.mount(); return () => gate.unmount(); }, [gate]);

  const emit = (x: number, y: number) => {
    const { deadzone, sensitivity } = useSettings.getState();
    const out = applyStickCurve(x / maxR, -y / maxR, deadzone, sensitivity);
    gate.move(out.x, out.y);
  };
  const begin = () => gate.begin();
  const release = () => gate.release();

  const pan = Gesture.Pan()
    .minDistance(0)
    .onBegin(() => {
      'worklet';
      runOnJS(begin)();
    })
    .onStart(() => {
      'worklet';
      runOnJS(begin)();
    })
    .onUpdate((e) => {
      'worklet';
      let x = e.translationX;
      let y = e.translationY;
      const d = Math.sqrt(x * x + y * y);
      if (d > maxR) {
        x = (x / d) * maxR;
        y = (y / d) * maxR;
      }
      tx.value = x;
      ty.value = y;
      runOnJS(emit)(x, y);
    })
    .onFinalize(() => {
      'worklet';
      tx.value = withTiming(0, { duration: 90 });
      ty.value = withTiming(0, { duration: 90 });
      runOnJS(release)();
    });

  const arcStyle = useAnimatedStyle(() => {
    const d = Math.sqrt(tx.value * tx.value + ty.value * ty.value);
    return {
      opacity: Math.min(1, d / (maxR * 0.35)),
      transform: [{ rotate: `${(Math.atan2(ty.value, tx.value) * 180) / Math.PI + 90}deg` }],
    };
  });

  const knobStyle = useAnimatedStyle(() => ({
    transform: [{ translateX: tx.value }, { translateY: ty.value }],
  }));

  const arcR = size / 2 - Math.max(3, size * 0.045);
  const ax = (deg: number) => size / 2 + arcR * Math.cos((deg * Math.PI) / 180);
  const ay = (deg: number) => size / 2 + arcR * Math.sin((deg * Math.PI) / 180);
  const arcPath = `M ${ax(-150)} ${ay(-150)} A ${arcR} ${arcR} 0 0 1 ${ax(-30)} ${ay(-30)}`;

  return (
    <View style={[styles.base, { width: size, height: size, borderRadius: size / 2 }]}>
      <Svg width={size} height={size} style={StyleSheet.absoluteFill}>
        <Defs>
          <RadialGradient id="dish" cx="50%" cy="38%" r="75%">
            <Stop offset="0%" stopColor={dish.a} />
            <Stop offset="70%" stopColor={dish.b} />
            <Stop offset="100%" stopColor={dish.c3} />
          </RadialGradient>
        </Defs>
        <Circle cx={size / 2} cy={size / 2} r={size / 2 - 2} fill="url(#dish)" fillOpacity={legacy ? 1 : 0.5}
          stroke={dish.line} strokeWidth={1.5} strokeOpacity={legacy ? 1 : 0.6} />
      </Svg>
      <Animated.View style={[StyleSheet.absoluteFill, arcStyle]} pointerEvents="none">
        <Svg width={size} height={size}>
          <Path d={arcPath} stroke={c.accent} strokeWidth={Math.max(3, size * 0.045)} strokeLinecap="round" fill="none" />
        </Svg>
      </Animated.View>
      <View style={[styles.ring, {
        top: ringIn, left: ringIn, right: ringIn, bottom: ringIn,
        borderRadius: (size - ringIn * 2) / 2, borderColor: 'rgba(127,127,127,0.22)',
      }]} />
      <View style={[styles.tick, { top: tickIn, width: 2, height: tickLen, backgroundColor: c.dim }]} />
      <View style={[styles.tick, { bottom: tickIn, width: 2, height: tickLen, backgroundColor: c.dim }]} />
      <View style={[styles.tick, { left: tickIn, height: 2, width: tickLen, backgroundColor: c.dim }]} />
      <View style={[styles.tick, { right: tickIn, height: 2, width: tickLen, backgroundColor: c.dim }]} />
      <GestureDetector gesture={pan}>
        <View style={[legacy ? { width: knobSz, height: knobSz } : StyleSheet.absoluteFill,
          { alignItems: 'center', justifyContent: 'center' }]}>
        <Animated.View style={[styles.knobWrap, { width: knobSz, height: knobSz }, knobStyle]}>
          <Svg width={knobSz} height={knobSz}>
            <Defs>
              <RadialGradient id="knob" cx="50%" cy="35%" r="75%">
                <Stop offset="0%" stopColor={knob.a} />
                <Stop offset="75%" stopColor={knob.b} />
              </RadialGradient>
            </Defs>
            <Circle cx={knobSz / 2} cy={knobSz / 2} r={knobSz / 2 - 1} fill="url(#knob)" stroke={knob.line} strokeWidth={1} />
            <Circle cx={knobSz / 2} cy={knobSz / 2} r={Math.round((size * 9) / SIZE)} fill={c.accent} />
          </Svg>
        </Animated.View>
        </View>
      </GestureDetector>
    </View>
  );
}

const styles = StyleSheet.create({
  base: {
    alignItems: 'center', justifyContent: 'center', overflow: 'hidden',
  },
  ring: {
    position: 'absolute', borderWidth: 1, borderStyle: 'dashed',
  },
  tick: { position: 'absolute', opacity: 0.5, borderRadius: 1 },
  knobWrap: { position: 'absolute' },
});
