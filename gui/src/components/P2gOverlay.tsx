import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, Animated, Easing, type LayoutChangeEvent } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { useTelemetry } from '@/store/telemetry';
import { P2G_STATE } from '@/lib/p2gState';
import { t } from '@/lib/i18n';

const TARGET_RADIUS_M = 0.25;

function pinSize(viewW: number, dist: number, fxNorm: number): number {
  if (!(fxNorm > 0)) return 0;
  const r = (fxNorm * TARGET_RADIUS_M / Math.max(0.3, dist)) * viewW;
  return Math.min(viewW * 0.12, r);
}

const MOVE_TONE = '#7CB8FF';
const SHADOW = { textShadowColor: 'rgba(0,0,0,0.75)', textShadowOffset: { width: 0, height: 1 }, textShadowRadius: 3 } as const;

export function P2gOverlay({ active }: { active: boolean }) {
  const { c, fonts, radius } = useTheme();
  const p2g = useTelemetry((s) => s.p2gState);
  const [size, setSize] = useState({ w: 0, h: 0 });
  const onLayout = (e: LayoutChangeEvent) => {
    const { width, height } = e.nativeEvent.layout;
    setSize((prev) => (prev.w === width && prev.h === height ? prev : { w: width, h: height }));
  };

  const pulse = useRef(new Animated.Value(0)).current;
  const moving = active && p2g?.state === P2G_STATE.moving;
  useEffect(() => {
    if (!moving) { pulse.stopAnimation(); pulse.setValue(0); return; }
    const loop = Animated.loop(Animated.timing(pulse, {
      toValue: 1, duration: 1600, easing: Easing.out(Easing.ease), useNativeDriver: true,
    }));
    loop.start();
    return () => loop.stop();
  }, [moving, pulse]);

  if (!moving || p2g == null) return null;

  const tone = MOVE_TONE;
  const { w, h } = size;

  const pin = pinSize(w, p2g.distHorizontal, p2g.fxNorm);
  const inFrame = p2g.visible && pin > 0 && p2g.u >= 0 && p2g.u <= 1 && p2g.v >= 0 && p2g.v <= 1;

  return (
    <View style={StyleSheet.absoluteFill} pointerEvents="none" onLayout={onLayout}>
      {inFrame && w > 0 && h > 0 && (
        <View style={{ position: 'absolute', left: p2g.u * w, top: p2g.v * h }}>
          {[0, 0.5].map((delay) => (
            <Animated.View
              key={delay}
              style={{
                position: 'absolute', left: -pin, top: -pin * 0.32,
                width: pin * 2, height: pin * 0.64, borderRadius: pin,
                borderWidth: 2, borderColor: tone,
                opacity: pulse.interpolate({
                  inputRange: [0, delay, Math.min(1, delay + 0.5), 1],
                  outputRange: delay === 0 ? [0.9, 0.9, 0, 0] : [0, 0.9, 0, 0],
                }),
                transform: [{
                  scale: pulse.interpolate({
                    inputRange: [0, delay, Math.min(1, delay + 0.5), 1],
                    outputRange: delay === 0 ? [0.55, 0.55, 1.7, 1.7] : [0.55, 0.55, 1.7, 1.7],
                  }),
                }],
              }}
            />
          ))}
          <View style={{
            position: 'absolute', left: -pin * 0.7, top: -pin * 0.22,
            width: pin * 1.4, height: pin * 0.44, borderRadius: pin,
            borderWidth: 2.5, borderColor: 'rgba(0,0,0,0.45)',
          }} />
          <View style={{
            position: 'absolute', left: -pin * 0.68, top: -pin * 0.2,
            width: pin * 1.36, height: pin * 0.4, borderRadius: pin,
            backgroundColor: 'rgba(124,184,255,0.14)', borderWidth: 1.5, borderColor: tone,
          }} />
          <View style={{ position: 'absolute', left: -2.5, top: -pin * 1.55, width: 5, height: pin * 1.55, borderRadius: 3, backgroundColor: 'rgba(0,0,0,0.45)' }} />
          <View style={{ position: 'absolute', left: -1, top: -pin * 1.55, width: 2, height: pin * 1.55, backgroundColor: tone }} />
          <View style={{
            position: 'absolute', left: -pin * 0.22, top: -pin * 1.77,
            width: pin * 0.44, height: pin * 0.44, borderRadius: pin,
            backgroundColor: tone, borderWidth: 2.5, borderColor: 'rgba(0,0,0,0.5)',
            alignItems: 'center', justifyContent: 'center',
          }}>
            <View style={{ width: pin * 0.14, height: pin * 0.14, borderRadius: pin, backgroundColor: 'rgba(0,0,0,0.55)' }} />
          </View>
          <View style={[styles.badge, { top: -pin * 1.77 - 29, borderRadius: radius.sm, borderColor: tone }]}>
            <Text style={[{ color: tone, fontFamily: fonts.mono, fontSize: 11.5, fontWeight: '700' }, SHADOW]}>
              {p2g.distHorizontal.toFixed(2)} m
            </Text>
          </View>
        </View>
      )}

      {!inFrame && (
        <View style={[styles.offscreen, { borderRadius: radius.md, borderColor: tone }]}>
          <Icon name="anchor" size={13} color={tone} />
          <Text style={[{ color: tone, fontFamily: fonts.mono, fontSize: 11.5, fontWeight: '700' }, SHADOW]}>
            {p2g.distHorizontal.toFixed(2)} m
          </Text>
          <Text style={{ color: 'rgba(255,255,255,0.7)', fontSize: 10.5 }}>{t('목표가 화면 밖입니다')}</Text>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  badge: {
    position: 'absolute', left: -34, width: 68, alignItems: 'center', justifyContent: 'center',
    height: 23, borderWidth: 1.5, backgroundColor: 'rgba(0,0,0,0.62)',
  },
  offscreen: {
    position: 'absolute', bottom: 104, alignSelf: 'center', flexDirection: 'row', alignItems: 'center',
    gap: 8, paddingHorizontal: 12, paddingVertical: 7, borderWidth: 1.5, backgroundColor: 'rgba(0,0,0,0.62)',
  },
});
