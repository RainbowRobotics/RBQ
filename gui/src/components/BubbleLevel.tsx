import { useEffect, useRef } from 'react';
import { View, Text, Animated, Easing } from 'react-native';
import Svg, { Circle, Line } from 'react-native-svg';
import { useTheme } from '@/theme';
import { useTelemetry } from '@/store/telemetry';
import { t } from '@/lib/i18n';

const AnimatedCircle = Animated.createAnimatedComponent(Circle);
const STARTS: [number, number][] = [[0.75, -0.6], [-0.7, 0.55], [0.35, 0.8]];

export function BubbleLevel({ size = 132 }: { size?: number }) {
  const { c } = useTheme();
  const R = size / 2, track = R - 6, bubbleR = size * 0.11, reach = track - bubbleR;
  const cx = useRef(new Animated.Value(R + reach * STARTS[0][0])).current;
  const cy = useRef(new Animated.Value(R + reach * STARTS[0][1])).current;
  useEffect(() => {
    const glideIn = (sx: number, sy: number) => Animated.sequence([
      Animated.parallel([
        Animated.timing(cx, { toValue: R - reach * 0.2 * sx, duration: 650, easing: Easing.out(Easing.quad), useNativeDriver: false }),
        Animated.timing(cy, { toValue: R - reach * 0.2 * sy, duration: 650, easing: Easing.out(Easing.quad), useNativeDriver: false }),
      ]),
      Animated.parallel([
        Animated.spring(cx, { toValue: R, friction: 6, tension: 45, useNativeDriver: false }),
        Animated.spring(cy, { toValue: R, friction: 6, tension: 45, useNativeDriver: false }),
      ]),
      Animated.delay(1100),
    ]);
    const driftOut = (sx: number, sy: number) => Animated.parallel([
      Animated.timing(cx, { toValue: R + reach * sx, duration: 900, easing: Easing.inOut(Easing.quad), useNativeDriver: false }),
      Animated.timing(cy, { toValue: R + reach * sy, duration: 900, easing: Easing.inOut(Easing.quad), useNativeDriver: false }),
    ]);
    const steps: Animated.CompositeAnimation[] = [];
    STARTS.forEach(([sx, sy], i) => {
      steps.push(glideIn(sx, sy));
      const [nx, ny] = STARTS[(i + 1) % STARTS.length];
      steps.push(driftOut(nx, ny));
    });
    const loop = Animated.loop(Animated.sequence(steps));
    loop.start();
    return () => loop.stop();
  }, [cx, cy, R, reach]);
  return (
    <Svg width={size} height={size} viewBox={`0 0 ${size} ${size}`}>
      <Circle cx={R} cy={R} r={track} fill={c.elev} stroke={c.line} strokeWidth={2} />
      <Line x1={R - track} y1={R} x2={R + track} y2={R} stroke={c.line} strokeWidth={1} />
      <Line x1={R} y1={R - track} x2={R} y2={R + track} stroke={c.line} strokeWidth={1} />
      <Circle cx={R} cy={R} r={reach * 0.2 + bubbleR} fill="none" stroke={c.green} strokeWidth={1.5} strokeDasharray="4 3" />
      <AnimatedCircle cx={cx} cy={cy} r={bubbleR} fill={c.accent2} opacity={0.9} stroke={c.panel} strokeWidth={2} />
    </Svg>
  );
}

export function ImuRollPitch() {
  const { c, fonts, radius } = useTheme();
  const rpy = useTelemetry((s) => s.robot?.imu.rpy);
  const fmt = (rad: number | undefined) => {
    if (rad == null) return '—';
    const d = (rad * 180) / Math.PI;
    return `${d >= 0 ? '+' : ''}${d.toFixed(2)}°`;
  };
  const cell = (label: string, rad: number | undefined) => (
    <View style={{ flexDirection: 'row', alignItems: 'baseline', gap: 6 }}>
      <Text style={{ color: c.muted, fontSize: 12, fontWeight: '700' }}>{label}</Text>
      <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', fontFamily: fonts.mono }}>{fmt(rad)}</Text>
    </View>
  );
  return (
    <View style={{ flexDirection: 'row', gap: 18, paddingVertical: 6, paddingHorizontal: 14,
      borderWidth: 1, borderColor: c.line, backgroundColor: c.elev, borderRadius: radius.sm }}>
      {cell('Roll', rpy?.[0])}
      {cell('Pitch', rpy?.[1])}
    </View>
  );
}

export function ImuLevelHint() {
  const { c } = useTheme();
  return (
    <View style={{ alignItems: 'center', marginTop: 12, gap: 8 }}>
      <BubbleLevel />
      <ImuRollPitch />
      <Text style={{ color: c.text, fontSize: 12, fontWeight: '600', textAlign: 'center', lineHeight: 17 }}>
        {t('수평계를 이용하여 로봇 상체의 수평을 맞추십시오.')}
      </Text>
    </View>
  );
}
