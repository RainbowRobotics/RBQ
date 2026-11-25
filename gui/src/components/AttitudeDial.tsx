import { View, Text, StyleSheet } from 'react-native';
import { horizonShift } from './attitude';

export function AttitudeDial({ size, roll, pitch, colors }: {
  size: number;
  roll: number;
  pitch: number;
  colors: { sky: string; ground: string; line: string; cross: string; border: string };
}) {
  const perDeg = size / 60;
  const pitchOffset = Math.max(-size / 3, Math.min(size / 3, horizonShift(pitch, size)));

  const ticks: number[] = [];
  for (let p = -40; p <= 40; p += 5) if (p !== 0) ticks.push(p);
  const showLabels = size >= 80;
  const fontSize = Math.max(6, Math.round(size * 0.075));

  return (
    <View style={{ width: size, height: size }}>
      <View style={[styles.dial, {
        width: size, height: size, borderRadius: size / 2, borderColor: colors.border,
        transform: [{ rotate: `${-roll}deg` }],
      }]}>
        <View style={{ position: 'absolute', left: -size, right: -size, top: -size, height: size * 1.5 + pitchOffset, backgroundColor: colors.sky }} />
        <View style={{ position: 'absolute', left: -size, right: -size, top: size / 2 + pitchOffset, height: size * 1.5, backgroundColor: colors.ground }} />
        <View style={{ position: 'absolute', left: 0, right: 0, top: size / 2 + pitchOffset - 1, height: 2, backgroundColor: colors.line }} />
        {ticks.map((p) => {
          const y = size / 2 + horizonShift(pitch, size) + p * perDeg;
          if (y < 2 || y > size - 2) return null;
          const isLong = p % 10 === 0;
          const dashW = isLong ? size * 0.32 : size * 0.2;
          return (
            <View key={p} pointerEvents="none"
              style={{ position: 'absolute', top: y - 0.5, left: size / 2 - dashW / 2, width: dashW, height: 1, backgroundColor: colors.line }}>
              {showLabels && isLong && (
                <>
                  <Text style={{ position: 'absolute', right: dashW + 3, top: -fontSize / 1.4, fontSize, color: colors.line, fontWeight: '700' }}>{p}</Text>
                  <Text style={{ position: 'absolute', left: dashW + 3, top: -fontSize / 1.4, fontSize, color: colors.line, fontWeight: '700' }}>{p}</Text>
                </>
              )}
            </View>
          );
        })}
      </View>
      <View pointerEvents="none" style={[StyleSheet.absoluteFill, styles.center]}>
        <View style={{ width: Math.round(size * 0.42), height: 2, backgroundColor: colors.cross }} />
        <View style={{ position: 'absolute', width: 2, height: Math.round(size * 0.14), backgroundColor: colors.cross }} />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  dial: { overflow: 'hidden', borderWidth: 2 },
  center: { alignItems: 'center', justifyContent: 'center' },
});
