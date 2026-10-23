import { View, Text, Image, StyleSheet, ActivityIndicator, useWindowDimensions, type DimensionValue } from 'react-native';
import Constants from 'expo-constants';
import { useTheme } from '@/theme';
import { BOOT_ROBOT, BOOT_ROBOT_RATIO } from '@/components/bootRobot';

export const BOOT_BG = '#EEF1F4';

export function BootScreen() {
  const { c } = useTheme();
  const { width, height } = useWindowDimensions();
  const known = width > 0 && height > 0;
  const row = known ? width >= 720 && width > height : true;
  const short = Math.min(width, height);
  const boxH = Math.round(Math.min(
    row ? height * 0.72 : height * 0.42,
    (row ? width * 0.42 : width * 0.78) / BOOT_ROBOT_RATIO,
    460,
  ));
  const R = BOOT_ROBOT_RATIO;
  const art = known
    ? { width: Math.round(boxH * R), height: boxH }
    : {
        width: `min(42vw, calc(72vh * ${R}), ${Math.round(460 * R)}px)` as DimensionValue,
        height: `min(72vh, calc(42vw / ${R}), 460px)` as DimensionValue,
      };
  const title = Math.round(Math.max(26, Math.min(short * 0.075, 46)));
  const titleSize = known ? title : ('clamp(26px, 7.5vmin, 46px)' as unknown as number);
  const subSize = known ? Math.round(title * 0.36) : ('calc(0.36 * clamp(26px, 7.5vmin, 46px))' as unknown as number);

  return (
    <View style={[styles.root, { backgroundColor: c.bg, flexDirection: row ? 'row' : 'column' }]}>
      <View style={[styles.copy, row ? { alignItems: 'flex-start', paddingLeft: known ? Math.round(width * 0.08) : ('8vw' as DimensionValue) } : { alignItems: 'center' }]}>
        <Text style={[styles.title, { fontSize: titleSize, color: c.accent, textAlign: row ? 'left' : 'center' }]}>RBQ</Text>
        <Text style={[styles.sub, { fontSize: subSize, color: c.muted, textAlign: row ? 'left' : 'center' }]}>
          Rainbow Robotics Quadruped
        </Text>
        <View style={styles.loading}>
          <ActivityIndicator size="small" color={c.accent} />
          <Text style={[styles.ver, { color: c.dim }]}>v{Constants.expoConfig?.version ?? '—'}</Text>
        </View>
      </View>
      <View style={[styles.art, row ? null : { marginTop: 8 }]}>
        <Image source={BOOT_ROBOT} style={art} resizeMode="contain" />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  root: { flex: 1, alignItems: 'center', justifyContent: 'center' },
  copy: { justifyContent: 'center', gap: 6 },
  art: { alignItems: 'center', justifyContent: 'center' },
  title: { fontWeight: '800', letterSpacing: 1 },
  sub: { fontWeight: '500', letterSpacing: 0.3 },
  loading: { flexDirection: 'row', alignItems: 'center', gap: 9, marginTop: 18 },
  ver: { fontSize: 11, fontWeight: '600' },
});
