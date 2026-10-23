import { View, Text, StyleSheet, type StyleProp, type ViewStyle } from 'react-native';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';

export function RailButton({ icon, label, active, dashed, dot, hint, onPress, onLongPress, delayLongPress, style, accessibilityLabel }: {
  icon: IconName; label: string; active?: boolean; dashed?: boolean;
  dot?: boolean;
  hint?: boolean;
  onPress?: () => void; onLongPress?: () => void; delayLongPress?: number;
  style?: StyleProp<ViewStyle>; accessibilityLabel?: string;
}) {
  const { c, radius } = useTheme();
  const fg = active ? c.accent2 : dashed ? c.muted : c.text;
  return (
    <Tappable onPress={onPress} onLongPress={onLongPress} delayLongPress={delayLongPress} accessibilityLabel={accessibilityLabel ?? label}
      style={[styles.btn, {
        borderRadius: radius.md, borderStyle: dashed ? 'dashed' : 'solid',
        backgroundColor: active ? 'rgba(77,156,245,0.22)' : dashed ? 'transparent' : c.glassHi,
        borderColor: active ? 'rgba(77,156,245,0.6)' : c.glassLine,
      }, style]}>
      <Icon name={icon} size={24} color={fg} />
      <Text numberOfLines={1} style={{ color: fg, fontSize: 12, fontWeight: '700', letterSpacing: 0.2 }}>{label}</Text>
      {dot && <View style={[styles.dot, { backgroundColor: c.green }]} />}
      {hint && <View style={[styles.hint, { backgroundColor: c.dim }]} />}
    </Tappable>
  );
}

const styles = StyleSheet.create({
  btn: { height: 50, alignSelf: 'stretch', borderWidth: 1, alignItems: 'center', justifyContent: 'center', gap: 3, overflow: 'hidden' },
  dot: { position: 'absolute', right: 6, top: 6, width: 6, height: 6, borderRadius: 3 },
  hint: { position: 'absolute', right: 5, bottom: 5, width: 4, height: 4, borderRadius: 2 },
});
