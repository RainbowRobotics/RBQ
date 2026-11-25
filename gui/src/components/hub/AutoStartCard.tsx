import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { t } from '@/lib/i18n';

export function AutoStartCard({ onPress, disabled }: { onPress: () => void; disabled?: boolean }) {
  const { c, radius } = useTheme();
  return (
    <Tappable
      onPress={disabled ? undefined : onPress}
      style={[
        styles.auto,
        { backgroundColor: c.panel, borderRadius: radius.md },
        disabled ? { borderColor: 'rgba(128,128,128,0.4)' } : { borderColor: 'rgba(63,185,80,0.5)' },
      ]}
    >
      <View style={[styles.playTri, { borderLeftColor: disabled ? c.muted : c.green }]} />
      <Text style={[styles.autoTxt, { color: disabled ? c.muted : c.greenTx }]}>{t('자동 기동')}</Text>
    </Tappable>
  );
}

const styles = StyleSheet.create({
  auto: {
    flexDirection: 'row', alignItems: 'center', gap: 9, height: 40, paddingHorizontal: 16, borderWidth: 1, alignSelf: 'flex-start',
    shadowColor: '#000', shadowOpacity: 0.15, shadowRadius: 10, shadowOffset: { width: 0, height: 3 }, elevation: 5,
  },
  playTri: {
    width: 0, height: 0, borderLeftWidth: 9, borderTopWidth: 6, borderBottomWidth: 6,
    borderTopColor: 'transparent', borderBottomColor: 'transparent',
  },
  autoTxt: { fontWeight: '600', fontSize: 13 },
});
