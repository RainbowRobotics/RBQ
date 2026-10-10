import { View, Text, StyleSheet, Platform } from 'react-native';
import { useDeviceBattery } from '@/lib/deviceBattery';
import { isDesktop } from '@/lib/desktopBridge';
import { useRb } from '@/rb/theme';
import { useTheme } from '@/theme';
import { RBBattery } from '@/rb/components/RBBattery';
import { t } from '@/lib/i18n';

export function Battery({ pct, label, charging = false }: {
  pct: number | null; label?: string; charging?: boolean;
  dense?: boolean;
}) {
  const { c } = useRb();
  const { fonts } = useTheme();
  const has = pct != null;
  const v = has ? Math.max(0, Math.min(100, Math.round(pct))) : undefined;
  return (
    <View style={styles.row} accessibilityLabel={`${label ?? ''} ${has ? `${v}%` : '—'}`}>
      {label ? <Text style={[styles.label, { color: c('fg-subtler') }]}>{label}</Text> : null}
      <RBBattery size="sm" percent={v} status={has ? undefined : 'empty'} charging={charging && has && (v ?? 0) < 100} />
      <Text style={[styles.num, { fontFamily: fonts.mono, color: !has ? c('fg-subtlest') : (v ?? 0) < 25 ? c('fg-danger') : c('fg-default') }]}>{has ? `${v}%` : '—'}</Text>
    </View>
  );
}

export function DeviceBattery(_: { dense?: boolean } = {}) {
  const batt = useDeviceBattery();
  if (!batt) return null;
  const label = isDesktop() ? 'PC' : Platform.OS === 'web' ? t('기기') : t('패드');
  return <Battery pct={batt.pct} label={label} charging={batt.charging} />;
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', alignItems: 'center', gap: 3 },
  label: { fontSize: 9, fontWeight: '700' },
  num: { fontSize: 10.5, fontWeight: '700', includeFontPadding: false },
});
