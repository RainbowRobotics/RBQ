import { View, Text, StyleSheet } from 'react-native';
import { useRouter } from 'expo-router';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useRobot } from '@/store/robot';
import { useAccessLevel } from '@/store/settings';
import { healthRows } from '@/lib/health';
import { t } from '@/lib/i18n';

const TONE = '#FF7A70';

export function HealthBanner() {
  const { radius } = useTheme();
  const router = useRouter();
  const level = useAccessLevel();
  const conn = useRobot((s) => s.conn);
  const findings = useRobot((s) => s.health?.findings);
  const faults = healthRows(findings?.filter((f) => f.severity === 'fault'), level);
  if (conn !== 'connected' || !faults.length) return null;
  const first = faults[0];
  const to = first.to;
  const more = level >= 2 && faults.length > 1 ? ` ${t('외 {n}건').replace('{n}', String(faults.length - 1))}` : '';
  const body = (
    <>
      <Icon name="warn" size={14} color={TONE} />
      <Text numberOfLines={1} style={[styles.txt, { color: TONE }]}>{`${t('확인 필요')}: ${first.text}${more}`}</Text>
    </>
  );
  const style = [styles.banner, { borderColor: TONE, borderRadius: radius.md }];
  return to ? (
    <Tappable onPress={() => router.navigate(to)} accessibilityLabel={t('확인 필요 항목 열기')} style={style}>{body}</Tappable>
  ) : (
    <View style={style}>{body}</View>
  );
}

const styles = StyleSheet.create({
  banner: {
    flexDirection: 'row', alignItems: 'center', gap: 8, maxWidth: '92%',
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
    backgroundColor: 'rgba(0,0,0,0.55)',
  },
  txt: { fontSize: 11.5, fontWeight: '700', flexShrink: 1 },
});
