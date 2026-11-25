import { View, Text, StyleSheet, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useRobots } from '@/store/robots';
import { t } from '@/lib/i18n';

export function LanOfferBanner() {
  const { c, radius } = useTheme();
  const offer = useRobots((s) => s.lanOffer);
  if (!offer) return null;
  return (
    <View style={[styles.banner, { backgroundColor: c.panel, borderColor: c.accent, borderRadius: radius.md }]}>
      <Icon name="wifi" size={14} color={c.accent} />
      <Text numberOfLines={1} style={{ color: c.text, fontSize: 12, fontWeight: '700', flexShrink: 1 }}>{offer.name}</Text>
      <Text numberOfLines={1} style={{ color: c.muted, fontSize: 11 }}>{t('이 WiFi에 있습니다')}</Text>
      <Tappable onPress={() => void useRobots.getState().acceptLanOffer()} accessibilityLabel={t('연결')}
        style={[styles.btn, { backgroundColor: c.accent, borderRadius: radius.sm }]}>
        <Text style={{ color: c.onAccent, fontSize: 12, fontWeight: '700' }}>{t('연결')}</Text>
      </Tappable>
      <Pressable onPress={() => useRobots.getState().dismissLanOffer()} hitSlop={8} accessibilityLabel={t('닫기')}>
        <Icon name="x" size={13} color={c.muted} />
      </Pressable>
    </View>
  );
}

const styles = StyleSheet.create({
  banner: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingLeft: 14, paddingRight: 10, paddingVertical: 6, borderWidth: 1 },
  btn: { paddingHorizontal: 14, paddingVertical: 6 },
});
