import { View, Text, Pressable } from 'react-native';
import { router, usePathname } from 'expo-router';
import { t } from '@/lib/i18n';
import { kindLabel, shortTime, verdictWord } from '@/lib/ptzQc';
import { markPtzQcSeen, usePtzQcUnseen } from '@/store/ptzQc';

export function PtzQcBanner() {
  const unseen = usePtzQcUnseen();
  const path = usePathname();
  if (!unseen || path.startsWith('/maintenance')) return null;
  return (
    <View pointerEvents="box-none" style={{ position: 'absolute', top: 10, left: 0, right: 0, alignItems: 'center', zIndex: 998 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 12, maxWidth: 640, marginHorizontal: 16, paddingVertical: 9, paddingHorizontal: 14,
        backgroundColor: 'rgba(22,27,34,0.94)', borderRadius: 10, borderWidth: 1, borderColor: 'rgba(77,156,245,0.6)' }}>
        <Text style={{ color: '#fff', fontSize: 12.5, fontWeight: '600', flexShrink: 1 }} numberOfLines={2}>
          {t('PTZ 검사 결과가 있습니다')} — {shortTime(unseen.startedAt)} {kindLabel(unseen.kind, unseen.soakMin)} · {verdictWord(unseen.verdict)}
        </Text>
        <Pressable onPress={() => router.push({ pathname: '/maintenance', params: { sec: 'ptzqc' } })} hitSlop={6} accessibilityRole="button">
          <Text style={{ color: '#7cb7ff', fontSize: 12.5, fontWeight: '700' }}>{t('결과 보기')}</Text>
        </Pressable>
        <Pressable onPress={() => markPtzQcSeen(unseen.stem)} hitSlop={8} accessibilityRole="button" accessibilityLabel={t('닫기')}>
          <Text style={{ color: '#fff', fontSize: 14, fontWeight: '800' }}>✕</Text>
        </Pressable>
      </View>
    </View>
  );
}
