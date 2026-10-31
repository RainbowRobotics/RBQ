import { View, Text, Image, StyleSheet, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { t } from '@/lib/i18n';

const VISUAL_CHECK_IMAGE: number | null = null;

const ITEMS = [
  '하박(정강이) 링크를 최대한 접은 상태에서 링크에 휘어짐이 생겼는지 확인하세요.',
  '무릎 쪽 볼트가 풀린 곳이 없는지 확인하세요.',
  '발 고무의 마모 상태를 보고, 좌우 다리 사이에 차이(불균형)가 없는지 확인하세요.',
];

export function VisualCheckModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const { width: winW } = useWindowDimensions();
  return (
    <Modal onClose={onClose}>
      <View style={[styles.box, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: Math.min(winW - 24, 640) }]}>
        <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{t('다리 육안검사')}</Text>
        <Text style={{ color: c.muted, fontSize: 12.5 }}>{t('로봇을 앉힌 상태에서 네 다리를 차례로 확인하세요.')}</Text>
        <View style={[styles.photo, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
          {VISUAL_CHECK_IMAGE != null
            ? <Image source={VISUAL_CHECK_IMAGE} style={StyleSheet.absoluteFill} resizeMode="contain" />
            : <Text style={{ color: c.dim, fontSize: 12 }}>{t('이미지 준비 중')}</Text>}
        </View>
        <View style={{ gap: 8 }}>
          {ITEMS.map((s, i) => (
            <View key={s} style={{ flexDirection: 'row', gap: 8 }}>
              <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '800', width: 16 }}>{i + 1}</Text>
              <Text style={{ color: c.text, fontSize: 13, lineHeight: 19, flex: 1 }}>{t(s)}</Text>
            </View>
          ))}
        </View>
        <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
        </Tappable>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  box: { maxWidth: '94%', borderWidth: 1, padding: 18, gap: 12 },
  photo: { height: 220, borderWidth: 1, alignItems: 'center', justifyContent: 'center', overflow: 'hidden' },
  btn: { height: 44, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
