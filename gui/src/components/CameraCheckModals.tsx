import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { t } from '@/lib/i18n';

function Frame({ title, width, onClose, children }: { title: string; width: number; onClose: () => void; children: React.ReactNode }) {
  const { c, radius } = useTheme();
  const { width: winW } = useWindowDimensions();
  return (
    <Modal onClose={onClose}>
      <View style={[styles.box, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: Math.min(winW - 24, width) }]}>
        <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{title}</Text>
        {children}
        <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
        </Tappable>
      </View>
    </Modal>
  );
}

export function CameraBasicCheckModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  return (
    <Frame title={t('카메라 기본 검사')} width={860} onClose={onClose}>
      <Text style={{ color: c.muted, fontSize: 12.5 }}>{t('카메라마다 시야가 가려지거나 흐리지 않은지 보고, FPS 가 제대로 나오는지 확인하세요.')}</Text>
      <View style={styles.grid}>
        {[1, 2, 3, 4, 5, 6].map((n) => (
          <View key={n} style={[styles.tile, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Text style={{ color: c.dim, fontSize: 12 }}>{t('영상 준비 중')}</Text>
            <Text style={[styles.cap, { color: c.muted }]}>{t('카메라 {n}').replace('{n}', String(n))} · — FPS</Text>
          </View>
        ))}
      </View>
    </Frame>
  );
}

export function DepthOnChipModal({ onClose }: { onClose: () => void }) {
  const { c } = useTheme();
  return (
    <Frame title={t('하단 depth 카메라 On-chip 캘리브레이션')} width={560} onClose={onClose}>
      <Text style={{ color: c.text, fontSize: 13, lineHeight: 19 }}>
        {t('하단 depth 카메라의 On-chip 캘리브레이션을 진행합니다. 로봇을 평평한 바닥에 세워 두고 끝날 때까지 움직이지 마세요.')}
      </Text>
      <Text style={{ color: c.dim, fontSize: 12 }}>{t('보정 시퀀스는 준비 중입니다.')}</Text>
    </Frame>
  );
}

export function DepthExtrinsicModal({ onClose }: { onClose: () => void }) {
  const { c } = useTheme();
  return (
    <Frame title={t('하단 depth 카메라 위치 보정')} width={560} onClose={onClose}>
      <Text style={{ color: c.text, fontSize: 13, lineHeight: 19 }}>{t('로봇 하단에 마커보드를 설치해 주세요.')}</Text>
      <Text style={{ color: c.dim, fontSize: 12 }}>{t('보정 시퀀스는 준비 중입니다.')}</Text>
    </Frame>
  );
}

const styles = StyleSheet.create({
  box: { maxWidth: '94%', borderWidth: 1, padding: 18, gap: 12 },
  grid: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  tile: { flexGrow: 1, flexBasis: 240, height: 150, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  cap: { position: 'absolute', left: 8, bottom: 6, fontSize: 11, fontWeight: '600' },
  btn: { height: 44, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
