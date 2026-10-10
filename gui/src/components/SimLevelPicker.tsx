import { View, Text, ScrollView, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { LEVELS, fmtTime } from '@/lib/simCourse';
import { useSimProgress } from '@/store/simProgress';
import { t } from '@/lib/i18n';

const THEME_LABEL: Record<string, string> = { site: '공사현장', mountain: '산악지대' };

export function SimLevelPicker({ current, onPick, onClose }: {
  current: string;
  onPick: (id: string) => void;
  onClose: () => void;
}) {
  const { c, radius } = useTheme();
  const cleared = useSimProgress((s) => s.cleared);
  const n = LEVELS.filter((l) => cleared[l.id]).length;
  return (
    <Modal onClose={onClose} fit>
      <View style={[styles.box, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
        <View style={styles.head}>
          <Text style={{ color: c.text, fontSize: 15, fontWeight: '700', flex: 1 }}>{t('시뮬 레벨')}</Text>
          <Text style={{ color: c.muted, fontSize: 12 }}>{t('클리어')} {n}/{LEVELS.length}</Text>
        </View>
        <ScrollView style={{ flexShrink: 1 }} contentContainerStyle={{ gap: 6 }}>
          {LEVELS.map((l) => {
            const rec = cleared[l.id], on = l.id === current;
            return (
              <Tappable key={l.id} onPress={() => onPick(l.id)} accessibilityLabel={`${l.no}. ${t(l.name)}`}
                style={[styles.row, { borderColor: on ? c.accent : c.line, backgroundColor: on ? c.elev : 'transparent', borderRadius: radius.md }]}>
                <Text style={[styles.no, { color: c.muted }]}>{String(l.no).padStart(2, '0')}</Text>
                <View style={{ flex: 1, gap: 2 }}>
                  <Text style={{ color: c.text, fontSize: 13, fontWeight: '600', lineHeight: 17 }}>
                    {t(l.name)} <Text style={{ color: c.dim, fontSize: 11, fontWeight: '400' }}>{t(THEME_LABEL[l.theme ?? ''] ?? '')}</Text>
                  </Text>
                  {!!l.teach && <Text style={{ color: c.muted, fontSize: 11, lineHeight: 15 }} numberOfLines={1}>{t(l.teach)}</Text>}
                </View>
                {rec ? (
                  <View style={{ alignItems: 'flex-end' }}>
                    <Text style={{ color: c.green, fontSize: 13, fontWeight: '800' }}>✓</Text>
                    <Text style={{ color: c.muted, fontSize: 11 }}>{fmtTime(rec.best)}</Text>
                  </View>
                ) : null}
              </Tappable>
            );
          })}
        </ScrollView>
        <Text style={{ color: c.dim, fontSize: 11, lineHeight: 15, marginTop: 10 }}>
          {t('게이트를 번호 순서대로 모두 지나 결승 발판에 서면 클리어. 떨어지면 마지막 게이트에서 다시 시작합니다.')}
        </Text>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  box: { width: 380, maxWidth: '94%', borderWidth: 1, padding: 16, flexShrink: 1 },
  head: { flexDirection: 'row', alignItems: 'center', marginBottom: 10 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingHorizontal: 12, paddingVertical: 8, borderWidth: 1 },
  no: { fontSize: 12, fontWeight: '700', width: 20, fontVariant: ['tabular-nums'] },
});
