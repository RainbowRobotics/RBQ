import { View, Text, ScrollView, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import { useRb } from '@/rb/theme';
import { getLang } from '@/store/lang';
import { t } from '@/lib/i18n';
import { RBIconButton } from '@/rb/components/RBIconButton';
import { Icon } from '@/components/Icon';
import { flaggedText, kindLabel, shortTime, verdictWord, type HistoryRow } from '@/lib/ptzQc';
import { BigBtn, Card, Note, VerdictChip } from '@/components/ptzqc/parts';

export function ResultsView({ rows, staff, busy, unseen, error, onOpen, onRerun, onDelete }: {
  rows: HistoryRow[]; staff: boolean; busy: boolean; unseen: HistoryRow | null; error: string | null;
  onOpen: (r: HistoryRow) => void; onRerun: (r: HistoryRow) => void; onDelete: (r: HistoryRow) => void;
}) {
  const { c, fonts } = useTheme();
  const { c: rc } = useRb();
  const en = getLang() === 'en';
  const latest = rows.find((r) => !r.running) ?? null;
  const cell = (flex: number, minWidth: number) => ({ flex, minWidth });
  const head = { color: c.dim, fontSize: 11.5, fontWeight: '600' as const };
  return (
    <ScrollView contentContainerStyle={{ gap: 12, paddingBottom: 12 }} showsVerticalScrollIndicator={false}>
      {unseen ? (
        <Note icon="check" title={t('PTZ 검사 결과가 있습니다')}
          text={`— ${shortTime(unseen.startedAt)} ${kindLabel(unseen.kind, unseen.soakMin)} · ${verdictWord(unseen.verdict)}`}
          right={<BigBtn small kind="ghost" icon="log" label={t('보기')} onPress={() => onOpen(unseen)} />} />
      ) : null}
      {error ? <Note tone="warn" text={`${t('이력을 받지 못했습니다')} — ${error}`} /> : null}
      <Card>
        {rows.length ? (
          <ScrollView horizontal showsHorizontalScrollIndicator={false} contentContainerStyle={{ minWidth: '100%' }}>
            <View style={{ flex: 1, minWidth: 860 }}>
              <View style={{ flexDirection: 'row', gap: 10, paddingBottom: 6, borderBottomWidth: 1, borderColor: c.line }}>
                <Text style={[head, cell(1.1, 96)]}>{t('언제')}</Text>
                <Text style={[head, cell(1.1, 96)]}>{t('검사')}</Text>
                <Text style={[head, cell(0.8, 64)]}>{t('판정')}</Text>
                <Text style={[head, cell(3, 220)]}>{t('걸린 단계')}</Text>
                <Text style={[head, cell(0.7, 60)]}>{t('다시 돌림')}</Text>
                <Text style={[head, cell(1.4, 120)]}>{t('펌웨어')}</Text>
                <Text style={[head, cell(0.8, 64)]}>UID</Text>
                <View style={cell(1.3, 120)} />
              </View>
              {rows.map((r) => (
                <Pressable key={r.stem} onPress={() => onOpen(r)} accessibilityLabel={`${shortTime(r.startedAt)} ${verdictWord(r.verdict)}`}
                  style={({ pressed }) => ({ flexDirection: 'row', alignItems: 'center', gap: 10, paddingVertical: 8, borderBottomWidth: 1, borderColor: c.line,
                    backgroundColor: pressed ? rc('fill-information-subtlest') : 'transparent' })}>
                  <Text style={[{ color: c.text, fontSize: 13, fontFamily: fonts.mono }, cell(1.1, 96)]}>{shortTime(r.startedAt)}</Text>
                  <Text style={[{ color: c.text, fontSize: 12.5 }, cell(1.1, 96)]}>{kindLabel(r.kind, r.soakMin)}</Text>
                  <View style={cell(0.8, 64)}><VerdictChip v={r.running ? 'RUN' : r.verdict} /></View>
                  <Text style={[{ color: c.text, fontSize: 12.5 }, cell(3, 220)]} numberOfLines={2}>{r.running ? t('도는 중') : flaggedText(r, en)}</Text>
                  <Text style={[{ color: c.text, fontSize: 12.5, fontFamily: fonts.mono }, cell(0.7, 60)]}>{r.reruns ? `×${r.reruns}` : '—'}</Text>
                  <Text style={[{ color: c.muted, fontSize: 12, fontFamily: fonts.mono }, cell(1.4, 120)]} numberOfLines={1}>
                    {r.versions?.gw ? `${r.versions.gw} · ${r.versions.yaw ?? '—'}` : '—'}
                  </Text>
                  <Text style={[{ color: c.muted, fontSize: 12, fontFamily: fonts.mono }, cell(0.8, 64)]}>{r.uid ? `…${r.uid.replace(/-/g, '').slice(-4)}` : '—'}</Text>
                  <View style={[cell(1.3, 120), { flexDirection: 'row', gap: 6, justifyContent: 'flex-end' }]}>
                    {latest && r.stem === latest.stem ? <BigBtn small kind="ghost" icon="recover" label={t('단계 다시')} disabled={busy} onPress={() => onRerun(r)} /> : null}
                    {staff ? <RBIconButton variant="outlined" accessibilityLabel={t('보고서 지우기')} disabled={busy} onPress={() => onDelete(r)} icon={(p) => <Icon name="trash" size={p.size} color={p.color} />} /> : null}
                  </View>
                </Pressable>
              ))}
            </View>
          </ScrollView>
        ) : (
          <Text style={{ color: c.dim, fontSize: 13 }}>{t('이 로봇에서 한 PTZ 검사가 아직 없습니다.')}</Text>
        )}
      </Card>
      <Text style={{ color: c.dim, fontSize: 12 }}>
        {t('이 로봇의 PTZ 검사를 20건까지 남깁니다 · 지금 {n}건 · 새 검사가 생기면 가장 오래된 것부터 지웁니다').replace('{n}', String(rows.length))}
      </Text>
    </ScrollView>
  );
}
