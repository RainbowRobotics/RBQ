import { useMemo, useState } from 'react';
import { View, Text, Pressable, StyleSheet, ScrollView, Modal } from 'react-native';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { useRouter } from 'expo-router';
import { useTheme } from '@/theme';
import { useRobot, getLiveAlerts } from '@/store/robot';
import { useLogAck } from '@/store/logAck';
import type { LogLevel, LogLine } from '@/types/robot';
import { t } from '@/lib/i18n';

const RED: LogLevel[] = ['ERROR', 'FATAL'];

export function LogBadge() {
  const { c } = useTheme();
  const router = useRouter();
  const alertSeq = useRobot((s) => s.alertSeq);
  const [open, setOpen] = useState(false);
  const ackLen = useLogAck((s) => s.ackLen);
  const setAckLen = useLogAck((s) => s.setAck);
  const [view, setView] = useState<LogLine[]>([]);

  // eslint-disable-next-line react-hooks/exhaustive-deps
  const alerts = useMemo(() => [...getLiveAlerts()], [alertSeq]);
  const safeAck = Math.min(ackLen, alerts.length);
  const newAlerts = useMemo(() => alerts.slice(safeAck), [alerts, safeAck]);
  const count = newAlerts.length;
  const hasRed = useMemo(() => newAlerts.some((l) => RED.includes(l.level)), [newAlerts]);
  const tone = count === 0 ? c.dim : hasRed ? c.red : c.amber;
  const disp = count > 99 ? '99+' : String(count);

  const onTap = () => {
    if (!open) {
      setView(newAlerts);
      setAckLen(alerts.length);
    }
    setOpen((v) => !v);
  };

  const jump = (l: LogLine) => {
    setOpen(false);
    router.push({ pathname: '/log', params: { focusTs: l.ts, focusMsg: l.msg } });
  };

  return (
    <View>
      <Pressable
        onPress={onTap}
        style={[styles.badge, { borderColor: tone, backgroundColor: count === 0 ? 'transparent' : tone + '22' }]}
      >
        <Text style={{ color: tone, fontSize: 11, fontWeight: '700' }}>{disp}</Text>
      </Pressable>
      <Modal supportedOrientations={MODAL_ORIENTATIONS} visible={open} transparent animationType="fade" onRequestClose={() => setOpen(false)}>
        <Pressable style={styles.scrim} onPress={() => setOpen(false)}>
          <Pressable style={[styles.panel, { backgroundColor: c.panel, borderColor: c.line }]} onPress={() => {}}>
            <View style={[styles.head, { borderBottomColor: c.line }]}>
              <Text style={{ color: c.muted, fontSize: 10, fontWeight: '700' }}>{t('새 경고')} {view.length ? `${view.length}${t('건')}` : ''}</Text>
              <Text style={{ color: c.dim, fontSize: 9 }}>{t('탭하면 로그 탭에서 해당 위치로 이동')}</Text>
            </View>
            <ScrollView style={{ maxHeight: 340 }} persistentScrollbar showsVerticalScrollIndicator>
              {view.length === 0 ? (
                <Text style={{ color: c.dim, fontSize: 11, padding: 12 }}>{t('새 경고 없음')}</Text>
              ) : (
                view
                  .slice()
                  .reverse()
                  .map((l, i) => (
                    <Pressable key={i} onPress={() => jump(l)} style={styles.row}>
                      <View style={[styles.lv, { backgroundColor: RED.includes(l.level) ? c.red : c.amber }]} />
                      <Text style={{ color: c.dim, fontSize: 10 }}>{l.ts}</Text>
                      <Text numberOfLines={1} style={{ color: c.text, fontSize: 11, flex: 1 }}>
                        {l.msg}
                      </Text>
                    </Pressable>
                  ))
              )}
            </ScrollView>
          </Pressable>
        </Pressable>
      </Modal>
    </View>
  );
}

const styles = StyleSheet.create({
  badge: { minWidth: 26, height: 22, borderRadius: 11, borderWidth: 1, alignItems: 'center', justifyContent: 'center', paddingHorizontal: 6 },
  scrim: { flex: 1, backgroundColor: 'rgba(0,0,0,0.25)' },
  panel: { position: 'absolute', top: 58, right: 12, width: 400, maxWidth: '80%', borderWidth: 1, borderRadius: 10, overflow: 'hidden' },
  head: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center', paddingHorizontal: 10, paddingVertical: 7, borderBottomWidth: 1 },
  row: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 10, paddingVertical: 7 },
  lv: { width: 6, height: 6, borderRadius: 3 },
});
