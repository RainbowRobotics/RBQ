import { useEffect, useState } from 'react';
import { Modal, View, Text, Pressable, StyleSheet } from 'react-native';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { useRobot } from '@/store/robot';
import { connection } from '@/lib/connection';
import { useTheme } from '@/theme';
import { t } from '@/lib/i18n';

export function OwnershipGuard() {
  const conn = useRobot((s) => s.conn);
  const owner = useRobot((s) => s.owner);
  const myIp = useRobot((s) => s.myIp);
  const { c, fonts } = useTheme();

  const lost = conn === 'connected' && !!owner && !!myIp && owner !== myIp;
  const [dismissed, setDismissed] = useState(false);
  const [busy, setBusy] = useState(false);
  const [failed, setFailed] = useState(false);

  useEffect(() => {
    if (!lost) { setDismissed(false); setFailed(false); }
  }, [lost]);

  const reclaim = async () => {
    setBusy(true);
    setFailed(false);
    const ok = await connection.claimOwnership();
    setBusy(false);
    if (!ok) setFailed(true);
  };

  return (
    <Modal supportedOrientations={MODAL_ORIENTATIONS} transparent visible={lost && !dismissed} animationType="fade" onRequestClose={() => setDismissed(true)}>
      <View style={styles.scrim}>
        <View style={[styles.panel, { backgroundColor: c.panel, borderColor: c.amber }]}>
          <Text style={[styles.title, { color: c.amber }]}>{t('조종 권한이 넘어갔습니다')}</Text>
          <Text style={[styles.body, { color: c.text }]}>
            {t('다른 기기가 로봇 제어권을 가져갔습니다.')}{'\n'}
            {t('조이스틱·모션 명령이 차단된 상태입니다.')} {t('영상·상태 표시는 유지됩니다.')}
          </Text>
          <Text style={[styles.ipLine, { color: c.dim, fontFamily: fonts.mono }]}>
            {t('현재 소유:')} {owner}   {t('내 IP:')} {myIp}
          </Text>
          {failed && (
            <Text style={[styles.fail, { color: c.redbright }]}>{t('제어권 회수 실패 — 다시 시도하세요')}</Text>
          )}
          <View style={styles.btnRow}>
            <Pressable
              onPress={() => setDismissed(true)}
              style={({ pressed }) => [styles.btn, { borderColor: c.muted, opacity: pressed ? 0.6 : 1 }]}
            >
              <Text style={[styles.btnText, { color: c.dim }]}>{t('보기만')}</Text>
            </Pressable>
            <Pressable
              onPress={reclaim}
              disabled={busy}
              style={({ pressed }) => [
                styles.btn, styles.btnPrimary,
                { backgroundColor: c.amber, opacity: busy ? 0.5 : pressed ? 0.75 : 1 },
              ]}
            >
              <Text style={[styles.btnText, { color: '#000' }]}>{busy ? t('요청 중…') : t('제어권 되찾기')}</Text>
            </Pressable>
          </View>
        </View>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  scrim: { flex: 1, backgroundColor: 'rgba(0,0,0,0.55)', alignItems: 'center', justifyContent: 'center' },
  panel: { width: 340, borderRadius: 14, borderWidth: 1, padding: 20, gap: 10 },
  title: { fontSize: 16, fontWeight: '800', letterSpacing: 0.5 },
  body: { fontSize: 13, lineHeight: 19 },
  ipLine: { fontSize: 11 },
  fail: { fontSize: 12, fontWeight: '600' },
  btnRow: { flexDirection: 'row', gap: 10, marginTop: 6, justifyContent: 'flex-end' },
  btn: { borderWidth: 1, borderRadius: 9, paddingVertical: 9, paddingHorizontal: 16, borderColor: 'transparent' },
  btnPrimary: {},
  btnText: { fontSize: 13, fontWeight: '700' },
});
