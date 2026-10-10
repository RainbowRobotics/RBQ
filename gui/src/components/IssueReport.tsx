import { useState } from 'react';
import { View, Text, TextInput, Image, StyleSheet, Platform } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { useSettings } from '@/store/settings';
import { useRobot } from '@/store/robot';
import { useReviewServer, sendIssueReport, buildMeta } from '@/lib/issueReport';
import { t } from '@/lib/i18n';

async function captureB64(): Promise<string | null> {
  if (Platform.OS === 'web') return null;
  try {
    const { captureScreen } = await import('react-native-view-shot');
    return await captureScreen({ format: 'png', result: 'base64', quality: 0.9 });
  } catch { return null; }
}

export function IssueReportButton() {
  const { c, radius } = useTheme();
  const level3 = useSettings((s) => s.accessLevel >= 3);
  const serverOk = useReviewServer(level3);
  const robotIp = useRobot((s) => s.ip);
  const [open, setOpen] = useState(false);
  const [shot, setShot] = useState<string | null>(null);
  const [comment, setComment] = useState('');
  const [busy, setBusy] = useState(false);
  const [msg, setMsg] = useState<string | null>(null);

  if (!level3 || !serverOk) return null;

  const begin = async () => {
    setShot(await captureB64());
    setComment(''); setMsg(null); setOpen(true);
  };
  const send = () => {
    if (!comment.trim() || busy) return;
    setBusy(true); setMsg(null);
    sendIssueReport({ comment: comment.trim(), meta: buildMeta(robotIp), shotB64: shot })
      .then(() => { setMsg(t('전송됨 — 대시보드 앱 리포트 큐에서 확인됩니다')); setComment(''); })
      .catch((e: Error) => setMsg(t('전송 실패: ') + e.message))
      .finally(() => setBusy(false));
  };

  return (
    <>
      <Tappable onPress={begin} style={[styles.btn, { borderColor: c.glassLine, backgroundColor: 'transparent', borderRadius: radius.md }]}>
        <Icon name="warn" size={14} color={c.amberTx} />
      </Tappable>
      {open && (
        <Modal onClose={() => setOpen(false)}>
          <View style={{ backgroundColor: c.panel, borderColor: c.line, borderWidth: 1, borderRadius: radius.lg, padding: 16 }}>
          <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', marginBottom: 6 }}>{t('이슈 리포팅')}</Text>
          <Text style={{ color: c.dim, fontSize: 10.5, marginBottom: 10 }}>
            {t('내부망 대시보드 큐로 전송됩니다 — 검토 후 GitHub 이슈로 등록돼요')}
          </Text>
          {shot && (
            <Image source={{ uri: `data:image/png;base64,${shot}` }}
              style={{ width: 280, height: 175, borderRadius: 8, marginBottom: 10, borderWidth: 1, borderColor: c.line }}
              resizeMode="contain" />
          )}
          <TextInput
            value={comment} onChangeText={setComment} multiline
            placeholder={t('무엇이 문제인가요? (첫 줄이 이슈 제목이 됩니다)')} placeholderTextColor={c.dim}
            style={{ minHeight: 72, width: 280, color: c.text, backgroundColor: c.elev, borderColor: c.line, borderWidth: 1, borderRadius: 8, padding: 8, fontSize: 12, textAlignVertical: 'top' }} />
          {msg && <Text style={{ color: msg.startsWith(t('전송됨')) ? c.greenTx : c.redTx, fontSize: 10.5, marginTop: 8 }}>{msg}</Text>}
          <View style={{ flexDirection: 'row', gap: 8, marginTop: 12, justifyContent: 'flex-end' }}>
            <Tappable onPress={() => setOpen(false)} style={[styles.act, { borderColor: c.line, backgroundColor: c.elev }]}>
              <Text style={{ color: c.muted, fontSize: 11 }}>{t('닫기')}</Text>
            </Tappable>
            <Tappable onPress={send} disabled={!comment.trim() || busy}
              style={[styles.act, { borderColor: 'rgba(77,156,245,0.5)', backgroundColor: 'rgba(77,156,245,0.14)', opacity: comment.trim() && !busy ? 1 : 0.4 }]}>
              <Text style={{ color: c.accent2, fontSize: 11, fontWeight: '600' }}>{busy ? t('전송 중…') : t('보내기')}</Text>
            </Tappable>
          </View>
          </View>
        </Modal>
      )}
    </>
  );
}

const styles = StyleSheet.create({
  btn: { width: 30, height: 30, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  act: { paddingHorizontal: 14, paddingVertical: 7, borderRadius: 9, borderWidth: 1 },
});
