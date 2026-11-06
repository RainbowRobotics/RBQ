import { useState } from 'react';
import { Text } from 'react-native';
import { RBLabelButton } from '@/rb/components/RBLabelButton';
import { QcDialog } from '@/components/ptzqc/dialogs';
import { Note } from '@/components/ptzqc/parts';
import { EditField } from '@/components/panels/settings/common';
import { useTheme } from '@/theme';
import { actions } from '@/lib/rest';
import { t } from '@/lib/i18n';

const MAX_LEN = 64;

export function RobotSerialDialog({ ip, current, onSaved, onClose }: {
  ip: string; current: string; onSaved: (s: string) => void; onClose: () => void;
}) {
  const { c } = useTheme();
  const [v, setV] = useState(current);
  const [busy, setBusy] = useState(false);
  const [err, setErr] = useState('');
  const val = v.trim();
  const ok = val.length > 0 && val.length <= MAX_LEN;
  const save = () => {
    if (!ok || val === current || busy) return;
    setBusy(true);
    setErr('');
    actions.setSerialNumber(ip, val)
      .then(() => { onSaved(val); onClose(); })
      .catch((e: unknown) => { setErr(e instanceof Error ? e.message : String(e)); setBusy(false); });
  };
  return (
    <QcDialog title={t('로봇 S/N 쓰기')} onClose={onClose} width={460}
      footer={<>
        <RBLabelButton variant="outlined" onPress={onClose}>{t('취소')}</RBLabelButton>
        <RBLabelButton variant="brand" disabled={!ok || val === current || busy} onPress={save}>{t('쓰기')}</RBLabelButton>
      </>}>
      <EditField label={t('로봇 S/N')} value={v} onChangeText={(s) => { setV(s); setErr(''); }} onSubmit={save} />
      <Text style={{ color: ok || !val ? c.dim : c.redTx, fontSize: 12, marginTop: -8 }}>{t('1~64자')}</Text>
      <Note tone="warn" text={t('앱은 S/N 으로 로봇을 구분합니다 — 바꾸면 저장된 비밀번호·원격 등록이 새 S/N 기준으로 다시 필요할 수 있습니다.')} />
      {!!err && <Text style={{ color: c.redTx, fontSize: 12 }}>{err}</Text>}
    </QcDialog>
  );
}
