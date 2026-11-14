import { useEffect, useState } from 'react';
import { View, Text } from 'react-native';
import { useTheme } from '@/theme';
import { useRobot } from '@/store/robot';
import { actions, rest } from '@/lib/rest';
import { H2, Desc, EditField, SBtn, S, useLegible, Group } from './common';
import { t } from '@/lib/i18n';

export function DynamicsPanel() {
  const ip = useRobot((s) => s.ip);
  return <DockCalib ip={ip} />;
}

const DOCK_FACTORY = { offset_x: '0.133', offset_y: '0', count_req: '300', count_try: '10' };

function DockCalib({ ip }: { ip: string }) {
  const { c } = useTheme();
  const lg = useLegible();
  const [f, setF] = useState({ offset_x: '', offset_y: '', count_req: '', count_try: '' });
  const [msg, setMsg] = useState('');
  const [msgOk, setMsgOk] = useState(false);
  const load = () => rest.getDockParams(ip)
    .then((d) => { setF({ offset_x: String(d.offset_x), offset_y: String(d.offset_y), count_req: String(d.count_req), count_try: String(d.count_try) }); setMsg(''); })
    .catch(() => { setMsg(t('읽기 실패 — 로봇 연결 확인')); setMsgOk(false); });
  useEffect(() => { load(); }, [ip]); // eslint-disable-line react-hooks/exhaustive-deps
  const set = (k: keyof typeof f) => (v: string) => setF((p) => ({ ...p, [k]: v }));
  const apply = async () => {
    try {
      await actions.setDockParams(ip, {
        offset_x: parseFloat(f.offset_x) || 0, offset_y: parseFloat(f.offset_y) || 0,
        count_req: parseInt(f.count_req, 10) || 0, count_try: parseInt(f.count_try, 10) || 0,
      });
      await load();
      setMsg(t('적용됨 (로봇 값으로 재확인)')); setMsgOk(true);
    } catch { setMsg(t('적용 실패')); setMsgOk(false); }
  };
  return (
    <>
      <H2>{t('도킹 캘리브레이션')}</H2>
      <Desc>{t('자동 도킹 정렬 파라미터 (dock/parameters — 도킹 gait가 사용)')}</Desc>
      <Group>
      <View style={lg ? { paddingTop: 12, paddingBottom: 14 } : undefined}>
      <View style={S.row3}>
        <EditField label="Offset X (m)" value={f.offset_x} onChangeText={set('offset_x')} keyboardType="numbers-and-punctuation" />
        <EditField label="Offset Y (m)" value={f.offset_y} onChangeText={set('offset_y')} keyboardType="numbers-and-punctuation" />
      </View>
      <View style={S.row3}>
        <EditField label="Req Count" value={f.count_req} onChangeText={set('count_req')} keyboardType="decimal-pad" />
        <EditField label="Try Count" value={f.count_try} onChangeText={set('count_try')} keyboardType="decimal-pad" />
      </View>
      <View style={S.actions}>
        <SBtn kind="ghost" icon="download" label={t('다시 읽기')} onPress={load} />
        <SBtn kind="ghost" icon="recover" label={t('기본값')} onPress={() => { setF(DOCK_FACTORY); setMsg(t('출고 기본값을 넣었습니다 — [적용]을 눌러야 로봇에 저장됩니다')); setMsgOk(true); }} />
        <SBtn kind="primary" icon="save" label={t('적용')} onPress={apply} />
      </View>
      {msg ? <Text style={{ color: msgOk ? c.greenTx : c.amber, fontSize: lg ? 12 : 11, marginTop: 10 }}>{msg}</Text> : null}
      </View>
      </Group>
    </>
  );
}
