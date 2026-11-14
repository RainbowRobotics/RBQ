import { useEffect, useState } from 'react';
import { Text } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { TRow, useActBtn } from '@/components/panels/PowerPanel';
import { useRobot } from '@/store/robot';
import { actions } from '@/lib/rest';
import { webrtcClient } from '@/lib/webrtcClient';
import { t } from '@/lib/i18n';
import { notify } from '@/components/DoneToast';

export function VisionResetRow() {
  const { c, radius } = useTheme();
  const btn = useActBtn();
  const ip = useRobot((s) => s.ip);
  useEffect(() => { if (ip) webrtcClient.ensureConnected(ip); }, [ip]);
  const [resetting, setResetting] = useState(false);
  const [err, setErr] = useState('');

  const doReset = async () => {
    setErr(''); setResetting(true);
    try { await actions.visionReset(ip); notify(t('비전 리셋을 요청했습니다 — 잠시 영상이 끊깁니다')); }
    catch (e: any) { setErr(String(e?.message ?? e)); }
    setTimeout(() => setResetting(false), 8000);
  };

  return (
    <>
      <TRow nm="VISION RESET" sub={t('Handler가 비전 데몬(HAL·Streamer 등)을 재시작 — 카메라 전원도 껐다 켬 (그동안 영상 끊김)')}
        right={
          <Tappable disabled={resetting} onPress={doReset}
            style={[btn.box, { flexDirection: 'row', gap: 6, borderColor: 'rgba(210,153,34,0.5)', backgroundColor: 'rgba(210,153,34,0.10)', borderRadius: radius.sm, opacity: resetting ? 0.5 : 1 }]}>
            <Icon name="recover" size={btn.icon} color={c.amber} />
            <Text style={{ color: c.amber, fontSize: btn.font, fontWeight: btn.weight }}>{resetting ? t('리셋 중…') : t('리셋')}</Text>
          </Tappable>
        } />
      {err ? <Text style={{ color: c.redbright, fontSize: 11, marginTop: 6 }}>{err}</Text> : null}
    </>
  );
}

