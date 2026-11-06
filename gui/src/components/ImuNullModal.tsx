import { useEffect, useState } from 'react';
import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { ImuRollPitch } from '@/components/BubbleLevel';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { commissioning } from '@/lib/commissioning';
import { t } from '@/lib/i18n';

const NULL_MS = 1200;

export function ImuNullModal({ onClose }: { onClose: () => void }) {
  const { c, radius, fonts } = useTheme();
  const { width: winW } = useWindowDimensions();
  const ip = useRobot((s) => s.ip);
  const [ackAt, setAckAt] = useState<number | null>(null);
  const [failed, setFailed] = useState(false);
  const [now, setNow] = useState(Date.now());
  useEffect(() => {
    commissioning.imuNull(ip).then(() => setAckAt(Date.now())).catch(() => setFailed(true));
  }, [ip]);
  const elapsed = ackAt == null ? 0 : now - ackAt;
  const done = ackAt != null && elapsed >= NULL_MS;
  useEffect(() => {
    if (ackAt == null || done) return;
    const id = setInterval(() => setNow(Date.now()), 50);
    return () => clearInterval(id);
  }, [ackAt, done]);

  const percent = done ? 100 : Math.min(99, Math.round((elapsed / NULL_MS) * 100));
  const rpy = useTelemetry((s) => s.robot?.imu?.rpy);
  const deg = (v: number | undefined) => Math.abs(((v ?? 0) * 180) / Math.PI);
  const level = done && deg(rpy?.[0]) <= 1 && deg(rpy?.[1]) <= 1;
  const msg = failed ? t('IMU 영점을 잡지 못했습니다.')
    : done ? (level ? t('성공') : t('영점이 반영되지 않았을 수 있습니다'))
    : t('영점을 잡는 중입니다.');
  const sub = failed ? t('로봇이 명령을 받지 않았습니다. 연결과 제어권을 확인한 뒤 다시 실행하세요.')
    : done ? (level ? t('지금 자세가 수평 기준으로 저장되었습니다.') : t('명령은 보냈지만 롤·피치가 아직 0 이 아닙니다. 로봇을 평지에 두고 다시 실행하세요.'))
    : t('로봇을 건드리지 마세요.');

  return (
    <Modal onClose={done || failed ? onClose : () => {}} dismissable={done || failed}>
      <LinearGradient colors={[c.modalA, c.modalB]} style={[styles.modal, { borderColor: c.line, width: Math.min(404, winW - 32) }]}>
        <Text style={{ fontSize: 18, fontWeight: '700', color: c.text }}>{t('IMU 롤/피치 영점')}</Text>

        <View style={styles.barRow}>
          <View style={[styles.track, { borderColor: c.line, backgroundColor: c.elev, borderRadius: radius.sm }]}>
            <View style={{ width: `${failed ? 100 : percent}%`, height: '100%', backgroundColor: failed ? c.redbright : c.green }} />
          </View>
          <Text style={{ fontSize: 12, color: c.muted, width: 46, textAlign: 'right', fontFamily: fonts.mono }}>{failed ? '—' : `${percent}%`}</Text>
        </View>

        <Text style={{ fontSize: done ? 20 : 14, fontWeight: done ? '800' : '600', textAlign: 'center',
          color: level ? c.greenTx : done || failed ? c.amberTx : c.text }}>{msg}</Text>
        <Text style={{ fontSize: 12.5, color: c.muted, textAlign: 'center', lineHeight: 18 }}>{sub}</Text>

        <ImuRollPitch />

        {(done || failed) && (
          <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
          </Tappable>
        )}
      </LinearGradient>
    </Modal>
  );
}

const styles = StyleSheet.create({
  modal: { borderWidth: 1, borderRadius: 18, padding: 26, alignItems: 'center', gap: 12 },
  barRow: { flexDirection: 'row', alignItems: 'center', gap: 10, width: '100%' },
  track: { flex: 1, height: 12, borderWidth: 1, overflow: 'hidden' },
  btn: { width: '100%', height: 46, borderRadius: 12, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
