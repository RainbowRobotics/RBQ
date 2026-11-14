import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { useRobot } from '@/store/robot';
import { commissioning } from '@/lib/commissioning';
import { ACC_CALIB } from '@/types/robot';
import { t } from '@/lib/i18n';

const G = 9.81;
const NORM_OK = 0.05;

type Phase = 'pending' | 'running' | 'done' | 'failed';

function failHint(reason: number): string {
  switch (reason) {
    case 1: return t('로봇이 서 있습니다. 앉힌 뒤 다시 실행하세요.');
    case 2: return t('IMU 가 연결되어 있지 않습니다. 연결을 확인하세요.');
    case 3: return t('다른 보정이 진행 중입니다. 끝난 뒤 다시 실행하세요.');
    case -1: return t('로봇이 응답하지 않습니다. 연결을 확인하세요.');
    default: return t('로봇을 앉힌 뒤 다시 실행하세요.');
  }
}

export function AccCalibModal({ onClose }: { onClose: () => void }) {
  const { c, radius, fonts } = useTheme();
  const { width: winW } = useWindowDimensions();
  const ip = useRobot((s) => s.ip);
  const ac = useRobot((s) => s.robot?.acc_calib);
  const startRun = useRef(useRobot.getState().robot?.acc_calib?.run ?? 0).current;
  const [timedOut, setTimedOut] = useState(false);
  useEffect(() => { commissioning.accCalibrate(ip).catch(() => {}); }, [ip]);

  const cur = ac && ac.run !== startRun ? ac : undefined;
  useEffect(() => {
    if (cur) return;
    const id = setTimeout(() => setTimedOut(true), 8000);
    return () => clearTimeout(id);
  }, [cur]);

  const phase: Phase = !cur ? (timedOut ? 'failed' : 'pending')
    : cur.state === ACC_CALIB.done ? 'done'
    : cur.state === ACC_CALIB.failed ? 'failed'
    : 'running';
  const done = phase === 'done', failed = phase === 'failed';
  const percent = done ? 100 : phase === 'pending' ? 0 : (cur?.percent ?? 0);
  const normOk = done && Math.abs((cur?.norm_after ?? 0) - G) <= NORM_OK;

  const msg = phase === 'pending' ? t('보정 명령을 보냈습니다.')
    : phase === 'running' ? t('중력 크기를 재는 중입니다.')
    : done ? t('가속도계 보정이 완료되었습니다.')
    : t('가속도계 보정을 완료하지 못하였습니다.');
  const sub = !done && !failed ? t('2초 동안 평균을 냅니다. 로봇을 건드리지 마세요.')
    : done ? t('보정계수가 로봇에 저장되었습니다.')
    : failHint(cur ? cur.reason : -1);

  const modalW = Math.min(404, winW - 32);
  const num = (v: number, digits = 2) => v.toFixed(digits);
  return (
    <Modal onClose={onClose}>
      <LinearGradient colors={[c.modalA, c.modalB]} style={[styles.modal, { borderColor: c.line, width: modalW }]}>
        <Text style={{ fontSize: 18, fontWeight: '700', color: c.text }}>{t('가속도계 보정')}</Text>

        <View style={styles.barRow}>
          <View style={[styles.track, { borderColor: c.line, backgroundColor: c.elev, borderRadius: radius.sm }]}>
            <View style={{ width: `${percent}%`, height: '100%', backgroundColor: failed ? c.redbright : c.green }} />
          </View>
          <Text style={{ fontSize: 12, color: c.muted, width: 46, textAlign: 'right', fontFamily: fonts.mono }}>{`${percent}%`}</Text>
        </View>

        <Text style={{ fontSize: 14, fontWeight: '600', textAlign: 'center',
          color: done ? (normOk ? c.greenTx : c.amberTx) : failed ? c.redTx : c.text }}>{msg}</Text>
        <Text style={{ fontSize: 12.5, color: c.muted, textAlign: 'center', lineHeight: 18 }}>{sub}</Text>

        {done && cur && (
          <View style={[styles.result, { borderColor: c.line, backgroundColor: c.elev, borderRadius: radius.sm }]}>
            <Row label={t('보정 전')} value={`${num(cur.norm_before)} m/s²`} />
            <Row label={t('보정 후')} value={`${num(cur.norm_after)} m/s²`} tone={normOk ? c.greenTx : c.amberTx} />
            <Row label={t('보정계수')} value={num(cur.ratio, 3)} />
          </View>
        )}

        <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
        </Tappable>
      </LinearGradient>
    </Modal>
  );
}

function Row({ label, value, tone }: { label: string; value: string; tone?: string }) {
  const { c, fonts } = useTheme();
  return (
    <View style={styles.resultRow}>
      <Text style={{ fontSize: 11.5, color: c.muted }}>{label}</Text>
      <Text style={{ fontSize: 11.5, color: tone ?? c.text, fontFamily: fonts.mono }}>{value}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  modal: { borderWidth: 1, borderRadius: 18, padding: 26, alignItems: 'center', gap: 12 },
  barRow: { flexDirection: 'row', alignItems: 'center', gap: 10, width: '100%' },
  track: { flex: 1, height: 12, borderWidth: 1, overflow: 'hidden' },
  result: { width: '100%', borderWidth: 1, paddingVertical: 8, paddingHorizontal: 12, gap: 5 },
  resultRow: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'center' },
  btn: { width: '100%', height: 46, borderRadius: 12, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
