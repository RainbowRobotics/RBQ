import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, useWindowDimensions } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { commissioning } from '@/lib/commissioning';
import { SitRobotHint } from '@/components/SitRobotHint';
import { GYRO_CALIB } from '@/types/robot';
import { t } from '@/lib/i18n';

const LIMIT_DPS = 1.0;
const RANGE_DPS = 3.0;
const R2D = 180 / Math.PI;
const LPF_ALPHA = 0.2;

type Phase = 'pending' | 'resetting' | 'done' | 'failed';

function useGyroDps(): number[] {
  const raw = useTelemetry((s) => s.robot?.imu.gyro);
  const [v, setV] = useState([0, 0, 0]);
  useEffect(() => {
    if (!raw) return;
    setV((p) => p.map((x, i) => x + (raw[i] * R2D - x) * LPF_ALPHA));
  }, [raw]);
  return v;
}

function failHint(reason: number): string {
  switch (reason) {
    case 1: return t('로봇이 서 있습니다. 앉힌 뒤 다시 실행하세요.');
    case 2: return t('IMU 가 연결되어 있지 않습니다. 연결을 확인하세요.');
    case 3: return t('다른 보정이 진행 중입니다. 끝난 뒤 다시 실행하세요.');
    case -1: return t('로봇이 응답하지 않습니다. 연결을 확인하세요.');
    default: return t('로봇을 앉힌 뒤 다시 실행하세요.');
  }
}

function AxisBar({ label, value, limit }: { label: string; value: number; limit: number }) {
  const { c, fonts, radius } = useTheme();
  const clamped = Math.max(-RANGE_DPS, Math.min(RANGE_DPS, value));
  const half = Math.abs(clamped) / RANGE_DPS * 50;
  const tick = limit / RANGE_DPS * 50;
  const ok = Math.abs(value) <= limit;
  return (
    <View style={styles.axisRow}>
      <Text style={{ width: 14, fontSize: 12, fontWeight: '700', color: c.muted }}>{label}</Text>
      <View style={[styles.track, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
        <View style={[styles.fill, { backgroundColor: ok ? c.green : c.redbright,
          left: `${clamped < 0 ? 50 - half : 50}%`, width: `${half}%` }]} />
        <View style={[styles.tick, { left: `${50 - tick}%`, backgroundColor: c.amber }]} />
        <View style={[styles.tick, { left: `${50 + tick}%`, backgroundColor: c.amber }]} />
        <View style={[styles.zero, { backgroundColor: c.text }]} />
      </View>
      <Text style={{ width: 68, fontSize: 11, fontFamily: fonts.mono, textAlign: 'right', color: ok ? c.greenTx : c.redTx }}>
        {`${value >= 0 ? '+' : ''}${value.toFixed(2)}`}
      </Text>
    </View>
  );
}

function GyroBars({ limit }: { limit: number }) {
  const { c } = useTheme();
  const dps = useGyroDps();
  return (
    <View style={styles.bars}>
      {['X', 'Y', 'Z'].map((ax, i) => <AxisBar key={ax} label={ax} value={dps[i]} limit={limit} />)}
      <Text style={{ fontSize: 10, color: c.dim, textAlign: 'center' }}>
        {t('허용치 ±{n}°/s').replace('{n}', limit.toFixed(1))} · {t('눈금 ±{n}°/s').replace('{n}', RANGE_DPS.toFixed(0))}
      </Text>
    </View>
  );
}

export function GyroCalibHint() {
  return <SitRobotHint><GyroBars limit={LIMIT_DPS} /></SitRobotHint>;
}

export function GyroBiasModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const { width: winW } = useWindowDimensions();
  const ip = useRobot((s) => s.ip);
  const gc = useRobot((s) => s.robot?.gyro_calib);
  const startRun = useRef(useRobot.getState().robot?.gyro_calib?.run ?? 0).current;
  const [timedOut, setTimedOut] = useState(false);
  const [percent, setPercent] = useState(0);
  const t0 = useRef<number | null>(null);
  useEffect(() => { commissioning.gyroBiasSet(ip).catch(() => {}); }, [ip]);

  const cur = gc && gc.run !== startRun ? gc : undefined;
  useEffect(() => {
    if (cur) return;
    const id = setTimeout(() => setTimedOut(true), 8000);
    return () => clearTimeout(id);
  }, [cur]);

  const phase: Phase = !cur ? (timedOut ? 'failed' : 'pending')
    : cur.state === GYRO_CALIB.done ? 'done'
    : cur.state === GYRO_CALIB.failed ? 'failed'
    : 'resetting';

  useEffect(() => {
    if (phase !== 'resetting' || !cur) return;
    if (t0.current == null) t0.current = Date.now() - cur.elapsed_ms;
    const start = t0.current, total = Math.max(1, cur.total_ms);
    const id = setInterval(() => setPercent(Math.min(99, Math.floor((Date.now() - start) / total * 100))), 100);
    return () => clearInterval(id);
  }, [phase, cur]);

  const limit = cur?.limit_dps ?? LIMIT_DPS;
  const done = phase === 'done', failed = phase === 'failed';
  const pass = done && !!cur?.pass;
  const shown = done ? 100 : phase === 'pending' ? 0 : percent;

  const msg = phase === 'pending' ? t('리셋 명령을 보냈습니다.')
    : phase === 'resetting' ? t('IMU 를 리셋하고 값이 자리 잡기를 기다리는 중입니다.')
    : pass ? t('자이로 바이어스 보정이 완료되었습니다.')
    : done ? t('리셋 후에도 자이로 편차가 허용치를 넘습니다.')
    : t('자이로 바이어스 보정을 완료하지 못하였습니다.');
  const sub = !done && !failed ? t('로봇을 건드리지 마세요. 끝나면 리셋 후의 자이로 값을 보여줍니다.')
    : pass ? t('세 축 모두 허용치 이내입니다.')
    : done ? t('로봇을 완전히 멈춘 뒤 다시 실행하세요. 반복되면 IMU 점검이 필요합니다.')
    : failHint(cur ? cur.reason : -1);

  const modalW = Math.min(404, winW - 32);
  return (
    <Modal onClose={onClose}>
      <LinearGradient colors={[c.modalA, c.modalB]} style={[styles.modal, { borderColor: c.line, width: modalW }]}>
        <Text style={{ fontSize: 18, fontWeight: '700', color: c.text }}>{t('자이로 바이어스 보정')}</Text>

        <GyroBars limit={limit} />

        <Text style={{ fontSize: 14, fontWeight: '600', textAlign: 'center',
          color: pass ? c.greenTx : (done || failed) ? c.redTx : c.text }}>{msg}</Text>
        <Text style={{ fontSize: 12.5, color: c.muted, textAlign: 'center', lineHeight: 18 }}>{sub}</Text>
        {done && cur && (
          <Text style={{ fontSize: 10.5, color: c.dim, textAlign: 'center' }}>
            {t('리셋 후 x / y / z')} {cur.bias_dps.map((x) => `${x >= 0 ? '+' : ''}${x.toFixed(2)}`).join(' / ')} °/s
          </Text>
        )}

        {!failed && (
          <View style={styles.barRow}>
            <View style={[styles.pTrack, { borderColor: c.line, backgroundColor: c.elev, borderRadius: radius.sm }]}>
              <View style={{ width: `${shown}%`, height: '100%', backgroundColor: done && !pass ? c.redbright : c.green }} />
            </View>
            <Text style={{ fontSize: 11, color: c.muted, width: 44, textAlign: 'right' }}>{`${shown}%`}</Text>
          </View>
        )}

        <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
        </Tappable>
      </LinearGradient>
    </Modal>
  );
}

const styles = StyleSheet.create({
  modal: { borderWidth: 1, borderRadius: 18, padding: 26, alignItems: 'center', gap: 12 },
  bars: { width: '100%', gap: 8, paddingVertical: 4 },
  axisRow: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  track: { flex: 1, height: 16, borderWidth: 1, overflow: 'hidden' },
  fill: { position: 'absolute', top: 0, bottom: 0 },
  tick: { position: 'absolute', top: 0, bottom: 0, width: 2, opacity: 0.85 },
  zero: { position: 'absolute', top: 0, bottom: 0, left: '50%', width: 2 },
  barRow: { flexDirection: 'row', alignItems: 'center', gap: 10, width: '100%' },
  pTrack: { flex: 1, height: 10, borderWidth: 1, overflow: 'hidden' },
  btn: { width: '100%', height: 46, borderRadius: 12, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
