import { useEffect, useRef, useState } from 'react';
import { View, Text, Image, StyleSheet, Animated, useWindowDimensions } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import Svg, { G, Line, Rect, Circle, Path } from 'react-native-svg';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { useRobot } from '@/store/robot';
import { gait } from '@/lib/gait';
import { ZMP_CALIB } from '@/types/robot';
import { t } from '@/lib/i18n';

const ROBOT = require('@/assets/images/zmp-robot.png');
const VW = 605, VH = 371;
const CX = 268, CY = 60, GY = 355, R = 17;
const OFFSET_MAX = 36;

type Phase = 'pending' | 'aligning' | 'running' | 'done' | 'failed';
const AnimatedG = Animated.createAnimatedComponent(G);

function failHint(reason: number): string {
  switch (reason) {
    case 1: return t('로봇이 서 있어야 합니다.');
    case 2: return t('로봇이 완전히 멈춘 뒤 다시 실행하세요.');
    case 3: return t('로봇을 평지에 위치시켜 주십시오.');
    case 4: return t('다리 정렬 걸음이 끝나지 않았습니다. 주변 공간을 확인하고 다시 실행하세요.');
    case 5: return t('서 있는 자세(STAND)로 바꾼 뒤 다시 실행하세요.');
    case 6: return t('60초 안에 수렴하지 못했습니다. 바닥 상태를 확인하고 다시 실행하세요.');
    case -1: return t('로봇이 응답하지 않습니다. 연결을 확인하세요.');
    default: return t('로봇을 평지에 정지 상태로 두고 다시 실행하세요.');
  }
}

export function ZmpCalibModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const { width: winW } = useWindowDimensions();
  const ip = useRobot((s) => s.ip);
  const zc = useRobot((s) => s.robot?.zmp_calib);
  const startRun = useRef(useRobot.getState().robot?.zmp_calib?.run ?? 0).current;
  const [timedOut, setTimedOut] = useState(false);
  useEffect(() => { gait.zmpCalibrate(ip).catch(() => {}); }, [ip]);

  const cur = zc && zc.run !== startRun ? zc : undefined;
  useEffect(() => {
    if (cur) return;
    const id = setTimeout(() => setTimedOut(true), 8000);
    return () => clearTimeout(id);
  }, [cur]);

  const phase: Phase = !cur ? (timedOut ? 'failed' : 'pending')
    : cur.state === ZMP_CALIB.running ? 'running'
    : cur.state === ZMP_CALIB.done ? 'done'
    : cur.state === ZMP_CALIB.failed ? 'failed'
    : 'aligning';
  const waiting = phase === 'pending' || phase === 'aligning';
  const done = phase === 'done', failed = phase === 'failed';
  const percent = done ? 100 : waiting ? 0 : (cur?.percent ?? 0);

  const onCloseRef = useRef(onClose);
  onCloseRef.current = onClose;
  useEffect(() => {
    if (!done) return;
    const id = setTimeout(() => onCloseRef.current(), 2000);
    return () => clearTimeout(id);
  }, [done]);

  const shift = useRef(new Animated.Value(OFFSET_MAX)).current;
  useEffect(() => {
    Animated.timing(shift, { toValue: OFFSET_MAX * (1 - percent / 100), duration: 600, useNativeDriver: false }).start();
  }, [percent, shift]);
  const pulse = useRef(new Animated.Value(0.35)).current;
  useEffect(() => {
    if (!waiting) return;
    const loop = Animated.loop(Animated.sequence([
      Animated.timing(pulse, { toValue: 0.9, duration: 700, useNativeDriver: false }),
      Animated.timing(pulse, { toValue: 0.35, duration: 700, useNativeDriver: false }),
    ]));
    loop.start();
    return () => loop.stop();
  }, [waiting, pulse]);

  const modalW = Math.min(404, winW - 32);
  const figW = modalW - 26 * 2 - 8 * 2, figH = figW * VH / VW;
  const msg = waiting ? t('자세를 정렬하는 중입니다.')
    : phase === 'running' ? t('무게중심 보정중입니다.')
    : done ? t('무게중심 보정이 완료되었습니다.') : t('무게중심 보정을 완료하지 못하였습니다.');
  const sub = waiting ? t('한 걸음 천천히 걸어 다리를 가지런히 놓습니다. 끝나면 자동으로 보정이 시작됩니다.')
    : phase === 'running' ? t('수렴하면 로봇이 스스로 저장하고 STAND 로 돌아옵니다. 그동안 로봇을 건드리지 마세요.')
    : done ? t('오프셋이 로봇에 저장되었습니다.')
    : `${failHint(cur ? cur.reason : -1)} ${t('이전 보정값이 그대로 유지됩니다.')}`;
  const cap = done ? t('2초 후 자동으로 닫힙니다') : failed ? t('사유는 시스템 로그에 남습니다')
    : phase === 'running' ? t('최대 60초') : t('닫아도 로봇의 보정은 계속됩니다');

  return (
    <Modal onClose={onClose}>
      <LinearGradient colors={[c.modalA, c.modalB]} style={[styles.modal, { borderColor: c.line, width: modalW }]}>
        <Text style={{ fontSize: 18, fontWeight: '700', color: c.text }}>{t('무게 중심 보정')}</Text>

        <View style={[styles.plate, { backgroundColor: c.elev, borderRadius: radius.md }]}>
          <Image source={ROBOT} style={{ width: figW, height: figH }} resizeMode="contain" />
          <Svg viewBox={`0 0 ${VW} ${VH}`} width={figW} height={figH} style={styles.overlay}>
            <Line x1={18} y1={GY} x2={588} y2={GY} stroke={c.line} strokeWidth={3} />
            <Rect x={CX - 18} y={GY - 7} width={36} height={14} rx={4} fill={c.green} opacity={0.3} />
            <AnimatedG translateX={shift}>
              <Line x1={CX} y1={CY + R + 4} x2={CX} y2={GY} stroke={c.accent2} strokeWidth={2} strokeDasharray="4 4" />
              <Circle cx={CX} cy={CY} r={R} fill={c.panel} stroke={c.text} strokeWidth={2} />
              <Path d={`M${CX} ${CY} L${CX} ${CY - R} A${R} ${R} 0 0 1 ${CX + R} ${CY} Z`} fill={c.text} />
              <Path d={`M${CX} ${CY} L${CX} ${CY + R} A${R} ${R} 0 0 1 ${CX - R} ${CY} Z`} fill={c.text} />
              <Circle cx={CX} cy={GY} r={R * 0.38} fill={c.accent2} />
            </AnimatedG>
            {done && <Circle cx={CX} cy={GY} r={R * 0.62} fill="none" stroke={c.green} strokeWidth={3} />}
          </Svg>
        </View>

        <Text style={{ fontSize: 14, fontWeight: '600', color: done ? c.greenTx : failed ? c.redTx : c.text, textAlign: 'center' }}>{msg}</Text>
        <Text style={{ fontSize: 12.5, color: c.muted, textAlign: 'center', lineHeight: 18 }}>{sub}</Text>

        <View style={styles.barRow}>
          <View style={[styles.track, { borderColor: c.line, backgroundColor: c.elev, borderRadius: radius.sm }]}>
            {waiting
              ? <Animated.View style={{ width: '100%', height: '100%', backgroundColor: c.accent2, opacity: pulse }} />
              : <View style={{ width: `${percent}%`, height: '100%', backgroundColor: failed ? c.redbright : c.green }} />}
          </View>
          <Text style={{ fontSize: 11, color: c.muted, width: 44, textAlign: 'right' }}>{waiting ? '—' : `${percent}%`}</Text>
        </View>

        <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
        </Tappable>
        <Text style={{ fontSize: 10.5, color: c.dim, textAlign: 'center' }}>{cap}</Text>
      </LinearGradient>
    </Modal>
  );
}

const styles = StyleSheet.create({
  modal: { borderWidth: 1, borderRadius: 18, padding: 26, alignItems: 'center', gap: 12 },
  plate: { padding: 8 },
  overlay: { position: 'absolute', left: 8, top: 8 },
  barRow: { flexDirection: 'row', alignItems: 'center', gap: 10, width: '100%' },
  track: { flex: 1, height: 10, borderWidth: 1, overflow: 'hidden' },
  btn: { width: '100%', height: 46, borderRadius: 12, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
