import { useRef, useState } from 'react';
import { View, Text, StyleSheet, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { RobotModel3D } from '@/components/RobotModel3D';
import { useTelemetry } from '@/store/telemetry';
import { recovery } from '@/lib/recovery';
import { connection } from '@/lib/connection';
import { t } from '@/lib/i18n';

const JOINT_NAMES = ['Roll', 'Hip', 'Knee'];
const LEGS: { title: string; base: number }[] = [
  { title: 'FL · 앞왼쪽', base: 0 },
  { title: 'FR · 앞오른쪽', base: 3 },
  { title: 'RL · 뒤왼쪽', base: 6 },
  { title: 'RR · 뒤오른쪽', base: 9 },
];
const d2deg = (r: number) => (r * 180) / Math.PI;

function JointCell({ id }: { id: number }) {
  const { c, fonts, radius } = useTheme();
  const j = useTelemetry((s) => s.robot?.joints?.[id]);
  const locked = !!j?.locked;
  const jog = (positive: boolean) => ({
    onPressIn: () => recovery.startJog(id, positive),
    onPressOut: () => recovery.stopJog(id),
  });
  const jogBtn = (label: string, positive: boolean) => (
    <Pressable
      disabled={!locked}
      {...jog(positive)}
      style={[styles.jog, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm, opacity: locked ? 1 : 0.35 }]}
    >
      <Text style={{ color: c.muted, fontSize: 14, fontWeight: '700' }}>{label}</Text>
    </Pressable>
  );
  return (
    <Pressable
      onPress={() => recovery.lockJoint(id, !locked)}
      style={[styles.cell, {
        borderRadius: radius.sm,
        backgroundColor: locked ? 'rgba(63,185,80,0.12)' : 'rgba(231,51,28,0.12)',
        borderColor: locked ? 'rgba(63,185,80,0.55)' : 'rgba(255,107,94,0.6)',
      }]}
    >
      <View style={{ flex: 1, minWidth: 0 }}>
        <Text style={{ color: c.text, fontSize: 10.5, fontWeight: '700' }}>
          J{id} {JOINT_NAMES[id % 3]}{' '}
          <Text style={{ color: locked ? c.greenTx : c.amber, fontSize: 8.5 }}>{locked ? t('잠김') : t('풀림')}</Text>
        </Text>
        <Text style={{ color: c.muted, fontSize: 11, fontFamily: fonts.mono }}>
          {j ? `${d2deg(j.position).toFixed(1)}°` : '—'}
        </Text>
      </View>
      {jogBtn('−', false)}
      {jogBtn('+', true)}
    </Pressable>
  );
}

function LegCard({ title, base }: { title: string; base: number }) {
  const { c, radius } = useTheme();
  return (
    <View style={[styles.leg, { backgroundColor: c.bg, borderColor: c.line, borderRadius: radius.md }]}>
      <Text style={{ color: c.dim, fontSize: 9, fontWeight: '700', letterSpacing: 0.6, marginBottom: 6 }}>{t(title)}</Text>
      <View style={{ gap: 5 }}>
        {[0, 1, 2].map((i) => <JointCell key={i} id={base + i} />)}
      </View>
    </View>
  );
}

const HOLD_MS = 600;

function HoldButton({ label, icon, onFire }: { label: string; icon: IconName; onFire: () => void }) {
  const { c, radius } = useTheme();
  const [holding, setHolding] = useState(false);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const start = () => {
    setHolding(true);
    timer.current = setTimeout(() => { setHolding(false); onFire(); }, HOLD_MS);
  };
  const cancel = () => {
    setHolding(false);
    if (timer.current) { clearTimeout(timer.current); timer.current = null; }
  };
  return (
    <Pressable
      onPressIn={start}
      onPressOut={cancel}
      style={[styles.action, {
        borderRadius: radius.md,
        backgroundColor: holding ? 'rgba(63,185,80,0.35)' : 'rgba(63,185,80,0.10)',
        borderColor: 'rgba(63,185,80,0.5)',
      }]}
    >
      <Icon name={icon} size={14} color={c.green} />
      <Text style={{ color: c.greenTx, fontSize: 11, fontWeight: '600' }}>
        {label} <Text style={{ color: c.dim, fontSize: 8 }}>{t('(길게)')}</Text>
      </Text>
    </Pressable>
  );
}

export function RecoveryPanel() {
  const { c, radius } = useTheme();
  const motionConn = useTelemetry((s) => s.motionConn);
  return (
    <>
      <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 3 }}>
        🛠 {t('수동 복구')} <Text style={{ color: c.dim, fontSize: 10 }}>{t('Recovery · 위험 조작')}</Text>
      </Text>
      <Text style={{ color: c.dim, fontSize: 11, marginBottom: 12, lineHeight: 16 }}>
        {t('넘어진 로봇의 관절을 개별 잠금/조그로 정리합니다. 일반 낙상은 STAND/WALK가 자동 복구 — 이 화면은 그것도 안 될 때. 명령은 응답이 없으므로(fire-and-forget) 결과는 각도·3D로 확인하세요.')}
      </Text>
      <View style={{ flexDirection: 'row', gap: 8, marginBottom: 12, alignItems: 'center' }}>
        <Tappable onPress={() => recovery.clearErrors()}
          style={[styles.action, { borderRadius: radius.md, backgroundColor: c.elev, borderColor: c.line }]}>
          <Icon name="recover" size={14} color={c.accent2} />
          <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('오류 초기화')}</Text>
        </Tappable>
        <HoldButton label="Auto Recovery" icon="stand" onFire={() => recovery.autoRecovery()} />
        <Text style={{ color: c.dim, fontSize: 9, flex: 1 }}>
          {t('셀 탭=잠금/해제 · ± 홀드=조그(잠금 상태에서만)')}{motionConn !== 'connected' ? t(' · ⚠ 텔레메트리 끊김') : ''}
        </Text>
      </View>
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginBottom: 6 }}>
        {t('정적 모션')} <Text style={{ fontWeight: '500' }}>{t('— 궤적 없이 자세를 강제합니다. 평지에서만. LOCK은 현재 자세 유지')}</Text>
      </Text>
      <View style={{ flexDirection: 'row', gap: 8, marginBottom: 14 }}>
        <HoldButton label="POS STAND" icon="stand" onFire={() => connection.sendMotion('pos_stand')} />
        <HoldButton label="POS SIT" icon="sit" onFire={() => connection.sendMotion('pos_sit')} />
        <HoldButton label="LOCK" icon="sliders" onFire={() => connection.sendMotion('lock')} />
      </View>
      <View style={{ flexDirection: 'row', gap: 12, alignItems: 'flex-start' }}>
        <View style={{ gap: 10 }}>
          <View style={{ flexDirection: 'row', gap: 10 }}>
            <LegCard {...LEGS[0]} />
            <LegCard {...LEGS[1]} />
          </View>
          <View style={{ flexDirection: 'row', gap: 10 }}>
            <LegCard {...LEGS[2]} />
            <LegCard {...LEGS[3]} />
          </View>
        </View>
        <View style={[styles.pose, { backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.md }]}>
          <RobotModel3D controls={false} listenPresets={false} />
        </View>
      </View>
    </>
  );
}

const styles = StyleSheet.create({
  action: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 34, paddingHorizontal: 14, borderWidth: 1 },
  leg: { width: 200, borderWidth: 1, padding: 9 },
  cell: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 8, paddingVertical: 6, borderWidth: 1 },
  jog: { width: 26, height: 26, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  pose: { flex: 1, height: 306, borderWidth: 1, overflow: 'hidden', minWidth: 200 },
});
