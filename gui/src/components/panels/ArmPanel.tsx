import { useEffect } from 'react';
import { View, Text, StyleSheet, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useTelemetry } from '@/store/telemetry';
import { useHasArm } from '@/store/capability';
import { arm, MANI_MOTION, type ManiMotion } from '@/lib/arm';
import { t } from '@/lib/i18n';

const ARM_JOINT_NAMES = ['M0Y', 'M1P', 'M2P', 'M3Y', 'M4P', 'M5Y', 'M6E'];
const PRESETS: ManiMotion[] = Object.keys(MANI_MOTION) as ManiMotion[];
const r2d = (r: number) => (r * 180) / Math.PI;

function JogBtn({ label, onStart, onStop, w = 78 }: {
  label: string; onStart: () => void; onStop: () => void; w?: number;
}) {
  const { c, radius } = useTheme();
  return (
    <Pressable
      onPressIn={onStart}
      onPressOut={onStop}
      style={({ pressed }) => [styles.jog, {
        width: w, borderRadius: radius.sm,
        backgroundColor: pressed ? 'rgba(77,156,245,0.18)' : c.elev,
        borderColor: pressed ? 'rgba(77,156,245,0.6)' : c.line,
      }]}
    >
      <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>{label}</Text>
    </Pressable>
  );
}

function Card({ title, sub, children }: { title: string; sub: string; children: React.ReactNode }) {
  const { c, radius } = useTheme();
  return (
    <View style={[styles.card, { backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.md }]}>
      <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>
        {title} <Text style={{ color: c.dim, fontSize: 9, fontWeight: '500' }}>{sub}</Text>
      </Text>
      {children}
    </View>
  );
}

export function ArmPanel({ onExit, onOpenDoor }: {
  onExit?: () => void;
  onOpenDoor?: () => void;
}) {
  const { c, fonts, radius } = useTheme();
  const hasArm = useHasArm();
  const armStat = useTelemetry((s) => s.robot?.armStat);
  const joints = useTelemetry((s) => s.robot?.joints);
  const jointCount = useTelemetry((s) => s.robot?.jointCount ?? 12);

  useEffect(() => { if (hasArm) arm.manualReset(); }, [hasArm]);

  const armJoints = (joints ?? []).slice(12, Math.min(jointCount, 20));
  const pose = armStat?.isReady ? 'READY' : armStat?.isHome ? 'HOME' : armStat?.isStraight ? 'STRAIGHT' : armStat?.isPacking ? 'PACKING' : '—';

  return (
    <View style={styles.wrap}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, marginBottom: 10 }}>
        <Text style={{ color: c.dim, fontSize: 10, fontWeight: '700' }}>{t('정적 자세')}</Text>
        <View style={{ flexDirection: 'row', gap: 6, flexWrap: 'wrap', flex: 1 }}>
          {PRESETS.map((p) => (
            <Tappable key={p} onPress={() => arm.goMotion(p)}
              style={[styles.chip, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
              <Text style={{ color: c.text, fontSize: 10, fontWeight: '600' }}>{p}</Text>
            </Tappable>
          ))}
        </View>
        {onOpenDoor && (
          <Tappable onPress={onOpenDoor}
            style={[styles.chip, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Text style={{ color: c.text, fontSize: 10, fontWeight: '600' }}>{t('🚪 문 열기')}</Text>
          </Tappable>
        )}
        {onExit && (
          <Tappable onPress={() => { arm.goMotion('Folding'); onExit(); }}
            style={[styles.chip, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Icon name="x" size={12} color={c.muted} />
            <Text style={{ color: c.text, fontSize: 10, fontWeight: '600' }}>{t('나가기')}</Text>
          </Tappable>
        )}
      </View>

      <View style={{ flexDirection: 'row', gap: 10, flex: 1 }}>
        <Card title={t('위치 미세조절')} sub={t('XYZ · 누르는 동안 이동')}>
          <View style={{ alignItems: 'center', gap: 6, marginTop: 10 }}>
            <JogBtn label="N ↑" w={60} onStart={() => arm.jogXyz(1, 1)} onStop={() => arm.jogXyz(1, 0)} />
            <View style={{ flexDirection: 'row', gap: 5, alignItems: 'center' }}>
              <JogBtn label="W ←" w={60} onStart={() => arm.jogXyz(2, -1)} onStop={() => arm.jogXyz(2, 0)} />
              <Text style={{ color: c.dim, fontSize: 9, width: 20, textAlign: 'center' }}>XY</Text>
              <JogBtn label="E →" w={60} onStart={() => arm.jogXyz(2, 1)} onStop={() => arm.jogXyz(2, 0)} />
            </View>
            <JogBtn label="S ↓" w={60} onStart={() => arm.jogXyz(1, -1)} onStop={() => arm.jogXyz(1, 0)} />
            <View style={{ flexDirection: 'row', gap: 5, marginTop: 8 }}>
              <JogBtn label={t('IN 전진')} w={82} onStart={() => arm.jogXyz(3, 1)} onStop={() => arm.jogXyz(3, 0)} />
              <JogBtn label={t('OUT 후진')} w={82} onStart={() => arm.jogXyz(3, -1)} onStop={() => arm.jogXyz(3, 0)} />
            </View>
            <Text style={{ color: c.dim, fontSize: 9 }}>{t('떼면 정지')}</Text>
          </View>
        </Card>

        <Card title={t('방향 미세조절')} sub={t('RPY · 누르는 동안 회전')}>
          <View style={{ gap: 6, marginTop: 10, alignItems: 'center' }}>
            {([['Roll', 1], ['Pitch', 2], ['Yaw', 3]] as const).map(([nm, axis]) => (
              <View key={nm} style={{ flexDirection: 'row', gap: 5 }}>
                <JogBtn label={`${nm} −`} w={72} onStart={() => arm.jogRpy(axis, -1)} onStop={() => arm.jogRpy(axis, 0)} />
                <JogBtn label={`${nm} +`} w={72} onStart={() => arm.jogRpy(axis, 1)} onStop={() => arm.jogRpy(axis, 0)} />
              </View>
            ))}
            <Text style={{ color: c.dim, fontSize: 9 }}>{t('떼면 정지')}</Text>
          </View>
        </Card>

        <Card title={t('그리퍼 / 모드')} sub="CAN 0x16">
          <View style={{ flexDirection: 'row', gap: 8, marginTop: 10 }}>
            <Pressable
              onPressIn={() => arm.gripper('open')} onPressOut={() => arm.gripper('stop')}
              style={({ pressed }) => [styles.grip, {
                borderRadius: radius.md, borderColor: 'rgba(63,185,80,0.5)',
                backgroundColor: pressed ? 'rgba(63,185,80,0.3)' : 'rgba(63,185,80,0.10)',
              }]}
            >
              <Icon name="hand" size={15} color={c.green} />
              <Text style={{ color: c.greenTx, fontSize: 12, fontWeight: '700' }}>{t('열기')}</Text>
            </Pressable>
            <Pressable
              onPressIn={() => arm.gripper('close')} onPressOut={() => arm.gripper('stop')}
              style={({ pressed }) => [styles.grip, {
                borderRadius: radius.md, borderColor: 'rgba(210,153,34,0.5)',
                backgroundColor: pressed ? 'rgba(210,153,34,0.3)' : 'rgba(210,153,34,0.10)',
              }]}
            >
              <Icon name="hand" size={15} color={c.amber} />
              <Text style={{ color: c.amberTx, fontSize: 12, fontWeight: '700' }}>{t('닫기')}</Text>
            </Pressable>
          </View>
          <Text style={{ color: c.dim, fontSize: 9, textAlign: 'center', marginTop: 5 }}>{t('누르는 동안 동작 · 떼면 정지')}</Text>
          <Text style={{ color: c.text, fontSize: 11, fontWeight: '700', marginTop: 14 }}>{t('모드')}</Text>
          <View style={{ gap: 6, marginTop: 8 }}>
            <Tappable onPress={() => arm.manualReset()}
              style={[styles.mode, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
              <Icon name="recover" size={13} color={c.muted} />
              <Text style={{ color: c.muted, fontSize: 11, fontWeight: '600' }}>{t('초기 위치 (Reset)')}</Text>
            </Tappable>
            <Tappable onPress={() => arm.fix()}
              style={[styles.mode, {
                borderRadius: radius.sm,
                backgroundColor: armStat?.lockPosition ? 'rgba(77,156,245,0.14)' : c.elev,
                borderColor: armStat?.lockPosition ? 'rgba(77,156,245,0.6)' : c.line,
              }]}>
              <Text style={{ color: armStat?.lockPosition ? c.text : c.muted, fontSize: 11, fontWeight: '600' }}>{t('🔒 로봇팔 고정 (Fix)')}</Text>
            </Tappable>
            <Tappable onPress={() => arm.control()}
              style={[styles.mode, {
                borderRadius: radius.sm,
                backgroundColor: !armStat?.lockPosition ? 'rgba(77,156,245,0.14)' : c.elev,
                borderColor: !armStat?.lockPosition ? 'rgba(77,156,245,0.6)' : c.line,
              }]}>
              <Icon name="hand" size={13} color={!armStat?.lockPosition ? c.accent2 : c.muted} />
              <Text style={{ color: !armStat?.lockPosition ? c.text : c.muted, fontSize: 11, fontWeight: '600' }}>{t('로봇팔 제어 (Control)')}</Text>
            </Tappable>
          </View>
        </Card>

        <Card title={t('상태')} sub="RobotState arm_B">
          <View style={{ flexDirection: 'row', gap: 5, flexWrap: 'wrap', marginTop: 10, marginBottom: 6 }}>
            <View style={[styles.badge, {
              borderColor: armStat?.conStart ? 'rgba(63,185,80,0.5)' : c.line,
              backgroundColor: armStat?.conStart ? 'rgba(63,185,80,0.12)' : 'transparent',
            }]}>
              <Text style={{ color: armStat?.conStart ? c.greenTx : c.dim, fontSize: 8.5, fontWeight: '700' }}>
                {armStat?.conStart ? t('제어 중') : t('제어 꺼짐')}
              </Text>
            </View>
            <View style={[styles.badge, { borderColor: c.line }]}>
              <Text style={{ color: c.dim, fontSize: 8.5, fontWeight: '700' }}>{pose}</Text>
            </View>
          </View>
          <View style={[styles.jointBox, { backgroundColor: c.bg, borderColor: c.line, borderRadius: radius.sm }]}>
            {armJoints.map((j, i) => (
              <View key={i} style={{ flexDirection: 'row', justifyContent: 'space-between', paddingVertical: 1.5 }}>
                <Text style={{ color: c.dim, fontSize: 9.5, fontFamily: fonts.mono }}>{ARM_JOINT_NAMES[i] ?? `A${i}`}</Text>
                <Text style={{ color: c.muted, fontSize: 9.5, fontFamily: fonts.mono }}>{r2d(j.position).toFixed(1)}°</Text>
              </View>
            ))}
            {armJoints.length === 0 && <Text style={{ color: c.dim, fontSize: 9.5 }}>{t('관절 수신 대기…')}</Text>}
          </View>
        </Card>
      </View>

      <Text style={{ color: c.amberTx, fontSize: 10, marginTop: 8 }}>
        {t('⚠ 팔 제어 전 정비 → 팔에서 ')}<Text style={{ fontWeight: '700' }}>{t('제어 시작')}</Text>{t('이 켜져 있어야 합니다. 첫 사용 시 Ready 자세부터 — 다른 모션이 거부됩니다.')}
      </Text>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { flex: 1, padding: 15 },
  card: { flex: 1, borderWidth: 1, paddingHorizontal: 12, paddingVertical: 10, minWidth: 0 },
  chip: { flexDirection: 'row', alignItems: 'center', gap: 4, height: 28, paddingHorizontal: 11, borderWidth: 1 },
  jog: { height: 38, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
  grip: { flex: 1, height: 52, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, borderWidth: 1 },
  mode: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 34, borderWidth: 1 },
  badge: { paddingHorizontal: 7, paddingVertical: 2, borderRadius: 6, borderWidth: 1 },
  jointBox: { borderWidth: 1, paddingHorizontal: 9, paddingVertical: 6 },
});
