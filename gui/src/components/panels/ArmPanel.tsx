import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, Pressable, Platform } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { useRobots } from '@/store/robots';
import { armAvailable } from '@/lib/armAvailability';
import { simulatedRobot } from '@/lib/simulatedRobot';
import { useArmAvailable } from '@/lib/useArmAvailable';
import { useArmHold } from '@/hooks/useArmHold';
import { armManualReady, manualCommands as cmd, sendArmManual } from '@/lib/armManual';
import { arm, MANI_MOTION, type ManiMotion } from '@/lib/arm';
import { t } from '@/lib/i18n';

const ARM_JOINT_NAMES = ['M0Y', 'M1P', 'M2P', 'M3Y', 'M4P', 'M5Y', 'M6E'];
const PRESETS: ManiMotion[] = Object.keys(MANI_MOTION) as ManiMotion[];
const r2d = (r: number) => (r * 180) / Math.PI;

function JogBtn({ label, onStart, onStop, disabled, w = 78 }: {
  label: string; onStart: () => void; onStop: () => void; disabled?: boolean; w?: number;
}) {
  const { c, radius } = useTheme();
  return (
    <Pressable
      disabled={disabled}
      onPressIn={onStart}
      onPressOut={onStop}
      style={({ pressed }) => [styles.jog, {
        width: w, borderRadius: radius.sm, opacity: disabled ? .4 : 1,
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
  const hasArm = useArmAvailable();
  const armStat = useTelemetry((s) => s.robot?.armStat);
  const joints = useTelemetry((s) => s.robot?.joints);
  const jointCount = useTelemetry((s) => s.robot?.jointCount ?? 12);

  const canMove = hasArm && armManualReady(armStat);
  const {hold,error} = useArmHold(sendArmManual,canMove);
  const [actionError,setActionError]=useState('');
  const alive=useRef(true);
  useEffect(()=>{alive.current=true;return()=>{alive.current=false;};},[]);
  const afterStop = async (action:()=>void|Promise<unknown>) => {
    const expected=useRobots.getState().currentSerial||useRobot.getState().ip;
    hold.stop();
    try {
      await hold.settled();
      const r=useRobot.getState(),tel=useTelemetry.getState();
      if(!alive.current||r.conn!=='connected'||useRobots.getState().switching
        ||expected!==(useRobots.getState().currentSerial||r.ip)
        ||!armAvailable(tel.robot,tel.pdu,simulatedRobot()))return;
      await action();
    }
    catch(e) { if(alive.current)setActionError(e instanceof Error?e.message:String(e)); }
  };
  const jog = (type:1|2,axis:1|2|3,sign:-1|1) => ({
    disabled: !canMove,
    onStart: ()=>hold.start(`${type}/${axis}/${sign}`,cmd.jog(type,axis,sign),cmd.jog(type,axis,0)),
    onStop: ()=>hold.stop(`${type}/${axis}/${sign}`),
  });

  const armJoints = (joints ?? []).slice(12, Math.min(jointCount, 20));
  const pose = armStat?.isReady ? 'READY' : armStat?.isHome ? 'HOME' : armStat?.isStraight ? 'STRAIGHT' : armStat?.isPacking ? 'PACKING' : '—';

  return (
    <View style={styles.wrap}>
      {!!(error||actionError) && <Text accessibilityRole="alert" style={{color:c.redbright}}>{t(error||actionError)}</Text>}
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, marginBottom: 10 }}>
        <Text style={{ color: c.dim, fontSize: 10, fontWeight: '700' }}>{t('정적 자세')}</Text>
        <View style={{ flexDirection: 'row', gap: 6, flexWrap: 'wrap', flex: 1 }}>
          {PRESETS.map((p) => (
            <Tappable key={p} onPress={() => { void afterStop(()=>arm.goMotion(p)); }}
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
          <Tappable onPress={() => { void afterStop(async()=>{await sendArmManual({command:102,int:[101,0]});onExit();}); }}
            style={[styles.chip, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Icon name="x" size={12} color={c.muted} />
            <Text style={{ color: c.text, fontSize: 10, fontWeight: '600' }}>{t('나가기')}</Text>
          </Tappable>
        )}
      </View>

      <View style={{ flexDirection: 'row', gap: 10, flex: 1 }}>
        <Card title={t('위치 미세조절')} sub={t('XYZ · 누르는 동안 이동')}>
          <View style={{ alignItems: 'center', gap: 6, marginTop: 10 }}>
            <JogBtn label="N ↑" w={60} {...jog(1,1,1)} />
            <View style={{ flexDirection: 'row', gap: 5, alignItems: 'center' }}>
              <JogBtn label="W ←" w={60} {...jog(1,2,1)} />
              <Text style={{ color: c.dim, fontSize: 9, width: 20, textAlign: 'center' }}>XY</Text>
              <JogBtn label="E →" w={60} {...jog(1,2,-1)} />
            </View>
            <JogBtn label="S ↓" w={60} {...jog(1,1,-1)} />
            <View style={{ flexDirection: 'row', gap: 5, marginTop: 8 }}>
              <JogBtn label={t('UP 상승')} w={82} {...jog(1,3,1)} />
              <JogBtn label={t('DOWN 하강')} w={82} {...jog(1,3,-1)} />
            </View>
            <Text style={{ color: c.dim, fontSize: 9 }}>{t('떼면 정지')}</Text>
          </View>
        </Card>

        <Card title={t('방향 미세조절')} sub={t('RPY · 누르는 동안 회전')}>
          <View style={{ gap: 6, marginTop: 10, alignItems: 'center' }}>
            {([['Roll', 1], ['Pitch', 2], ['Yaw', 3]] as const).map(([nm, axis]) => (
              <View key={nm} style={{ flexDirection: 'row', gap: 5 }}>
                <JogBtn label={`${nm} −`} w={72} {...jog(2,axis,-1)} />
                <JogBtn label={`${nm} +`} w={72} {...jog(2,axis,1)} />
              </View>
            ))}
            <Text style={{ color: c.dim, fontSize: 9 }}>{t('떼면 정지')}</Text>
          </View>
        </Card>

        <Card title={t('그리퍼 / 모드')} sub="CAN 0x16">
          <View style={{ flexDirection: 'row', gap: 8, marginTop: 10 }}>
            <Pressable
              disabled={!canMove} onPressIn={() => hold.start('open',cmd.grip(true),cmd.gripStop)} onPressOut={() => hold.stop('open')}
              style={({ pressed }) => [styles.grip, {
                borderRadius: radius.md, borderColor: 'rgba(63,185,80,0.5)',
                backgroundColor: pressed ? 'rgba(63,185,80,0.3)' : 'rgba(63,185,80,0.10)',
              }]}
            >
              <Icon name="hand" size={15} color={c.green} />
              <Text style={{ color: c.greenTx, fontSize: 12, fontWeight: '700' }}>{t('열기')}</Text>
            </Pressable>
            <Pressable
              disabled={!canMove} onPressIn={() => hold.start('close',cmd.grip(false),cmd.gripStop)} onPressOut={() => hold.stop('close')}
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
            <Tappable onPress={() => { void afterStop(()=>arm.manualReset()); }}
              style={[styles.mode, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
              <Icon name="recover" size={13} color={c.muted} />
              <Text style={{ color: c.muted, fontSize: 11, fontWeight: '600' }}>{t('초기 위치 (Reset)')}</Text>
            </Tappable>
            <Tappable onPress={() => { void afterStop(()=>arm.fix()); }}
              style={[styles.mode, {
                borderRadius: radius.sm,
                backgroundColor: armStat?.lockPosition ? 'rgba(77,156,245,0.14)' : c.elev,
                borderColor: armStat?.lockPosition ? 'rgba(77,156,245,0.6)' : c.line,
              }]}>
              <Text style={{ color: armStat?.lockPosition ? c.text : c.muted, fontSize: 11, fontWeight: '600' }}>{t('🔒 로봇팔 고정 (Fix)')}</Text>
            </Tappable>
            <Tappable onPress={() => { void afterStop(()=>arm.control()); }}
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
  jog: { ...Platform.select({web:{touchAction:'none'}}), height: 38, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
  grip: { ...Platform.select({web:{touchAction:'none'}}), flex: 1, height: 52, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, borderWidth: 1 },
  mode: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 34, borderWidth: 1 },
  badge: { paddingHorizontal: 7, paddingVertical: 2, borderRadius: 6, borderWidth: 1 },
  jointBox: { borderWidth: 1, paddingHorizontal: 9, paddingVertical: 6 },
});
