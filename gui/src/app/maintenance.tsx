import { useEffect, useState } from 'react';
import { useLocalSearchParams, useRouter } from 'expo-router';
import { View, Text, StyleSheet, ScrollView, Pressable } from 'react-native';
import Animated, { FadeIn } from 'react-native-reanimated';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { PowerPanel } from '@/components/panels/PowerPanel';
import { VisionPanel } from '@/components/panels/VisionPanel';
import { IniEditorPanel } from '@/components/panels/IniEditorPanel';
import { LegQcPanel } from '@/components/panels/LegQcPanel';
import { CameraQcPanel } from '@/components/panels/CameraQcPanel';
import { useTelemetry } from '@/store/telemetry';
import { useDevMode, useSettings } from '@/store/settings';
import { CalibPanel } from '@/components/panels/settings/CalibPanel';
import { CheckPanel } from '@/components/panels/settings/CheckPanel';
import { FirmwarePanel } from '@/components/panels/FirmwarePanel';
import { BoardFirmwarePanel } from '@/components/panels/BoardFirmwarePanel';
import { arm, MANI_MOTION, type ManiMotion } from '@/lib/arm';
import { useHasArm } from '@/store/capability';
import { maintenanceTabs } from '@/modules/registry';
import { t } from '@/lib/i18n';
import { useCompactH } from '@/lib/layout';

type Sec = string;
const SNAV: { key: Sec; label: string; icon: IconName; dev?: boolean }[] = [
  { key: 'cal', label: '캘리브레이션', icon: 'anchor' },
  { key: 'chk', label: '로봇 점검', icon: 'pulse' },
  { key: 'sw', label: '소프트웨어 업데이트', icon: 'download' },
  { key: 'fw', label: '펌웨어 업데이트', icon: 'cpu' },
  { key: 'pwr', label: '전원 / 시스템', icon: 'power', dev: true },
  { key: 'vision', label: '비전', icon: 'track', dev: true },
  { key: 'ini', label: '설정 파일', icon: 'log', dev: true },
  { key: 'legqc', label: '다리 QC', icon: 'walk', dev: true },
  { key: 'camqc', label: '카메라 QC', icon: 'ptz', dev: true },
  { key: 'arm', label: '팔', icon: 'hand', dev: true },
];
const allNav = (): { key: Sec; label: string; icon: IconName; dev?: boolean }[] =>
  [...SNAV, ...maintenanceTabs.map((m) => ({ key: m.key, label: m.label, icon: m.icon, dev: true }))];

const d2deg = (r: number) => (r * 180) / Math.PI;


const ARM_JOINT_NAMES = ['M0Y', 'M1P', 'M2P', 'M3Y', 'M4P', 'M5Y', 'M6E'];
const ARM_PRESETS: ManiMotion[] = Object.keys(MANI_MOTION) as ManiMotion[];

function ArmJointCell({ id }: { id: number }) {
  const { c, fonts, radius } = useTheme();
  const j = useTelemetry((s) => s.robot?.joints?.[id]);
  const locked = !!j?.locked;
  const jogBtn = (label: string, positive: boolean) => (
    <Pressable
      disabled={!locked}
      onPressIn={() => arm.jointJogStart(id, positive)}
      onPressOut={() => arm.jointJogStop(id)}
      style={[styles.jog, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm, opacity: locked ? 1 : 0.35 }]}
    >
      <Text style={{ color: c.muted, fontSize: 14, fontWeight: '700' }}>{label}</Text>
    </Pressable>
  );
  return (
    <Pressable
      onPress={() => arm.jointLock(id, !locked)}
      style={[styles.cell, {
        borderRadius: radius.sm,
        backgroundColor: locked ? 'rgba(63,185,80,0.12)' : 'rgba(231,51,28,0.12)',
        borderColor: locked ? 'rgba(63,185,80,0.55)' : 'rgba(255,107,94,0.6)',
      }]}
    >
      <View style={{ flex: 1, minWidth: 0 }}>
        <Text style={{ color: c.text, fontSize: 10.5, fontWeight: '700' }}>
          J{id} {ARM_JOINT_NAMES[id - 12] ?? ''}{' '}
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

function ArmBadge({ on, label }: { on: boolean; label: string }) {
  const { c } = useTheme();
  return (
    <View style={{
      paddingHorizontal: 7, paddingVertical: 2, borderRadius: 6, borderWidth: 1,
      borderColor: on ? 'rgba(63,185,80,0.5)' : c.line,
      backgroundColor: on ? 'rgba(63,185,80,0.12)' : 'transparent',
    }}>
      <Text style={{ color: on ? c.greenTx : c.dim, fontSize: 9, fontWeight: '700' }}>{label}</Text>
    </View>
  );
}

function ArmBody() {
  const { c, radius } = useTheme();
  const armStat = useTelemetry((s) => s.robot?.armStat);
  const jointCount = useTelemetry((s) => s.robot?.jointCount ?? 12);
  const armJointIds = Array.from({ length: Math.max(0, Math.min(jointCount, 20) - 12) }, (_, i) => 12 + i);
  const pose = armStat?.isReady ? 'READY' : armStat?.isHome ? 'HOME' : armStat?.isStraight ? 'STRAIGHT' : armStat?.isPacking ? 'PACKING' : '—';
  const half = Math.ceil(armJointIds.length / 2);
  const initBtn = (label: string, onPress: () => void, primary = false) => (
    <Tappable onPress={onPress} style={[styles.action, {
      borderRadius: radius.md,
      backgroundColor: primary ? c.accent : c.elev,
      borderColor: primary ? c.accent : c.line,
    }]}>
      <Text style={{ color: primary ? c.onAccent : c.text, fontSize: 11, fontWeight: '600' }}>{label}</Text>
    </Tappable>
  );
  return (
    <>
      <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 3 }}>
        {t('🦾 팔 ')}<Text style={{ color: c.dim, fontSize: 10 }}>Manipulator · {t('위험 조작')}</Text>
      </Text>
      <Text style={{ color: c.dim, fontSize: 11, marginBottom: 10, lineHeight: 16 }}>
        {t('팔 초기화·정적 자세·관절 단위 복구. 일반 조작(EE 조그·그리퍼)은 제어 화면의 팔 조작에서.')}
      </Text>
      <View style={{ flexDirection: 'row', gap: 8, alignItems: 'center', marginBottom: 6 }}>
        {initBtn('▶ Auto Start ARM', () => arm.autoReady(), true)}
        {initBtn(t('CAN 체크'), () => arm.canCheck())}
        {initBtn(t('브레이크 해제'), () => arm.brakeRelease())}
        {initBtn(t('제어 시작'), () => arm.controlStart())}
        <Text style={{ color: c.dim, fontSize: 9, flex: 1 }}>{t('순서대로 — 상태 배지가 켜지면 다음 단계')}</Text>
      </View>
      <View style={{ flexDirection: 'row', gap: 5, marginBottom: 10, flexWrap: 'wrap' }}>
        <ArmBadge on={!!armStat?.canCheck} label={armStat?.canCheck ? 'CAN OK' : 'CAN —'} />
        <ArmBadge on={!!armStat?.brakeRelease} label={armStat?.brakeRelease ? t('브레이크 해제됨') : t('브레이크 —')} />
        <ArmBadge on={!!armStat?.conStart} label={armStat?.conStart ? t('제어 중') : t('제어 —')} />
        <ArmBadge on={pose !== '—'} label={t('자세: {p}').replace('{p}', pose)} />
      </View>
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginBottom: 6 }}>
        {t('정적 자세 ')}<Text style={{ fontWeight: '500' }}>{t('— GO_MOTION · 첫 사용은 Ready부터')}</Text>
      </Text>
      <View style={{ flexDirection: 'row', gap: 6, marginBottom: 12, flexWrap: 'wrap' }}>
        {ARM_PRESETS.map((p) => (
          <Tappable key={p} onPress={() => arm.goMotion(p)}
            style={[styles.action, { height: 28, backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Text style={{ color: c.text, fontSize: 10, fontWeight: '600' }}>{p}</Text>
          </Tappable>
        ))}
      </View>
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginBottom: 6 }}>
        {t('관절 잠금 / 조그 ')}<Text style={{ fontWeight: '500' }}>{t('— 카드 탭=잠금, ± 홀드=조그(잠금 상태에서만)')}</Text>
      </Text>
      <View style={{ flexDirection: 'row', gap: 10, alignItems: 'flex-start' }}>
        <View style={{ flex: 1, gap: 5 }}>
          {armJointIds.slice(0, half).map((id) => <ArmJointCell key={id} id={id} />)}
        </View>
        <View style={{ flex: 1, gap: 5 }}>
          {armJointIds.slice(half).map((id) => <ArmJointCell key={id} id={id} />)}
        </View>
      </View>
    </>
  );
}

export default function Maintenance() {
  const { c, radius } = useTheme();
  const devMode = useDevMode();
  const compact = useCompactH();
  const { sec: secParam } = useLocalSearchParams<{ sec?: string }>();
  const router = useRouter();
  const validSec = (v?: string): v is Sec => !!v && allNav().some((s) => s.key === v);
  const [secSel, setSec] = useState<Sec>(validSec(secParam) ? secParam : 'cal');
  useEffect(() => { if (validSec(secParam)) setSec(secParam); }, [secParam]); // eslint-disable-line react-hooks/exhaustive-deps
  const hasArm = useHasArm();
  const level3 = useSettings((s) => s.accessLevel >= 3);
  const modVisible = useTelemetry((s) => maintenanceTabs.filter((m) => m.visible(s)).map((m) => m.key).join(','));
  const isQc = (k: Sec) => k === 'legqc' || k === 'camqc';
  const nav = allNav().filter((s) => (!s.dev || devMode) && (s.key !== 'arm' || hasArm) && (!isQc(s.key) || level3)
    && (!maintenanceTabs.some((m) => m.key === s.key) || modVisible.split(',').includes(s.key)));
  const sec: Sec = nav.some((s) => s.key === secSel) ? secSel : 'cal';
  const ModPanel = maintenanceTabs.find((m) => m.key === sec)?.Panel;


  const body = (
    <Animated.View key={sec} entering={FadeIn.duration(160)} style={{ flex: 1 }}>
      {sec === 'ini' ? (
        <IniEditorPanel />
      ) : sec === 'legqc' ? (
        <LegQcPanel />
      ) : sec === 'sw' ? (
        <FirmwarePanel />
      ) : sec === 'fw' ? (
        <BoardFirmwarePanel />
      ) : (
        <ScrollView showsVerticalScrollIndicator={false}>
          {ModPanel ? <ModPanel /> : sec === 'cal' ? <CalibPanel /> : sec === 'chk' ? <CheckPanel /> : sec === 'arm' ? <ArmBody /> : sec === 'camqc' ? <CameraQcPanel /> : sec === 'vision' ? <VisionPanel /> : <PowerPanel />}
        </ScrollView>
      )}
    </Animated.View>
  );

  return (
    <Screen>
      <HubHeader title={t('정비')} subtitle={t(allNav().find((s) => s.key === sec)!.label)} />
      <View style={[styles.tabs, compact && { paddingHorizontal: 10 }]}>
        {nav.map((s) => {
          const on = s.key === sec;
          return (
            <Tappable key={s.key} onPress={() => { setSec(s.key); router.setParams({ sec: s.key }); }} accessibilityLabel={t(s.label)}
              style={[styles.tab, { backgroundColor: on ? 'rgba(77,156,245,0.18)' : c.glass, borderColor: on ? 'rgba(77,156,245,0.6)' : c.glassLine, borderRadius: radius.md }]}>
              <Icon name={s.icon} size={15} color={on ? c.accent2 : c.muted} />
              {!compact && <Text style={{ color: on ? c.text : c.muted, fontSize: 12.5, fontWeight: '600' }}>{t(s.label)}</Text>}
            </Tappable>
          );
        })}
      </View>
      <View style={[styles.panel, { backgroundColor: c.glassHi, borderColor: c.glassLine, borderRadius: radius.lg }, compact && { marginHorizontal: 10, marginBottom: 10, paddingHorizontal: 14, paddingVertical: 10 }]}>{body}</View>
    </Screen>
  );
}

const styles = StyleSheet.create({
  tabs: { flexDirection: 'row', gap: 8, paddingHorizontal: 16, paddingTop: 4, paddingBottom: 10, flexWrap: 'wrap' },
  tab: { height: 36, paddingHorizontal: 14, borderWidth: 1, flexDirection: 'row', alignItems: 'center', gap: 8 },
  panel: { flex: 1, marginHorizontal: 16, marginBottom: 16, borderWidth: 1, paddingHorizontal: 22, paddingVertical: 16, overflow: 'hidden' },
  wrap: { flex: 1, flexDirection: 'row', padding: 15, gap: 14 },
  snav: { width: 190, borderWidth: 1, borderRadius: 14, padding: 8, gap: 3 },
  snavItem: { flexDirection: 'row', alignItems: 'center', gap: 11, padding: 11, borderRadius: 9 },
  sbody: { flex: 1, borderWidth: 1, borderRadius: 14, paddingHorizontal: 22, paddingVertical: 18 },
  action: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 34, paddingHorizontal: 14, borderWidth: 1 },
  cell: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 8, paddingVertical: 6, borderWidth: 1 },
  jog: { width: 26, height: 26, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
