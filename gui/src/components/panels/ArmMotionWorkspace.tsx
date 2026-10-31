import { useCallback, useEffect, useRef, useState } from 'react';
import { Platform, ScrollView, StyleSheet, Text, View, useWindowDimensions } from 'react-native';
import { useRouter } from 'expo-router';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { RobotModel3D } from '@/components/RobotModel3D';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useArmAvailable } from '@/lib/useArmAvailable';
import { arm } from '@/lib/arm';
import { armMotionReady } from '@/lib/armMotion';
import { t } from '@/lib/i18n';

type Item = { id: string; label: string; run: () => void };

export function ArmMotionWorkspace({ onExit }: { onExit: () => void }) {
  const { c, radius } = useTheme();
  const { width, height } = useWindowDimensions();
  const compact = height < 600, narrow = width < 900;
  const router = useRouter();
  const connected = useRobot(s => s.conn === 'connected');
  const hasArm = useArmAvailable();
  const stat = useTelemetry(s => s.robot?.armStat);
  const [error, setError] = useState('');
  const [last, setLast] = useState<string | null>(null);
  const [readyDone, setReadyDone] = useState(false);
  const alive = useRef(true);
  useEffect(() => { alive.current = true; return () => { alive.current = false; }; }, []);

  const ready = connected && hasArm && armMotionReady(stat);
  useEffect(() => { if (!ready) setReadyDone(false); }, [ready]);

  const fire = useCallback((id: string, run: () => void) => {
    setError(''); setLast(id);
    try { run(); } catch (e) { if (alive.current) setError(e instanceof Error ? e.message : String(e)); }
  }, []);

  const goReady = () => { fire('ready', () => arm.goMotion('Ready')); setReadyDone(true); };
  const goFolding = () => fire('folding', () => arm.goMotion('Folding', 1));

  const motions: Item[] = [
    { id: 'dGreeting', label: 'Greeting', run: () => arm.demoGreeting() },
    { id: 'dMulti', label: 'Multi joint', run: () => arm.demoAllJointMove() },
    { id: 'dRecovery', label: 'Auto Recovery', run: () => arm.demoAutoRecovery() },
    { id: 'dDamping', label: 'Damping Mode', run: () => arm.demoDampingMode() },
  ];

  const enabled = ready && readyDone;
  const cell = (it: Item, on = enabled) =>
    <Tappable key={it.id} disabled={!on} onPress={() => fire(it.id, it.run)}
      style={[styles.cell, { borderRadius: radius.sm, borderColor: last === it.id ? c.accent2 : c.line,
        backgroundColor: c.elev, opacity: on ? 1 : .4, height: compact ? 30 : 36 }]}>
      <Text style={{ color: last === it.id ? c.accent2 : c.text, fontSize: 11, fontWeight: '600' }}>{t(it.label)}</Text>
    </Tappable>;

  const status = !connected ? '연결 확인 필요'
    : !hasArm ? '팔 전원 및 통신 확인 필요'
    : !stat?.conStart ? '팔 제어 시작을 확인하세요.'
    : stat.lockPosition ? '팔 위치 고정'
    : !readyDone ? '준비 자세(Ready)를 먼저 실행하세요.'
    : '모션 실행 가능';

  return <View testID="arm-motion-workspace" style={[styles.root, { gap: compact ? 6 : 10, padding: compact ? 8 : 12 }]}>
    <View style={[styles.toolbar, { borderColor: c.line }]}>
      <View style={{ flex: 1, minWidth: 110 }}>
        <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{t('팔 모션')}</Text>
        <Text style={{ color: enabled ? c.greenTx : c.muted, fontSize: 11, marginTop: 3 }}>{t(status)}</Text>
      </View>
      <Tappable disabled={!ready} onPress={goReady}
        style={[styles.button, { borderRadius: radius.md, borderColor: readyDone ? c.line : c.accent2,
          backgroundColor: readyDone ? c.elev : 'rgba(77,156,245,0.12)', opacity: ready ? 1 : .4 }]}>
        <Text style={{ color: readyDone ? c.text : c.accent2, fontSize: 11, fontWeight: '700' }}>{t('준비 자세 (Ready)')}</Text>
      </Tappable>
      <Tappable disabled={!ready} onPress={goFolding}
        style={[styles.button, { borderRadius: radius.md, borderColor: c.line, backgroundColor: c.elev, opacity: ready ? 1 : .4 }]}>
        <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('접기 (Folding)')}</Text>
      </Tappable>
      <Tappable onPress={() => router.replace('/arm-manual')}
        style={[styles.button, { borderRadius: radius.md, borderColor: c.line, backgroundColor: c.elev }]}>
        <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('수동제어')}</Text>
      </Tappable>
      <Tappable onPress={onExit} style={[styles.button, { borderRadius: radius.md, borderColor: c.line, backgroundColor: c.elev }]}>
        <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('나가기')}</Text>
      </Tappable>
    </View>

    {!!error && <Text accessibilityRole="alert" style={{ color: c.redTx, fontSize: 11 }}>{error}</Text>}
    {ready && !readyDone && <Text style={{ color: c.amberTx, fontSize: 11 }}>
      {t('ManiControl 은 첫 모션이 Ready 가 아니면 거부합니다 — 위 [준비 자세 (Ready)] 를 먼저 누르세요.')}</Text>}

    <View style={[styles.body, { flexDirection: narrow ? 'column' : 'row' }]}>
      <ScrollView style={[styles.list, { borderColor: c.line, borderRadius: radius.lg, backgroundColor: c.panel }]}
        contentContainerStyle={{ padding: compact ? 8 : 10, gap: 10 }}>
        <View style={styles.grid}>{motions.map(it => cell(it))}</View>
      </ScrollView>

      <View testID="arm-motion-model" style={[styles.pane, { borderColor: c.line, borderRadius: radius.lg, backgroundColor: c.panel2 }]}>
        <View style={[styles.paneHead, { backgroundColor: c.panel }]}>
          <Text style={{ color: c.text, fontWeight: '700', fontSize: 12 }}>{t('로봇 · 팔 3D')}</Text>
          <Text style={{ color: c.muted, fontSize: 10 }}>{t('드래그하여 시점 회전')}</Text>
        </View>
        <View style={{ flex: 1 }}>
          <RobotModel3D active controls listenPresets={false} showPresetRow={false} initialDist={2.5} gridSize={8} gridDivisions={16} backgroundColor={c.panel2} />
        </View>
      </View>
    </View>
  </View>;
}

const styles = StyleSheet.create({
  root: { flex: 1, minHeight: 0 },
  toolbar: { flexDirection: 'row', alignItems: 'center', gap: 6, flexWrap: 'wrap', paddingBottom: 8, borderBottomWidth: 1 },
  button: { paddingHorizontal: 11, paddingVertical: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  body: { flex: 1, gap: 10, minHeight: 0 },
  list: { flex: 1, borderWidth: 1 },
  pane: { flex: 1, overflow: 'hidden', borderWidth: 1, minHeight: 180 },
  paneHead: { minHeight: 40, padding: 7, flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 5, flexWrap: 'wrap' },
  grid: { flexDirection: 'row', flexWrap: 'wrap', gap: 6 },
  cell: { minWidth: 118, flexGrow: 1, alignItems: 'center', justifyContent: 'center', borderWidth: 1, ...(Platform.OS === 'web' ? { touchAction: 'none' as const } : {}) },
});
