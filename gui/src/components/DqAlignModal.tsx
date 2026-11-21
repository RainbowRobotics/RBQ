import { useEffect, useRef, useState } from 'react';
import { View, Text, ScrollView, StyleSheet, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { Modal } from '@/components/ui/overlays';
import { RobotTopViewArt } from '@/components/JointCalibArt';
import { LegHomePreview3D, type Orbit } from '@/components/LegHomePreview3D';
import { CalibFixButton } from '@/components/CalibFixButton';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { LEG_INFO, JOINT_LABELS, humanError } from '@/lib/legHomeCalib';
import { useSitGuard, sitAllowed, commissioning } from '@/lib/commissioning';
import { useCompactW } from '@/lib/layout';
import { DQ_ALIGN } from '@/types/robot';
import { t } from '@/lib/i18n';

const LEFT_LEGS = [3, 1];
const RIGHT_LEGS = [2, 0];
const MODAL_MAX_W = 760;
const REPLY_TIMEOUT_MS = 10000;
const PANE_W = 320;
const PANE_H = 260;
const PANE_H_NARROW = 200;
const r2d = (rad: number) => (rad * 180) / Math.PI;

const REF_VIEW: Orbit[] = [
  { yaw: 1.25, pitch: 1.0 },
  { yaw: 1.8, pitch: 1.05 },
  { yaw: 1.4, pitch: 0.95 },
  { yaw: 1.74, pitch: 0.95 },
];

const REF_POSE_DEG: [number, number, number][] = [
  [-32.5, 175.7, -78.9],
  [32.5, 175.7, -78.9],
  [-32.5, 175.7, -78.9],
  [32.5, 175.7, -78.9],
];

type JointState = 'sending' | 'done' | 'rejected' | 'noreply' | 'unsupported' | 'error';

export function DqAlignModal({ onClose, ignoreFall }: { onClose: () => void; ignoreFall?: boolean }) {
  const { c, radius, fonts } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const ip = useRobot((s) => s.ip);
  const dqAlign = useRobot((s) => s.robot?.dq_align);
  const guard = useSitGuard({ ignoreFall });
  const narrow = useCompactW();
  const [results, setResults] = useState<Record<number, JointState>>({});
  const [pending, setPending] = useState<{ joint: number; seq?: number } | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [sel, setSel] = useState<number | null>(null);
  const orbit = useRef<Orbit>({ yaw: Math.PI / 2, pitch: 0.35 });
  const liveJoints = useTelemetry((s) => s.robot?.joints);

  useEffect(() => {
    if (!pending || !dqAlign || dqAlign.running || dqAlign.joint !== pending.joint || dqAlign.seq === pending.seq) return;
    const st: JointState = dqAlign.result === DQ_ALIGN.done ? 'done' : dqAlign.result === DQ_ALIGN.noReply ? 'noreply' : 'rejected';
    setResults((r) => ({ ...r, [pending.joint]: st }));
    setPending(null);
    if (st === 'noreply') setErr(t('모터 보드가 답하지 않았습니다 — 보드 펌웨어가 모터 캘리브레이션을 지원하지 않을 수 있습니다.'));
  }, [dqAlign, pending]);
  const robotBusy = !!dqAlign?.running && dqAlign.joint === pending?.joint;
  useEffect(() => {
    if (!pending || robotBusy) return;
    const id = setTimeout(() => {
      setResults((r) => ({ ...r, [pending.joint]: 'unsupported' }));
      setPending(null);
      setErr(t('로봇 소프트웨어가 모터 캘리브레이션을 지원하지 않습니다 — 로봇 소프트웨어를 업데이트한 뒤 다시 실행하세요.'));
    }, REPLY_TIMEOUT_MS);
    return () => clearTimeout(id);
  }, [pending, robotBusy]);

  const align = async (j: number) => {
    if (!ip || guard.blocked || pending || !sitAllowed({ ignoreFall })) return;
    setErr(null);
    setResults((r) => ({ ...r, [j]: 'sending' }));
    setPending({ joint: j, seq: dqAlign?.seq });
    try {
      await commissioning.dqAlign(ip, j);
    } catch (e) {
      setPending(null);
      setResults((r) => ({ ...r, [j]: 'error' }));
      setErr(humanError(e));
    }
  };

  const failed = (st?: JointState) => st === 'rejected' || st === 'noreply' || st === 'unsupported' || st === 'error';
  const stateText = (st?: JointState) =>
    st === 'sending' ? '…' : st === 'done' ? t('완료') : st === 'rejected' ? t('거부') : st === 'noreply' ? t('보드 무응답')
      : st === 'unsupported' ? t('로봇 미지원') : st === 'error' ? t('실패') : t('선택');
  const stateColor = (st?: JointState) => st === 'done' ? c.greenTx : failed(st) ? c.redbright : st === 'sending' ? c.muted : c.accent2;
  const legColor = (leg: number) => {
    const sts = [0, 1, 2].map((ji) => results[leg * 3 + ji]);
    return sts.includes('sending') ? c.cyan : sts.some(failed) ? c.red : sts.includes('done') ? c.green : c.elev;
  };

  const legBox = (leg: number) => (
    <View key={leg} style={[styles.leg, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
      <View style={{ flexDirection: 'row', alignItems: 'baseline', gap: 6 }}>
        <Text style={{ color: c.text, fontSize: 16, fontWeight: '800' }}>{LEG_INFO[leg].name}</Text>
        <Text style={{ color: c.muted, fontSize: 12 }}>{t(LEG_INFO[leg].label)}</Text>
      </View>
      <View style={{ gap: 6, marginTop: 8 }}>
        {JOINT_LABELS.map((jl, ji) => {
          const j = leg * 3 + ji;
          const st = results[j];
          const locked = !!pending;
          return (
            <Tappable key={jl} disabled={locked} onPress={() => { setErr(null); setSel(j); }}
              accessibilityLabel={`${LEG_INFO[leg].name} ${jl}`}
              style={[styles.joint, {
                borderRadius: radius.sm, opacity: locked && st !== 'sending' ? 0.5 : 1,
                backgroundColor: st === 'done' ? 'rgba(63,185,80,0.14)' : c.panel,
                borderColor: st === 'done' ? 'rgba(63,185,80,0.6)' : failed(st) ? c.redbright : st === 'sending' ? c.cyan : c.line,
              }]}>
              <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', width: 46 }}>{jl}</Text>
              <Text style={{ color: c.dim, fontSize: 11, fontFamily: fonts.mono }}>J{j}</Text>
              <Text style={{ marginLeft: 'auto', fontSize: 12, fontWeight: '700', color: stateColor(st) }}>{stateText(st)}</Text>
            </Tappable>
          );
        })}
      </View>
    </View>
  );

  const art = (
    <View style={{ alignItems: 'center', justifyContent: 'center' }}>
      <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '700', marginBottom: 4 }}>{t('앞')}</Text>
      <RobotTopViewArt width={narrow ? 110 : 150} legColor={[0, 1, 2, 3].map(legColor)} />
    </View>
  );

  const detail = (j: number) => {
    const leg = Math.floor(j / 3);
    const st = results[j];
    const ref = REF_POSE_DEG[leg];
    const base = leg * 3;
    const live: [number, number, number] | undefined = liveJoints && liveJoints.length >= base + 3
      ? [r2d(liveJoints[base].position), r2d(liveJoints[base + 1].position), r2d(liveJoints[base + 2].position)]
      : undefined;
    const paneH = narrow ? PANE_H_NARROW : PANE_H;
    const running = pending?.joint === j;
    const canRun = !!ip && !guard.blocked && !pending;
    return (
      <View style={{ marginTop: 14, gap: 12 }}>
        <View style={{ flexDirection: narrow ? 'column' : 'row', gap: 12, alignItems: narrow ? 'stretch' : 'center' }}>
          <View style={{ alignItems: 'center' }}>
            <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '700', marginBottom: 4 }}>{t('앞')}</Text>
            <RobotTopViewArt width={narrow ? 90 : 110} legColor={[0, 1, 2, 3].map((l) => (l === leg ? c.cyan : c.elev))} />
          </View>
          <View style={{ flex: 1, minWidth: 0 }}>
            <Text style={{ color: c.text, fontSize: 16, fontWeight: '800' }}>
              {LEG_INFO[leg].name} · {t(LEG_INFO[leg].label)} — {JOINT_LABELS[j % 3]}
              <Text style={{ color: c.dim, fontSize: 12, fontFamily: fonts.mono }}>  J{j}</Text>
            </Text>
            <Text style={{ color: c.dim, fontSize: 13, lineHeight: 19, marginTop: 4 }}>
              {t('로봇을 앉힌 채 아래 그림의 기준 자세인지 확인한 뒤 [실행]을 누르세요. 실행하는 동안 이 관절은 힘이 빠집니다.')}
            </Text>
          </View>
        </View>

        <View style={{ flexDirection: 'row', gap: 8, justifyContent: 'center' }}>
          <View style={narrow ? { flex: 1 } : { width: PANE_W }}>
            <LegHomePreview3D leg={leg} poseDeg={ref} view={REF_VIEW[leg]} rollSign={1} orbitRef={orbit} height={paneH} badge={t('목표 자세')} />
          </View>
          <View style={narrow ? { flex: 1 } : { width: PANE_W }}>
            <LegHomePreview3D leg={leg} poseDeg={live ?? ref} view={REF_VIEW[leg]} rollSign={1} orbitRef={orbit} height={paneH}
              badge={live ? t('현재 자세') : t('현재 자세 — 관절각 수신 없음')} />
          </View>
        </View>

        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 12 }}>
          <Text style={{ flex: 1, fontSize: 14, fontWeight: '700', color: stateColor(st) }}>
            {running ? t('진행 중… 로봇이 끝낼 때까지 기다리세요') : st ? stateText(st) : ''}
          </Text>
          <Tappable disabled={!canRun} onPress={() => align(j)} accessibilityLabel={`${LEG_INFO[leg].name} ${JOINT_LABELS[j % 3]}`}
            style={[styles.primary, { backgroundColor: c.accent, borderRadius: radius.md, opacity: canRun ? 1 : 0.5 }]}>
            <Text style={{ color: '#fff', fontSize: 14, fontWeight: '700' }}>{t('실행')}</Text>
          </Tappable>
        </View>
      </View>
    );
  };

  return (
    <Modal onClose={onClose}>
      <View style={[styles.box, {
        backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg,
        width: Math.min(winW - 24, MODAL_MAX_W), maxHeight: winH - 24,
      }]}>
      <ScrollView contentContainerStyle={{ padding: 18 }}>
        <Text style={{ color: c.text, fontSize: 17, fontWeight: '700' }}>{t('모터 캘리브레이션')}</Text>

        <Notice text={t('관절의 모터 캘리브레이션 기준을 제자리에서 다시 잡습니다 — 로봇이 앉은 상태에서만 실행하세요. 관절을 고르면 그 다리 위치와 기준 자세가 나오고, [실행]을 누르면 그 관절의 보드가 잠깐 멈춰 회전자를 붙잡아 기준을 저장하고 다시 켜집니다(약 1~4초). 한 번에 관절 하나씩, 그동안 로봇을 건드리지 마세요.')} />
        {guard.blocked && (
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
            <View style={{ flex: 1 }}><Notice text={`⛔ ${guard.reason}`} /></View>
            {guard.fix && <CalibFixButton fix={guard.fix} />}
          </View>
        )}

        {sel !== null ? detail(sel) : narrow ? (
          <View style={{ gap: 10, marginTop: 14 }}>
            {art}
            <View style={{ flexDirection: 'row', gap: 10 }}>{LEFT_LEGS.map(legBox)}</View>
            <View style={{ flexDirection: 'row', gap: 10 }}>{RIGHT_LEGS.map(legBox)}</View>
          </View>
        ) : (
          <View style={{ flexDirection: 'row', gap: 12, marginTop: 14, alignItems: 'center' }}>
            <View style={{ flex: 1, gap: 10 }}>{LEFT_LEGS.map(legBox)}</View>
            {art}
            <View style={{ flex: 1, gap: 10 }}>{RIGHT_LEGS.map(legBox)}</View>
          </View>
        )}

        {err && <Text style={{ color: c.redbright, fontSize: 12, marginTop: 10 }}>{err}</Text>}

        <View style={{ flexDirection: 'row', marginTop: 14, gap: 10, justifyContent: 'flex-end' }}>
          {sel !== null && (
            <Tappable disabled={!!pending} onPress={() => { setErr(null); setSel(null); }}
              style={[styles.secondary, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, opacity: pending ? 0.5 : 1 }]}>
              <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('관절 다시 고르기')}</Text>
            </Tappable>
          )}
          <Tappable onPress={onClose}
            style={[styles.secondary, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
          </Tappable>
        </View>
      </ScrollView>
      </View>
    </Modal>
  );
}

function Notice({ text }: { text: string }) {
  const { c, radius } = useTheme();
  return (
    <View style={[styles.notice, { borderRadius: radius.md, backgroundColor: 'rgba(210,153,34,0.10)', borderColor: 'rgba(210,153,34,0.5)' }]}>
      <View style={{ flexDirection: 'row', gap: 8, alignItems: 'flex-start' }}>
        <Icon name="warn" size={15} color={c.amber} />
        <Text style={{ color: c.amber, fontSize: 13, lineHeight: 19, flex: 1 }}>{text}</Text>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  box: { maxWidth: '94%', borderWidth: 1, overflow: 'hidden' },
  leg: { flex: 1, minWidth: 0, borderWidth: 1, paddingHorizontal: 12, paddingVertical: 10 },
  joint: { flexDirection: 'row', alignItems: 'center', gap: 8, height: 44, paddingHorizontal: 12, borderWidth: 1 },
  notice: { borderWidth: 1, paddingHorizontal: 12, paddingVertical: 10, marginTop: 8 },
  secondary: { alignItems: 'center', justifyContent: 'center', height: 48, paddingHorizontal: 20, borderWidth: 1 },
  primary: { alignItems: 'center', justifyContent: 'center', height: 48, paddingHorizontal: 28 },
});
