import { useEffect, useState } from 'react';
import { View, Text, ScrollView, StyleSheet, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { Modal } from '@/components/ui/overlays';
import { RobotTopViewArt } from '@/components/JointCalibArt';
import { CalibFixButton } from '@/components/CalibFixButton';
import { useRobot } from '@/store/robot';
import { LEG_INFO, JOINT_LABELS, humanError } from '@/lib/legHomeCalib';
import { useSitGuard, sitAllowed, commissioning } from '@/lib/commissioning';
import { useCompactW } from '@/lib/layout';
import { DQ_ALIGN } from '@/types/robot';
import { t } from '@/lib/i18n';

const LEFT_LEGS = [3, 1];
const RIGHT_LEGS = [2, 0];
const MODAL_MAX_W = 760;
const REPLY_TIMEOUT_MS = 10000;

type JointState = 'sending' | 'done' | 'rejected' | 'noreply' | 'error';

export function DqAlignModal({ onClose }: { onClose: () => void }) {
  const { c, radius, fonts } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const ip = useRobot((s) => s.ip);
  const dqAlign = useRobot((s) => s.robot?.dq_align);
  const guard = useSitGuard();
  const narrow = useCompactW();
  const [results, setResults] = useState<Record<number, JointState>>({});
  const [pending, setPending] = useState<{ joint: number; seq?: number } | null>(null);
  const [err, setErr] = useState<string | null>(null);

  useEffect(() => {
    if (!pending || !dqAlign || dqAlign.running || dqAlign.joint !== pending.joint || dqAlign.seq === pending.seq) return;
    const st: JointState = dqAlign.result === DQ_ALIGN.done ? 'done' : dqAlign.result === DQ_ALIGN.noReply ? 'noreply' : 'rejected';
    setResults((r) => ({ ...r, [pending.joint]: st }));
    setPending(null);
  }, [dqAlign, pending]);
  useEffect(() => {
    if (!pending) return;
    const id = setTimeout(() => {
      setResults((r) => ({ ...r, [pending.joint]: 'noreply' }));
      setPending(null);
    }, REPLY_TIMEOUT_MS);
    return () => clearTimeout(id);
  }, [pending]);

  const align = async (j: number) => {
    if (!ip || guard.blocked || pending || !sitAllowed()) return;
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

  const failed = (st?: JointState) => st === 'rejected' || st === 'noreply' || st === 'error';
  const stateText = (st?: JointState) =>
    st === 'sending' ? '…' : st === 'done' ? t('완료') : st === 'rejected' ? t('거부') : st === 'noreply' ? t('무응답') : st === 'error' ? t('실패') : t('캘리브레이션');
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
          const locked = guard.blocked || !!pending;
          return (
            <Tappable key={jl} disabled={locked} onPress={() => align(j)}
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

  return (
    <Modal onClose={onClose}>
      <View style={[styles.box, {
        backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg,
        width: Math.min(winW - 24, MODAL_MAX_W), maxHeight: winH - 24,
      }]}>
      <ScrollView contentContainerStyle={{ padding: 18 }}>
        <Text style={{ color: c.text, fontSize: 17, fontWeight: '700' }}>{t('모터 캘리브레이션')}</Text>

        <Notice text={t('관절의 모터 캘리브레이션 기준을 제자리에서 다시 잡습니다 — 로봇이 앉은 상태에서만 실행하세요. 버튼을 누르면 그 관절의 보드가 잠깐 멈춰 회전자를 붙잡아 기준을 저장하고 다시 켜집니다(약 1~4초). 한 번에 관절 하나씩, 그동안 로봇을 건드리지 마세요.')} />
        {guard.blocked && (
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
            <View style={{ flex: 1 }}><Notice text={`⛔ ${guard.reason}`} /></View>
            {guard.fix && <CalibFixButton fix={guard.fix} />}
          </View>
        )}

        {narrow ? (
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

        <View style={{ flexDirection: 'row', marginTop: 14 }}>
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
});
