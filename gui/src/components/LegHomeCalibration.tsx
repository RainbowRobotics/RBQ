import { useMemo, useRef, useState } from 'react';
import { View, Text, Image, ScrollView, StyleSheet, ActivityIndicator, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { LegHomePreview3D, type Orbit, type JointAnim } from '@/components/LegHomePreview3D';
import { SittingRobotArt, PitchJigArt } from '@/components/JointCalibArt';
import { Modal } from '@/components/ui/overlays';
import { useRb } from '@/rb/theme';
import { RBBadge } from '@/rb/components/RBBadge';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import {
  useLegHomeSet, startLeg, jointGuide, legTitle, homeTargetDeg, doneLabel, groupDone, isRollNeutral,
  rollTargetDeg, LEG_COUNT, LEG_INFO, JOINT_LABELS, type LegHomeSetLeg, type LegHomeGroup, type LegHomeRef,
} from '@/lib/legHomeCalib';
import { useLegHomeGuard, legHomeCalibAllowed, type CalibFix } from '@/lib/commissioning';
import { CalibFixButton } from '@/components/CalibFixButton';
import { LegibleText, useLegible, H2, Desc, Group } from '@/components/panels/settings/common';
import { useCompactW } from '@/lib/layout';
import { t } from '@/lib/i18n';
import { AutoStartModal } from '@/components/control/overlays';

const r2d = (rad: number) => (rad * 180) / Math.PI;

const ROLL_START_DEG = 20;
const PITCH_START_DEG = 135;
const KNEE_START_DEG = -90;

const PANE_H = 330;
const PANE_H_NARROW = 240;
const PANE_W = 200;
const LEG_COL_W = 112;
const MODAL_MAX_W = 1180;

export function LegHomeCalibration() {
  const { c } = useTheme();
  const lg = useLegible();
  const connected = useRobot((s) => s.conn === 'connected');
  const { status, err: pollErr, refresh } = useLegHomeSet(useRobot((s) => s.ip), connected);
  const guard = useLegHomeGuard();
  const boardsDead = status?.boards_alive === false;
  const legs = status?.legs ?? [];
  const allDone = legs.length === LEG_COUNT && legs.every((l) => l.ok === true);

  const [step, setStep] = useState<null | 'intro' | 'wizard'>(null);

  const blocked = guard.blocked || boardsDead;
  const blockReason = boardsDead
    ? t('모터 보드와 통신이 없습니다 — 48V 구동(LEGS) 전원을 켠 뒤 다시 실행하세요.')
    : guard.reason;

  return (
    <>
      <H2>{t('관절 캘리브레이션 ')}<Text style={{ color: lg ? c.muted : c.dim, fontSize: lg ? 11 : 10 }}>Joint Calibration</Text></H2>
      <Desc>{t('다리 관절의 영점·오프셋을 다시 잡습니다 — 로봇을 앉힌 상태에서 실행합니다.')}</Desc>

      <Group>
      <Row
        nm={t('관절 홈포즈 세팅')}
        sub={blocked ? `⛔ ${blockReason}` : t('치구로 홈 자세를 고정한 뒤 그 자세를 관절 영점으로 저장 — 다리 4개')}
        subColor={blocked ? c.redbright : undefined}
        blocked={blocked}
        fix={boardsDead ? undefined : guard.fix}
        onPress={() => setStep('intro')} />

      <Row nm={t('모터 캘리브레이션')} sub={t('준비 중')} blocked disabled onPress={() => {}} />
      </Group>

      {allDone && <Notice tone="ok" text={t('네 다리 모두 완료된 기록이 있습니다.')} />}
      <PollNote connected={connected} pollErr={pollErr} status={status} />

      <LegibleText.Provider value={false}>
        {step === 'intro' && (
          <HomeSetIntroModal
            blocked={blocked} blockReason={blockReason}
            onClose={() => setStep(null)} onRun={() => setStep('wizard')} />
        )}
        {step === 'wizard' && (
          <LegHomeWizardModal status={status} refresh={refresh} onClose={() => setStep(null)} />
        )}
      </LegibleText.Provider>
    </>
  );
}

function Row({ nm, sub, subColor, blocked, disabled, fix, onPress }: {
  nm: string; sub: string; subColor?: string; blocked: boolean; disabled?: boolean;
  fix?: CalibFix;
  onPress: () => void;
}) {
  const { c, radius } = useTheme();
  const lg = useLegible();
  return (
    <View style={[styles.trow, { borderTopColor: c.line2 }]}>
      <View style={{ flex: 1 }}>
        <Text style={{ color: disabled ? c.dim : c.text, fontSize: lg ? 15 : 13, fontWeight: lg ? '600' : undefined }}>{nm}</Text>
        <Text style={{ color: subColor ?? (lg ? c.muted : c.dim), fontSize: lg ? 12 : 10.5, marginTop: 2 }}>{sub}</Text>
      </View>
      <View style={styles.runRow}>
        {blocked && fix && <CalibFixButton fix={fix} />}
        <Tappable disabled={blocked} onPress={blocked ? undefined : onPress}
          style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm, opacity: blocked ? 0.4 : 1 }]}>
          <Text style={{ color: c.text, fontSize: 19, fontWeight: '800' }}>{t('실행')}</Text>
        </Tappable>
      </View>
    </View>
  );
}

function HomeSetIntroModal({ blocked, blockReason, onClose, onRun }: {
  blocked: boolean; blockReason: string; onClose: () => void; onRun: () => void;
}) {
  const { c, radius } = useTheme();
  const narrow = useCompactW();
  return (
    <Modal onClose={onClose}>
      <View style={[styles.introBox, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: narrow ? 340 : 560 }]}>
        <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{t('관절 홈포즈 세팅')}</Text>

        <Notice tone="warn" text={t('관절 영점을 다시 쓰는 조작입니다 — 치구가 없거나 자세가 어긋난 상태로 실행하면 로봇이 잘못된 영점으로 기동합니다.')} />

        <View style={narrow ? { gap: 10, marginTop: 12 } : { flexDirection: 'row', gap: 12, marginTop: 12 }}>
          <Figure cap={t('로봇이 앉은 상태로 진행합니다.')}>
            <SittingRobotArt width={narrow ? 240 : 230} />
          </Figure>
          <Figure cap={t('pitch 축 세팅을 위한 고정 치구가 필요합니다.')}>
            <PitchJigArt width={narrow ? 240 : 230} />
          </Figure>
        </View>

        {blocked && <Notice tone="warn" text={`⛔ ${blockReason}`} />}

        <View style={{ flexDirection: 'row', gap: 10, marginTop: 14 }}>
          <Tappable onPress={onClose} style={[styles.modalBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{t('취소')}</Text>
          </Tappable>
          <Tappable disabled={blocked} onPress={blocked ? undefined : onRun}
            style={[styles.modalBtn, { backgroundColor: c.redbright, borderColor: 'transparent', borderRadius: radius.md, opacity: blocked ? 0.4 : 1 }]}>
            <Text style={{ color: c.onAccent, fontSize: 13, fontWeight: '700' }}>{t('실행')}</Text>
          </Tappable>
        </View>
      </View>
    </Modal>
  );
}

function Figure({ cap, children }: { cap: string; children: React.ReactNode }) {
  const { c, radius } = useTheme();
  return (
    <View style={{ flex: 1, minWidth: 0 }}>
      <View style={[styles.figure, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
        {children}
      </View>
      <Text style={{ color: c.muted, fontSize: 11, lineHeight: 16, marginTop: 6, textAlign: 'center' }}>{cap}</Text>
    </View>
  );
}

function LegHomeWizardModal({ status, refresh, onClose }: {
  status: ReturnType<typeof useLegHomeSet>['status']; refresh: () => void; onClose: () => void;
}) {
  const { c, radius, fonts } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const ip = useRobot((s) => s.ip);
  const [leg, setLeg] = useState(0);
  const [err, setErr] = useState<string | null>(null);
  const [starting, setStarting] = useState(false);
  const narrow = useCompactW();
  const liveJoints = useTelemetry((s) => s.robot?.joints);
  const liveDeg = useMemo<[number, number, number] | undefined>(() => {
    const base = leg * 3;
    if (!liveJoints || liveJoints.length < base + 3) return undefined;
    return [r2d(liveJoints[base].position), r2d(liveJoints[base + 1].position), r2d(liveJoints[base + 2].position)];
  }, [liveJoints, leg]);

  const rawLegs = status?.legs ?? [];

  const [ranGroups, setRanGroups] = useState<Record<string, true>>({});
  const legs: LegHomeSetLeg[] = rawLegs.map((l, i) => {
    const rollRan = !!ranGroups[`${i}:roll`];
    const pkRan   = !!ranGroups[`${i}:pitch_knee`];
    if (!rollRan && !pkRan) {
      return { ...l, ok: undefined, reason: undefined, measured_deg: undefined,
               joint_done: undefined, joint_ok: undefined, finished_at: undefined };
    }
    const keep: [boolean, boolean, boolean] = [rollRan, pkRan, pkRan];
    const done = ([0, 1, 2] as const).map((j) => keep[j] && !!l.joint_done?.[j]) as [boolean, boolean, boolean];
    const okj  = ([0, 1, 2] as const).map((j) => keep[j] && !!l.joint_ok?.[j]) as [boolean, boolean, boolean];
    return { ...l, joint_done: done, joint_ok: okj, ok: done.every(Boolean) && okj.every(Boolean) };
  });
  const entry: LegHomeSetLeg | undefined = legs[leg];
  const robotExpected = entry?.expected_deg ?? homeTargetDeg(leg);
  const [rollRef, setRollRef] = useState<LegHomeRef>('limit');
  const expected: [number, number, number] = [rollTargetDeg(rollRef, robotExpected[0]), robotExpected[1], robotExpected[2]];
  const judgedExpected = (j: 0 | 1 | 2) =>
    j === 0 && entry?.roll_ref ? rollTargetDeg(entry.roll_ref, robotExpected[0]) : expected[j];
  const jointCalibrated = (j: 0 | 1 | 2) => !!entry?.joint_done?.[j];

  const running = !!status?.running;
  const runningThisLeg = running && status?.leg === leg;
  const boardsDead = status?.boards_alive === false;
  const guard = useLegHomeGuard();
  const allDone = legs.length === LEG_COUNT && legs.every((l) => l.ok === true);

  const [sel, setSel] = useState<LegHomeGroup>('roll');
  const paneH = narrow ? PANE_H_NARROW : PANE_H;
  const orbit = useRef<Orbit>({ yaw: Math.PI / 2, pitch: 0.35 });
  const base: [number, number, number] = liveDeg ?? expected;
  const anim: JointAnim = sel === 'roll'
    ? {
        moves: [{ axis: 0, fromDeg: leg % 2 === 0 ? ROLL_START_DEG : -ROLL_START_DEG, toDeg: expected[0] }],
        cycleKey: `${leg}:roll`,
      }
    : {
        moves: [
          { axis: 1, fromDeg: PITCH_START_DEG, toDeg: expected[1] },
          { axis: 2, fromDeg: KNEE_START_DEG, toDeg: expected[2] },
        ],
        cycleKey: `${leg}:pitchknee`,
      };

  const savedJoints = ([0, 1, 2] as const).filter((j) => entry?.joint_done?.[j]);
  const savedOk = savedJoints.length > 0 && savedJoints.every((j) => entry?.joint_ok?.[j]);
  const savedName = savedJoints.length === 3 ? 'roll/pitch/knee'
    : savedJoints.includes(0) && savedJoints.length === 1 ? 'roll' : 'pitch/knee';

  const groupStyle = (g: LegHomeGroup) => groupDone(entry, g)
    ? { backgroundColor: 'rgba(63,185,80,0.12)', borderColor: 'rgba(63,185,80,0.6)' }
    : { backgroundColor: c.elev, borderColor: c.line };

  const groupButton = (g: LegHomeGroup) => {
    const done = groupDone(entry, g);
    const busy = running || starting;
    const blocked = boardsDead || guard.blocked;
    return (
      <Tappable disabled={busy || blocked} onPress={() => run(g)}
        style={[styles.primary, styles.cardBtn, {
          backgroundColor: busy ? c.elev : done ? c.green : c.accent,
          borderColor: busy ? c.line : done ? c.green : c.accent,
          borderRadius: radius.md, opacity: busy ? 0.6 : blocked ? 0.4 : 1,
        }]}>
        {busy ? <ActivityIndicator size="small" color={c.muted} /> : <Icon name="save" size={17} color={c.onAccent} />}
        <Text style={{ color: busy ? c.muted : c.onAccent, fontSize: 15, fontWeight: '700' }}>
          {runningThisLeg ? t('진행 중… 로봇이 끝낼 때까지 기다리세요')
            : running ? t('다른 다리 진행 중…')
            : t('LEG{n} - {g} 홈포즈 저장').replace('{n}', String(leg)).replace('{g}', g === 'roll' ? 'roll' : 'pitch/knee')}
        </Text>
      </Tappable>
    );
  };

  const jointBody = (j: 0 | 1 | 2) => (
    <View>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
        <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '800', width: 52 }}>{JOINT_LABELS[j]}</Text>
        <Text style={{ color: c.dim, fontSize: 11.5 }}>J{entry?.joints?.[j] ?? leg * 3 + j}</Text>
        <View style={{ marginLeft: 'auto', alignItems: 'flex-end' }}>
          <Text style={{ color: c.text, fontSize: 15, fontWeight: '700', fontFamily: fonts.mono }}>
            {expected[j].toFixed(1)}°
          </Text>
          {liveDeg && (
            <Text style={{
              fontSize: 11, fontFamily: fonts.mono, marginTop: 2,
              color: jointCalibrated(j) && Math.abs(liveDeg[j] - judgedExpected(j)) < (status?.tolerance_deg ?? 0.5) ? c.greenTx : c.dim,
            }}>
              {t('현재')} {liveDeg[j].toFixed(1)}°
            </Text>
          )}
        </View>
      </View>
      <Text style={{ color: c.muted, fontSize: 12, marginTop: 4, lineHeight: 17 }}>
        {j === 0 && rollRef === 'level' ? t('로봇 몸통이 수평인지 먼저 확인한 뒤, 수평계로 롤 조인트를 0°에 맞추세요') : jointGuide(leg, j, expected[j])}
      </Text>
    </View>
  );

  const [helpOpen, setHelpOpen] = useState(false);
  const [exitNotice, setExitNotice] = useState(false);
  const ranAny = Object.keys(ranGroups).length > 0;
  const fwRejected = !!ranGroups[`${leg}:roll`] && entry?.reason === 'fw_unsupported';
  const [rejectAck, setRejectAck] = useState<string | null>(null);
  const rejectOpen = fwRejected && (entry?.finished_at ?? '') !== rejectAck;
  const tooLarge = !!ranGroups[`${leg}:roll`] && entry?.reason === 'delta_too_large';
  const [tooLargeAck, setTooLargeAck] = useState<string | null>(null);
  const tooLargeOpen = tooLarge && (entry?.finished_at ?? '') !== tooLargeAck;
  const [autoOpen, setAutoOpen] = useState(false);
  const requestClose = () => { if (ranAny) setExitNotice(true); else onClose(); };
  const run = async (group: LegHomeGroup) => {
    if (!ip || running || starting || !legHomeCalibAllowed()) return;
    setSel(group);
    setStarting(true);
    const e = await startLeg(ip, leg, group, rollRef);
    setErr(e);
    if (e === null) setRanGroups((g) => ({ ...g, [`${leg}:${group}`]: true }));
    setStarting(false);
    refresh();
  };

  return (
    <Modal onClose={requestClose}>
      <View style={[styles.wizardBox, {
        backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg,
        width: Math.min(winW - 24, MODAL_MAX_W), maxHeight: winH - 24,
      }]}>
      <ScrollView contentContainerStyle={{ padding: 18 }}>
        <Text style={{ color: c.text, fontSize: 17, fontWeight: '700', marginBottom: 8 }}>
          {t('관절 홈포즈 세팅 — 다리별 치구 결합 & 자세 고정 이후 실행')}
        </Text>

        <Notice big tone="warn" text={t('지정된 관절 자세를 유지한 상태로 저장 버튼을 눌러주세요')} />

        <Notice big tone="info" text={sel === 'roll'
          ? (rollRef === 'level'
              ? t('로봇 몸통이 수평인지 먼저 확인한 뒤, 수평계로 롤 조인트를 0°에 맞추세요')
              : isRollNeutral(expected[0])
              ? t('몸통과 롤 조인트의 상대각도를 0도로 유지하세요')
              : t('롤 조인트를 그림과 같이 안쪽 리밋까지 당겨 위치시켜 주세요'))
          : `${t('무릎 조인트는 끝까지 접어주세요')}\n${t('피치 조인트는 고정치구를 꽂은 상태에서 끝까지 밀어 밀착시켜 주세요')}`} />

        {sel === 'pitch_knee' && (
          <Tappable onPress={() => setHelpOpen(true)}
            style={[styles.helpBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Icon name="wrench" size={16} color={c.accent2} />
            <Text style={{ color: c.accent2, fontSize: 14, fontWeight: '700' }}>{t('도움말 — 피치 축 고정 지그 설치 하는 법')}</Text>
          </Tappable>
        )}

        <View style={narrow ? { gap: 10 } : { flexDirection: 'row', gap: 10, alignItems: 'flex-start' }}>
          <View style={narrow
            ? { flexDirection: 'row', gap: 6, flexWrap: 'wrap' }
            : { width: LEG_COL_W, height: paneH, gap: 6 }}>
            {Array.from({ length: LEG_COUNT }, (_, i) => {
              const st = legs[i];
              const on = i === leg;
              const full = !!st?.joint_done?.every(Boolean);
              const failed = [0, 1, 2].some((j) => st?.joint_done?.[j] && !st?.joint_ok?.[j]);
              const busy = running && status?.leg === i;
              const label = doneLabel(st);
              return (
                <Tappable key={i} onPress={() => { setErr(null); setLeg(i); setSel('roll'); }}
                  style={[styles.legTab, narrow ? null : styles.legTabTall, {
                    borderRadius: radius.md,
                    backgroundColor: failed ? 'rgba(231,51,28,0.12)' : full ? 'rgba(63,185,80,0.14)'
                      : on ? 'rgba(77,156,245,0.14)' : c.elev,
                    borderColor: on ? c.accent2 : failed ? 'rgba(231,51,28,0.5)' : full ? 'rgba(63,185,80,0.6)' : c.line,
                  }]}>
                  <Text style={{ color: on ? c.text : c.muted, fontSize: 18, fontWeight: '800' }}>{`LEG${i}`}</Text>
                  <Text style={{ color: on ? c.text : c.muted, fontSize: 13.5, fontWeight: '600', textAlign: 'center' }}>
                    {t(LEG_INFO[i].label)}
                  </Text>
                  <Text numberOfLines={2}
                    style={{ color: busy ? c.amber : failed ? c.redbright : full ? c.greenTx : label ? c.accent2 : c.dim,
                             fontSize: 10.5, lineHeight: 13, fontWeight: '700', textAlign: 'center' }}>
                    {busy ? '…' : failed ? `✗ ${label ?? ''}`.trim() : label ?? '—'}
                  </Text>
                </Tappable>
              );
            })}
          </View>

          <View style={{ flexDirection: 'row', gap: 8 }}>
            <View style={narrow ? { flex: 1 } : { width: PANE_W }}>
              <LegHomePreview3D leg={leg} poseDeg={base} anim={anim}
                orbitRollDeg={expected[0]} orbitRef={orbit} badge={t('목표 동작')} height={paneH} />
            </View>
            <View style={narrow ? { flex: 1 } : { width: PANE_W }}>
              <LegHomePreview3D leg={leg} poseDeg={liveDeg ?? expected}
                orbitRollDeg={expected[0]} orbitRef={orbit} height={paneH}
                badge={liveDeg ? t('현재 자세') : t('현재 자세 — 관절각 수신 없음')} />
            </View>
          </View>

          <View style={{ flex: 1, minWidth: 0 }}>
            <Text style={{ color: c.text, fontSize: 14.5, fontWeight: '700', marginBottom: 3 }}>{legTitle(leg)}</Text>
            <Text style={{ color: c.dim, fontSize: 12, marginBottom: 8, lineHeight: 17 }}>
              {t('카드를 누르면 그 관절이 왼쪽 3D 에서 목표 자세까지 움직입니다. 자세를 만든 뒤 그 카드의 저장 버튼을 누르세요.')}
            </Text>

            <View style={{ gap: 6 }}>
              <Tappable onPress={() => setSel('roll')}
                style={[styles.card, styles.jointCard, { borderRadius: radius.sm }, groupStyle('roll')]}>
                {jointBody(0)}
                <View style={{ flexDirection: 'row', gap: 8, marginTop: 10 }}>
                  <RefRadio on={rollRef === 'limit'} name={t('리밋')} desc={t('안쪽 리밋에 갖다 대고 잡습니다')}
                    onPress={() => { setSel('roll'); setRollRef('limit'); }} />
                  <RefRadio on={rollRef === 'level'} name={t('수평계 0°')} desc={t('수평계로 0°에 맞춘 뒤 잡습니다')}
                    onPress={() => { setSel('roll'); setRollRef('level'); }} />
                </View>
                {groupButton('roll')}
              </Tappable>

              <Tappable onPress={() => setSel('pitch_knee')}
                style={[styles.card, styles.jointCard, { borderRadius: radius.sm, gap: 8 }, groupStyle('pitch_knee')]}>
                {jointBody(1)}
                <View style={{ height: 1, backgroundColor: c.line2 }} />
                {jointBody(2)}
                {groupButton('pitch_knee')}
              </Tappable>
            </View>
          </View>
        </View>

        <View style={{ flexDirection: 'row', marginTop: 12 }}>
          <Tappable onPress={requestClose}
            style={[styles.secondary, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
          </Tappable>
        </View>

        {guard.blocked && !boardsDead && (
          <Notice tone="warn" text={`⛔ ${guard.reason}`} />
        )}

        {entry?.reason === 'not_executed' && (
          <Notice tone="warn" text={t('로봇이 명령을 실행하지 않았습니다 — 영점은 기록되지 않았습니다. 다시 실행하세요.')} />
        )}
        {fwRejected && (
          <Notice tone="warn" text={t('이 로봇의 모터 펌웨어에는 홈 오프셋 보정 기능이 없어 수평계 0° 보정이 거부됐습니다 — 영점은 바뀌지 않았습니다.')} />
        )}

        {boardsDead && (
          <Notice tone="warn" text={t('모터 보드와 통신이 없습니다 — 48V 구동(LEGS) 전원을 켠 뒤 다시 실행하세요.')} />
        )}

        {savedJoints.length > 0 && !runningThisLeg && (
          <View style={[styles.card, {
            marginTop: 10, borderRadius: radius.md,
            backgroundColor: savedOk ? 'rgba(63,185,80,0.10)' : 'rgba(231,51,28,0.10)',
            borderColor: savedOk ? 'rgba(63,185,80,0.5)' : 'rgba(231,51,28,0.5)',
          }]}>
            <Text style={{ color: savedOk ? c.greenTx : c.redbright, fontSize: 12, fontWeight: '800' }}>
              {savedOk
                ? (savedName === 'roll' && entry?.roll_ref === 'level'
                    ? t('✓ LEG{n} roll 영점 저장 완료 — 수평계 0° 기준으로 기록됐습니다. (자세가 맞았는지는 확인하지 못합니다 — 수평계를 믿는 절차입니다)')
                    : t('✓ LEG{n} {g} 영점 저장 완료 — 홈각으로 기록됐습니다. (자세가 맞았는지는 확인하지 못합니다 — 치구를 믿는 절차입니다)'))
                    .replace('{n}', String(leg)).replace('{g}', savedName)
                : entry?.reason === 'can_check_failed'
                  ? t('✗ LEG{n} 실패 — 모터 보드와 통신이 없습니다. 48V 구동(LEGS) 전원을 켜고 다시 실행하세요. 영점은 기록되지 않았습니다.').replace('{n}', String(leg))
                  : t('✗ LEG{n} {g} 실패 — 영점이 기대 홈각으로 기록되지 않았습니다.')
                      .replace('{n}', String(leg)).replace('{g}', savedName)}
            </Text>
            {!savedOk && entry?.reason !== 'can_check_failed' && (
              <>
                <View style={{ marginTop: 6, gap: 2 }}>
                  {savedJoints.map((j) => {
                    const measured = entry?.measured_deg?.[j];
                    const exp = judgedExpected(j);
                    const diff = measured != null ? measured - exp : null;
                    return (
                      <Text key={j} style={{ color: c.muted, fontSize: 10.5, fontFamily: fonts.mono }}>
                        {JOINT_LABELS[j].padEnd(6, ' ')} {measured?.toFixed(2) ?? '—'}°
                        {'  '}({t('기대')} {exp.toFixed(2)}°, {t('오차')} {diff != null ? `${diff >= 0 ? '+' : ''}${diff.toFixed(2)}` : '—'}°)
                      </Text>
                    );
                  })}
                </View>
                <Text style={{ color: c.dim, fontSize: 9.5, marginTop: 5 }}>
                  {t('허용 오차 ±{n}°').replace('{n}', String(status?.tolerance_deg ?? 0.5))}
                </Text>
              </>
            )}
          </View>
        )}

        {allDone && <Notice tone="ok" text={t('네 다리 모두 완료되었습니다.')} />}
        {err && <Text style={{ color: c.redbright, fontSize: 11, marginTop: 10 }}>{err}</Text>}

        {helpOpen && <JigHelpModal onClose={() => setHelpOpen(false)} />}

        {rejectOpen && (
          <Modal onClose={() => {}} dismissable={false}>
            <View style={[styles.introBox, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: 460 }]}>
              <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{t('수평계 0° 홈 보정을 할 수 없습니다')}</Text>
              <Notice big tone="warn" text={t('이 로봇의 모터 펌웨어에는 홈 오프셋 보정 기능이 없습니다. 모터 펌웨어를 업데이트한 뒤 다시 실행하세요. 지금은 리밋 기준으로 홈을 잡을 수 있습니다.')} />
              <View style={{ flexDirection: 'row', marginTop: 14, justifyContent: 'center' }}>
                <Tappable onPress={() => setRejectAck(entry?.finished_at ?? '')}
                  style={[styles.modalBtn, { backgroundColor: c.accent, borderColor: 'transparent', borderRadius: radius.md, flex: 0, minWidth: 260 }]}>
                  <Text style={{ color: c.onAccent, fontSize: 14, fontWeight: '700' }}>{t('확인')}</Text>
                </Tappable>
              </View>
            </View>
          </Modal>
        )}

        {tooLargeOpen && (
          <Modal onClose={() => {}} dismissable={false}>
            <View style={[styles.introBox, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: 460 }]}>
              <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{t('수평계 0° 보정이 거부됐습니다')}</Text>
              <Notice big tone="warn" text={t('기준점과의 차이가 큽니다. 리밋에 대고 1차로 홈을 잡아주세요.')} />
              <View style={{ flexDirection: 'row', marginTop: 14, justifyContent: 'center' }}>
                <Tappable onPress={() => setTooLargeAck(entry?.finished_at ?? '')}
                  style={[styles.modalBtn, { backgroundColor: c.accent, borderColor: 'transparent', borderRadius: radius.md, flex: 0, minWidth: 260 }]}>
                  <Text style={{ color: c.onAccent, fontSize: 14, fontWeight: '700' }}>{t('확인')}</Text>
                </Tappable>
              </View>
            </View>
          </Modal>
        )}

        {exitNotice && (
          <Modal onClose={() => {}} dismissable={false}>
            <View style={[styles.introBox, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: 460 }]}>
              <Text style={{ color: c.text, fontSize: 16, fontWeight: '700' }}>{t('제어가 비활성화되었습니다')}</Text>
              <Notice big tone="warn" text={t('홈을 잡은 관절은 BOARD_RESET 으로 제어가 꺼졌습니다. 자동 기동 시퀀스를 다시 실행하세요.')} />
              <View style={{ flexDirection: 'row', marginTop: 14, gap: 10 }}>
                <Tappable onPress={() => { setExitNotice(false); onClose(); }}
                  style={[styles.modalBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                  <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('확인')}</Text>
                </Tappable>
                <Tappable onPress={() => { setExitNotice(false); setAutoOpen(true); }}
                  style={[styles.modalBtn, { backgroundColor: c.accent, borderColor: 'transparent', borderRadius: radius.md }]}>
                  <Text style={{ color: c.onAccent, fontSize: 14, fontWeight: '700' }}>{t('자동 기동')}</Text>
                </Tappable>
              </View>
            </View>
          </Modal>
        )}
        {autoOpen && <AutoStartModal onClose={() => { setAutoOpen(false); onClose(); }} />}

      </ScrollView>
      </View>
    </Modal>
  );
}

const JIG_HELP_PHOTOS: number[] = [];

export function LegHomeSetFlow({ onClose }: { onClose: () => void }) {
  const connected = useRobot((s) => s.conn === 'connected');
  const { status, refresh } = useLegHomeSet(useRobot((s) => s.ip), connected);
  const guard = useLegHomeGuard();
  const boardsDead = status?.boards_alive === false;
  const [step, setStep] = useState<'intro' | 'wizard'>('intro');
  const blocked = guard.blocked || boardsDead;
  const blockReason = boardsDead
    ? t('모터 보드와 통신이 없습니다 — 48V 구동(LEGS) 전원을 켠 뒤 다시 실행하세요.')
    : guard.reason;
  return (
    <LegibleText.Provider value={false}>
      {step === 'intro'
        ? <HomeSetIntroModal blocked={blocked} blockReason={blockReason} onClose={onClose} onRun={() => setStep('wizard')} />
        : <LegHomeWizardModal status={status} refresh={refresh} onClose={onClose} />}
    </LegibleText.Provider>
  );
}

function JigHelpModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const { width: winW } = useWindowDimensions();
  const narrow = useCompactW();
  return (
    <Modal onClose={onClose}>
      <View style={[styles.wizardBox, {
        backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg,
        width: Math.min(winW - 24, MODAL_MAX_W),
      }]}>
        <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 12 }}>
          {t('피치 축 고정 지그 설치 하는 법')}
        </Text>

        <View style={narrow ? { gap: 10 } : { flexDirection: 'row', gap: 12 }}>
          {[0, 1].map((i) => (
            <View key={i} style={[styles.photo, {
              height: narrow ? 220 : 420,
              backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md,
            }]}>
              {JIG_HELP_PHOTOS[i] != null
                ? <Image source={JIG_HELP_PHOTOS[i]} style={StyleSheet.absoluteFill} resizeMode="contain" />
                : <Text style={{ color: c.dim, fontSize: 12 }}>{t('사진 준비 중')}</Text>}
            </View>
          ))}
        </View>

        <View style={{ flexDirection: 'row', marginTop: 14, justifyContent: 'center' }}>
          <Tappable onPress={onClose}
            style={[styles.secondary, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, minWidth: 420 }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('닫기')}</Text>
          </Tappable>
        </View>
      </View>
    </Modal>
  );
}

function PollNote({ connected, pollErr, status }: {
  connected: boolean; pollErr: string | null; status: unknown;
}) {
  const { c } = useTheme();
  const lg = useLegible();
  if (!connected) {
    return <Text style={{ color: lg ? c.muted : c.dim, fontSize: lg ? 12 : 10.5, marginTop: 10 }}>{t('로봇에 연결되어야 사용할 수 있습니다.')}</Text>;
  }
  if (!pollErr || status) return null;
  return <Text style={{ color: c.amber, fontSize: lg ? 12 : 10.5, marginTop: 10 }}>{t('상태 조회 실패')} — {pollErr}</Text>;
}

function Notice({ tone, text, big }: { tone: 'warn' | 'ok' | 'info'; text: string; big?: boolean }) {
  const { c, radius } = useTheme();
  const lg = useLegible();
  const skin = tone === 'warn'
    ? { bg: 'rgba(210,153,34,0.10)', line: 'rgba(210,153,34,0.5)', ic: c.amber, tx: c.amber, icon: 'warn' as const }
    : tone === 'ok'
      ? { bg: 'rgba(63,185,80,0.10)', line: 'rgba(63,185,80,0.5)', ic: c.green, tx: c.greenTx, icon: 'stand' as const }
      : { bg: 'rgba(77,156,245,0.10)', line: 'rgba(77,156,245,0.5)', ic: c.accent, tx: c.accent2, icon: 'wrench' as const };
  return (
    <View style={[styles.card, {
      marginTop: 8, borderRadius: radius.md,
      backgroundColor: skin.bg, borderColor: skin.line,
    }]}>
      <View style={{ flexDirection: 'row', gap: 8, alignItems: 'flex-start' }}>
        <Icon name={skin.icon} size={big ? 17 : 13} color={skin.ic} />
        <Text style={{ color: skin.tx, fontSize: big ? 15 : lg ? 12 : 11, lineHeight: big ? 22 : 16,
                       fontWeight: big ? '700' : '400', flex: 1 }}>{text}</Text>
      </View>
    </View>
  );
}

function RefRadio({ on, name, desc, onPress }: { on: boolean; name: string; desc: string; onPress: () => void }) {
  const { c, radius } = useTheme();
  const { c: rc } = useRb();
  return (
    <Tappable onPress={onPress} accessibilityLabel={name}
      style={[styles.refRadio, { borderRadius: radius.sm, borderWidth: on ? 2 : 1,
        borderColor: on ? rc('border-brand') : c.line, backgroundColor: on ? rc('fill-brand-subtlest') : c.panel }]}>
      <View style={{ flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 6 }}>
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
          <View style={[styles.radio, { borderColor: on ? rc('border-brand') : c.muted }]}>
            {on && <View style={[styles.radioDot, { backgroundColor: rc('fill-brand') }]} />}
          </View>
          <Text style={{ color: on ? c.text : c.muted, fontSize: 15, fontWeight: '700' }}>{name}</Text>
        </View>
        {on && <RBBadge type="capsule" color="blue">{t('✓ 선택됨')}</RBBadge>}
      </View>
      <Text style={{ color: on ? c.muted : c.dim, fontSize: 12, marginTop: 4 }}>{desc}</Text>
    </Tappable>
  );
}

const styles = StyleSheet.create({
  card: { borderWidth: 1, paddingHorizontal: 12, paddingVertical: 10 },
  trow: { flexDirection: 'row', alignItems: 'center', paddingVertical: 12, borderTopWidth: 1 },
  runRow: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  action: { alignItems: 'center', justifyContent: 'center', height: 42, minWidth: 112, paddingHorizontal: 18, borderWidth: 1 },
  introBox: { maxWidth: '92%', borderWidth: 1, padding: 18 },
  wizardBox: { maxWidth: '94%', borderWidth: 1, overflow: 'hidden' },
  figure: { borderWidth: 1, alignItems: 'center', justifyContent: 'center', paddingVertical: 10 },
  modalBtn: { flex: 1, height: 40, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  primary: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 8, height: 48, paddingHorizontal: 22, borderWidth: 1, minWidth: 260 },
  secondary: { alignItems: 'center', justifyContent: 'center', height: 48, paddingHorizontal: 20, borderWidth: 1 },
  legTab: { alignItems: 'center', gap: 2, minWidth: 84, paddingHorizontal: 6, paddingVertical: 5, borderWidth: 1 },
  legTabTall: { flex: 1, justifyContent: 'center' },
  helpBtn: { flexDirection: 'row', alignItems: 'center', gap: 8, alignSelf: 'flex-start', marginTop: 8, height: 40, paddingHorizontal: 16, borderWidth: 1 },
  photo: { flex: 1, borderWidth: 1, alignItems: 'center', justifyContent: 'center', overflow: 'hidden' },
  jointCard: { paddingVertical: 8 },
  cardBtn: { marginTop: 10, minWidth: 0, alignSelf: 'stretch' },
  refRadio: { flex: 1, paddingVertical: 8, paddingHorizontal: 10 },
  radio: { width: 18, height: 18, borderRadius: 9, borderWidth: 2, alignItems: 'center', justifyContent: 'center' },
  radioDot: { width: 8, height: 8, borderRadius: 4 },
});
