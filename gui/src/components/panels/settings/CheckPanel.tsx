import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { CalibFixButton } from '@/components/CalibFixButton';
import { AccCalibModal } from '@/components/AccCalibModal';
import { GyroBiasModal, GyroCalibHint } from '@/components/GyroBiasModal';
import { ImuNullModal } from '@/components/ImuNullModal';
import { ImuLevelHint } from '@/components/BubbleLevel';
import { SitRobotHint } from '@/components/SitRobotHint';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { rest } from '@/lib/rest';
import { connection } from '@/lib/connection';
import { PROGRAM, QUADWALK_CMD } from '@/lib/robotState';
import { GAIT_CHECK, JOINT_COMM, type JointCommCheckInfo } from '@/types/robot';
import { LEG_INFO } from '@/lib/legHomeCalib';
import { LegHomeSetFlow } from '@/components/LegHomeCalibration';
import { VisualCheckModal } from '@/components/VisualCheckModal';
import { CameraBasicCheckModal, DepthOnChipModal, DepthExtrinsicModal } from '@/components/CameraCheckModals';
import { ZmpCalibModal } from '@/components/ZmpCalibModal';
import {
  useSitImuCalibGuard, sitImuCalibAllowed, useCalibGuard, calibAllowed, commissioning, type CalibGuard,
  useZmpCalibGuard, zmpCalibAllowed, useLegHomeGuard,
} from '@/lib/commissioning';
import {
  useSampleAverage, pickImuStill, pickLevel, IMU_STILL_MS, LEVEL_MS,
  ACC_NORM_MIN, ACC_NORM_MAX, GYRO_BIAS_MAX_DPS, LEVEL_MAX_DEG, type SampleState,
} from '@/lib/gaitCheck';
import type { ConfirmReq } from '@/components/panels/PowerPanel';
import { H2, Desc, LegibleText } from './common';
import { t } from '@/lib/i18n';
import { robotRunView } from '@/lib/robotRun';

type Verdict = 'wait' | 'measuring' | 'ok' | 'warn' | 'info' | 'fail';

const CAMERA_STAGES = [
  { title: '카메라 시야 · FPS', rx: '카메라 기본 검사', modal: 'basic',
    measure: '측정 — 카메라 화면을 띄워 시야를 눈으로 보고 FPS 를 잽니다',
    judge: '판정 — FPS 가 나오지 않으면 카메라 점검이 필요합니다' },
  { title: 'depth 카메라 품질', rx: '하단 depth 카메라 On-chip 캘리브레이션', modal: 'onchip',
    measure: '측정 — 하단 depth 카메라마다 depth 품질 수치를 잽니다',
    judge: '판정 — 수치가 기준보다 낮으면 On-chip 캘리브레이션을 합니다' },
  { title: 'depth 카메라 위치 정합', rx: '하단 depth 카메라 위치 보정', modal: 'extrinsic',
    measure: '측정 — 평평한 바닥에서 하단 depth 카메라들의 점군이 한 평면에 맞는지 봅니다',
    judge: '판정 — 평면과 점 사이 거리(RMS)가 크면 위치 보정을 합니다' },
] as const;

const calibReq = (nm: string, msg: string, extra: React.ReactNode, allowed: () => boolean, fire: () => void): ConfirmReq =>
  ({ title: t('보정을 실행할까요?'), message: `${nm} — ${msg}`, confirmLabel: t('실행'), danger: true,
     run: () => { if (allowed()) fire(); }, extra });

export function CheckPanel() {
  const { c } = useTheme();
  const [confirmReq, setConfirmReq] = useState<ConfirmReq | null>(null);
  return (
    <>
      <LegibleText.Provider value>
        <H2>{t('보행 점검 ')}<Text style={{ color: c.muted, fontSize: 11 }}>Gait Check</Text></H2>
        <Desc>{t('앉은 로봇에서 시작해 서고, 제자리 걸음까지 순서대로 점검합니다. 단계마다 측정한 뒤 이상이 있으면 그 자리에서 보정하고 다음으로 갑니다.')}</Desc>
        <View style={styles.stages}>
          <ImuStage request={setConfirmReq} />
          <DqStage />
          <LevelStage request={setConfirmReq} />
          <CommStage request={setConfirmReq} />
          <DriftStage request={setConfirmReq} />
        </View>

        <View style={{ height: 28 }} />
        <H2>{t('카메라 점검 ')}<Text style={{ color: c.muted, fontSize: 11 }}>Camera Check</Text></H2>
        <Desc>{t('카메라 시야와 FPS, 하단 depth 카메라의 품질과 위치를 점검합니다. 측정·판정은 준비 중이고, 보정 버튼은 캘리브레이션 탭과 같습니다.')}</Desc>
        <View style={styles.stages}>
          {CAMERA_STAGES.map((s, i) => <CameraStage key={s.modal} no={i + 1} stage={s} />)}
        </View>
      </LegibleText.Provider>
      {confirmReq && (
        <ConfirmModal title={confirmReq.title} message={confirmReq.message} confirmLabel={confirmReq.confirmLabel}
          danger={confirmReq.danger} skipKey={confirmReq.skipKey}
          onConfirm={() => { confirmReq.run(); setConfirmReq(null); }} onClose={() => setConfirmReq(null)}>
          {confirmReq.extra}
        </ConfirmModal>
      )}
    </>
  );
}

function ImuStage({ request }: { request: (r: ConfirmReq) => void }) {
  const guard = useSitImuCalibGuard();
  const { state, start } = useSampleAverage(IMU_STILL_MS, pickImuStill);
  const [accOpen, setAccOpen] = useState(false);
  const [gyroOpen, setGyroOpen] = useState(false);
  const m = state.phase === 'done' ? state.mean : null;
  const accNorm = m ? m[0] : 0;
  const gyroDps = m ? m.slice(1).map(Math.abs) : [];
  const accOk = accNorm >= ACC_NORM_MIN && accNorm <= ACC_NORM_MAX;
  const gyroOk = gyroDps.every((v) => v < GYRO_BIAS_MAX_DPS);
  const f2 = (v: number) => v.toFixed(2);
  return (
    <>
      <StageCard no={1} title={t('앉음 · IMU')} verdict={verdictOf(state, accOk && gyroOk)} active
        cond={<GuardLine guard={guard} ok={t('앉음 · IMU 연결됨 — 측정할 수 있습니다')} />}>
        <View style={styles.cells}>
          <Cell label={t('중력 크기')} value={m ? `${f2(accNorm)} m/s²` : '—'}
            basis={t('기준 {a} ~ {b}').replace('{a}', String(ACC_NORM_MIN)).replace('{b}', String(ACC_NORM_MAX))}
            tone={m ? (accOk ? 'ok' : 'warn') : undefined} />
          <Cell label={t('자이로 편차 x · y · z')} value={m ? `${gyroDps.map(f2).join(' · ')} °/s` : '—'}
            basis={t('기준 {v} °/s 미만').replace('{v}', String(GYRO_BIAS_MAX_DPS))}
            tone={m ? (gyroOk ? 'ok' : 'warn') : undefined} />
        </View>
        <View style={styles.foot}>
          {guard.blocked && guard.fix && <CalibFixButton fix={guard.fix} />}
          <RunButton label={measureLabel(state)} disabled={guard.blocked || state.phase === 'measuring'} onPress={start} />
          <RxButton label={t('가속도계 보정')} enabled={!guard.blocked}
            onPress={() => request(calibReq(t('가속도계 보정 (ACC)'),
              t('로봇이 앉은 상태에서만 실행할 수 있습니다. 2초 동안 중력 크기를 평균 내 보정값을 저장합니다 — 그동안 로봇을 건드리지 마세요.'),
              <SitRobotHint />, sitImuCalibAllowed, () => setAccOpen(true)))} />
          <RxButton label={t('자이로 바이어스 보정')} enabled={!guard.blocked}
            onPress={() => request(calibReq(t('자이로 바이어스 보정'),
              t('로봇이 앉은 상태에서만 실행할 수 있습니다. 실행하면 IMU 를 리셋하고 값이 자리 잡기를 기다린 뒤 자이로 값을 다시 측정합니다.'),
              <GyroCalibHint />, sitImuCalibAllowed, () => setGyroOpen(true)))} />
          <FootNote state={state} text={t('3초 평균 · 기준을 벗어나면 주의로 표시합니다')} />
        </View>
      </StageCard>
      {accOpen && <AccCalibModal onClose={() => setAccOpen(false)} />}
      {gyroOpen && <GyroBiasModal onClose={() => setGyroOpen(false)} />}
    </>
  );
}

function DqStage() {
  const { c, radius } = useTheme();
  return (
    <StageCard no={2} title={t('모터 캘리브레이션 상태')} verdict="wait" active
      cond={<Text style={{ color: c.muted, fontSize: 12 }}>{t('앉은 상태에서 측정')}</Text>}>
      <View style={[styles.result, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
        <Text style={{ color: c.muted, fontSize: 12.5, lineHeight: 18 }}>
          {t('모터 캘리브레이션 상태 측정은 펌웨어 지원 후 연결됩니다 — 결과가 여기에 표시됩니다.')}
        </Text>
      </View>
      <View style={styles.foot}>
        <RunButton label={t('측정')} disabled onPress={() => {}} />
        <RxButton label={t('모터 캘리브레이션')} enabled={false} onPress={() => {}} />
      </View>
    </StageCard>
  );
}

function useStandGuard(): CalibGuard {
  const calib = useCalibGuard();
  const robot = useTelemetry((s) => s.robot);
  if (calib.blocked || robot?.isStanding) return calib;
  return { blocked: true, reason: t('앉아 있음 — 세운 뒤 측정하세요'), fix: robot?.gaitId === -1 ? 'autostart' : 'stand' };
}

function LevelStage({ request }: { request: (r: ConfirmReq) => void }) {
  const { c } = useTheme();
  const guard = useStandGuard();
  const { state, start } = useSampleAverage(LEVEL_MS, pickLevel);
  const [imuOpen, setImuOpen] = useState(false);
  const m = state.phase === 'done' ? state.mean : null;
  const rollOk = !!m && Math.abs(m[0]) <= LEVEL_MAX_DEG;
  const pitchOk = !!m && Math.abs(m[1]) <= LEVEL_MAX_DEG;
  const deg = (v: number) => `${v >= 0 ? '+' : ''}${v.toFixed(2)}°`;
  const basis = t('기준 ±{v}°').replace('{v}', String(LEVEL_MAX_DEG));
  return (
    <>
      <StageCard no={3} title={t('서기 · 수평')} verdict={verdictOf(state, rollOk && pitchOk)} active
        cond={<GuardLine guard={guard} ok={t('서 있음 — 측정할 수 있습니다')} />}>
        <Text style={{ color: c.muted, fontSize: 12, lineHeight: 16 }}>
          {t('몸통 위에 물방울 수평계를 올리고, 기포가 중심에 오도록 손으로 몸통 각도를 직접 맞춘 뒤 측정하세요. 이때 롤·피치가 0°가 아니면 IMU 영점이 틀어진 것입니다.')}
        </Text>
        <View style={styles.cells}>
          <Cell label="Roll" value={m ? deg(m[0]) : '—'} basis={basis} tone={m ? (rollOk ? 'ok' : 'warn') : undefined} />
          <Cell label="Pitch" value={m ? deg(m[1]) : '—'} basis={basis} tone={m ? (pitchOk ? 'ok' : 'warn') : undefined} />
        </View>
        <View style={styles.foot}>
          {guard.blocked && guard.fix && <CalibFixButton fix={guard.fix} />}
          <RunButton label={measureLabel(state)} disabled={guard.blocked || state.phase === 'measuring'} onPress={start} />
          <RxButton label={t('IMU 롤/피치 영점')} enabled={!guard.blocked}
            onPress={() => {
              commissioning.standIfStanding();
              request(calibReq(t('IMU 롤/피치 영점'),
                t('지금 자세가 수평 기준이 됩니다. 기울어진 상태로 실행하면 이후 제어가 전부 틀어집니다.'),
                <ImuLevelHint />, calibAllowed, () => setImuOpen(true)));
            }} />
          <FootNote state={state} text={t('2초 평균 · ±0.3°를 넘으면 주의로 표시합니다')} />
        </View>
      </StageCard>
      {imuOpen && <ImuNullModal onClose={() => setImuOpen(false)} />}
    </>
  );
}

function useRobotRun<T extends { run: number; state: number }>(info: T | undefined, active: number[], doneState: number, failedState: number) {
  const infoRef = useRef(info);
  infoRef.current = info;
  const [startRun, setStartRun] = useState<number | null>(null);
  const [noReply, setNoReply] = useState(false);
  const seenRef = useRef<T | undefined>(undefined);
  const { cur, shown } = robotRunView(info, startRun, seenRef.current, doneState);
  seenRef.current = cur;
  const running = startRun != null && !noReply && (!cur || active.includes(cur.state));
  useEffect(() => {
    if (startRun == null || cur) return;
    const id = setTimeout(() => setNoReply(true), 5000);
    return () => clearTimeout(id);
  }, [startRun, cur]);
  const failed = noReply || cur?.state === failedState;
  const begin = (send: () => void) => { setNoReply(false); seenRef.current = undefined; setStartRun(infoRef.current?.run ?? 0); send(); };
  return { cur, running, noReply, failed, shown, done: shown?.state === doneState, begin };
}

const standAllowed = () => calibAllowed() && !!useTelemetry.getState().robot?.isStanding;

const COMM_AMP_DEG = 30;
const JOINT_NAMES = ['롤', '피치', '무릎'];
const LEG_READ_ORDER = [2, 3, 0, 1];
function CommStage({ request }: { request: (r: ConfirmReq) => void }) {
  const { c } = useTheme();
  const ip = useRobot((s) => s.ip);
  const jc = useRobot((s) => s.robot?.joint_comm_check);
  const guard = useStandGuard();
  const run = useRobotRun(jc, [JOINT_COMM.running], JOINT_COMM.done, JOINT_COMM.failed);
  const r: JointCommCheckInfo | undefined = run.shown;
  const jointBad = (i: number) => !!r && r.measured && (r.rx_hz_min[i] < r.hz_warn || r.gap_max_ms[i] > r.gap_warn_ms);
  const allOk = !!r && r.measured && r.rx_hz_min.every((_, i) => !jointBad(i));
  const badLegs = LEG_READ_ORDER.filter((li) => [0, 1, 2].some((ji) => jointBad(li * 3 + ji)))
    .map((li) => t(LEG_INFO[li].label));
  const verdict: Verdict = run.running ? 'measuring' : run.failed ? 'fail'
    : run.done ? (!r?.measured ? 'info' : allOk ? 'ok' : 'warn') : 'wait';

  const ask = () => request({
    title: t('관절 통신 점검을 시작할까요?'),
    message: t('서 있는 채 몸통을 좌우로 최대 {a}° 비틀기를 2초 주기로 5번(약 10초) 합니다. 다리 주변을 비우세요. 점검 중에는 조이스틱 입력이 무시됩니다.').replace('{a}', String(COMM_AMP_DEG)),
    confirmLabel: t('시작'), danger: true,
    run: () => { if (standAllowed()) run.begin(() => { rest.commandStruct(ip, PROGRAM.QuadWalk, QUADWALK_CMD.JOINT_COMM_CHECK, { float: [COMM_AMP_DEG] }).catch(() => {}); }); },
  });
  const failText = run.noReply ? t('로봇이 응답하지 않습니다 — 로봇 소프트웨어가 이 점검을 지원하는지 확인하세요')
    : run.cur?.reason === 1 ? t('서 있는 상태(모델 보행)에서만 시작할 수 있습니다')
    : run.cur?.reason === 2 ? t('로봇이 움직이고 있어 시작하지 않았습니다 — 멈춘 뒤 다시 측정하세요')
    : t('중지되었습니다 — 다시 측정하세요');
  const verdictNote: { text: string; ok: boolean } | null = !r || !r.measured || run.running || run.failed ? null
    : badLegs.length ? { ok: false, text: t('로봇 몸통과 [{legs}] 다리 모듈 사이의 배선을 점검하세요').replace('{legs}', badLegs.join(', ')) }
    : { ok: true, text: t('정상입니다') };
  const note = run.failed ? failText
    : r && !r.measured ? t('이 로봇(시뮬 등)은 CAN 통신 계측이 없어 판정하지 않습니다')
    : t('관절별 최저 수신 주파수와 최대 수신 간격을 봅니다 — Motion 의 CAN 경고와 같은 기준');
  const mch = r?.measured ? r.motor_ch : undefined;
  const channels = mch && r?.ch_err_frames && r.ch_warning && r.ch_passive && r.ch_bus_off ? [...new Set(mch)].sort((a, b) => a - b) : [];
  const chLegs = (ch: number) => LEG_INFO.filter((_, li) => [0, 1, 2].some((ji) => mch![li * 3 + ji] === ch)).map((l) => l.name).join(' · ');
  const criteria = r?.measured
    ? ` ${t('기준: 수신 {hz} Hz 이상 · 수신 간격 {ms} ms 이하')
        .replace('{hz}', String(Math.round(r.hz_warn))).replace('{ms}', String(r.gap_warn_ms))}`
    : '';
  return (
    <StageCard no={4} title={t('관절 통신 상태')} verdict={verdict} active
      cond={run.running ? <Text style={{ color: c.accent2, fontSize: 12 }}>{t('트위스트 중 {p}%').replace('{p}', String(run.cur?.percent ?? 0))}</Text>
        : <GuardLine guard={guard} ok={t('서 있음 — 측정할 수 있습니다')} />}>
      <Text style={{ color: c.muted, fontSize: 12, lineHeight: 16 }}>
        {t('서 있는 채 몸통을 좌우로 크게 비트는 동안 CAN 통신을 봅니다. 모터별은 빠진 프레임 수와 최저 수신 주파수 · 최대 수신 간격, 채널별은 CAN 채널이 상태 변화를 알린 횟수입니다.')}
        {criteria}
      </Text>
      <View style={{ gap: 6 }}>
        {channels.length > 0 && <Text style={{ color: c.muted, fontSize: 12, fontWeight: '700' }}>{t('모터별')}</Text>}
        {LEG_INFO.map((leg, li) => (
          <View key={leg.name} style={[styles.cells, { alignItems: 'center' }]}>
            <Text style={{ color: c.text, fontSize: 12, fontWeight: '700', width: 30 }}>{leg.name}</Text>
            {JOINT_NAMES.map((jn, ji) => {
              const i = li * 3 + ji;
              const has = !!r && r.measured;
              const missed = r?.missed?.[i];
              return (
                <Cell key={jn} label={t(jn)}
                  value={has ? (missed == null ? '—' : t('누락 {n}').replace('{n}', String(missed))) : '—'}
                  basis={has ? t('최저 {hz} Hz · 최대 {ms} ms').replace('{hz}', String(Math.round(r!.rx_hz_min[i])))
                    .replace('{ms}', r!.gap_max_ms[i].toFixed(1)) : ''}
                  tone={has ? (jointBad(i) ? 'warn' : missed === 0 ? 'ok' : undefined) : undefined} />
              );
            })}
          </View>
        ))}
        {channels.length > 0 && <Text style={{ color: c.muted, fontSize: 12, fontWeight: '700', marginTop: 4 }}>{t('채널별')}</Text>}
        {channels.map((ch) => {
          const cell = (label: string, n: number, basis = '') =>
            <Cell label={label} value={String(n)} basis={basis} tone={n === 0 ? 'ok' : undefined} />;
          return (
            <View key={ch} style={[styles.cells, { alignItems: 'center' }]}>
              <View style={{ width: 70 }}>
                <Text style={{ color: c.text, fontSize: 12, fontWeight: '700' }}>CH{ch}</Text>
                <Text style={{ color: c.muted, fontSize: 10.5 }}>{chLegs(ch)}</Text>
              </View>
              {cell(t('에러 프레임'), r!.ch_err_frames![ch], t('상태 변화 알림 수'))}
              {cell('error-warning', r!.ch_warning![ch])}
              {cell('error-passive', r!.ch_passive![ch])}
              {cell('bus-off', r!.ch_bus_off![ch])}
            </View>
          );
        })}
      </View>
      {run.running && <ProgressLine percent={run.cur?.percent ?? 0} />}
      <View style={styles.foot}>
        {!run.running && guard.blocked && guard.fix && <CalibFixButton fix={guard.fix} />}
        {run.running
          ? <RunButton label={t('중지')} disabled={false} onPress={() => connection.sendMotion('stand')} />
          : <RunButton label={t('측정')} disabled={guard.blocked} onPress={ask} />}
        {verdictNote
          ? <Text style={{ color: verdictNote.ok ? c.greenTx : c.amberTx, fontSize: 13, fontWeight: '700', flex: 1, minWidth: 160 }}>
              {verdictNote.ok ? '✓ ' : '⚠ '}{verdictNote.text}</Text>
          : <Text style={{ color: run.failed ? c.amberTx : c.muted, fontSize: 11.5, flex: 1, minWidth: 160 }}>{note}</Text>}
      </View>
    </StageCard>
  );
}

const DRIFT_SEC = 10;
const DRIFT_FB_WARN_CM = 10;
const DRIFT_LR_WARN_CM = 5;
function DriftStage({ request }: { request: (r: ConfirmReq) => void }) {
  const { c } = useTheme();
  const ip = useRobot((s) => s.ip);
  const gcAny = useRobot((s) => s.robot?.gait_check);
  const gc = gcAny && (gcAny.mode ?? 0) === 0 ? gcAny : undefined;
  const guard = useStandGuard();
  const homeGuard = useLegHomeGuard();
  const zmpGuard = useZmpCalibGuard();
  const [visualOpen, setVisualOpen] = useState(false);
  const [homeOpen, setHomeOpen] = useState(false);
  const [zmpOpen, setZmpOpen] = useState(false);
  const { cur, running, noReply, failed, shown, done, begin } =
    useRobotRun(gc, [GAIT_CHECK.entering, GAIT_CHECK.walking, GAIT_CHECK.stopping], GAIT_CHECK.done, GAIT_CHECK.failed);
  const overFb = !!shown && Math.abs(shown.x_mm) >= DRIFT_FB_WARN_CM * 10;
  const overLr = !!shown && Math.abs(shown.y_mm) >= DRIFT_LR_WARN_CM * 10;
  const over = overFb || overLr;
  const drifted = [
    overFb && t('앞뒤 {v} cm 이상').replace('{v}', String(DRIFT_FB_WARN_CM)),
    overLr && t('좌우 {v} cm 이상').replace('{v}', String(DRIFT_LR_WARN_CM)),
  ].filter(Boolean).join(' · ');
  const verdict: Verdict = running ? 'measuring' : failed ? 'fail' : done ? (over ? 'warn' : 'ok') : 'wait';

  const start = () => begin(() => {
    rest.commandStruct(ip, PROGRAM.QuadWalk, QUADWALK_CMD.GAIT_CHECK_TROT, { float: [DRIFT_SEC] }).catch(() => {});
  });
  const ask = () => request({
    title: t('제자리 걸음을 시작할까요?'),
    message: t('로봇이 제자리에서 약 {s}초 걷고 다시 섭니다. 주변 1 m 를 비우세요. 점검 중에는 조이스틱 입력이 무시됩니다.').replace('{s}', String(DRIFT_SEC)),
    confirmLabel: t('걷기 시작'), danger: true,
    run: () => { if (standAllowed()) start(); },
  });

  const cm = (mm: number) => `${mm >= 0 ? '+' : ''}${(mm / 10).toFixed(1)} cm`;
  const phaseText = !running ? null
    : !cur || cur.state === GAIT_CHECK.entering ? t('걷기 전환 중')
    : cur.state === GAIT_CHECK.walking ? t('걷는 중 {p}%').replace('{p}', String(cur.percent))
    : t('서는 중');
  const failText = noReply ? t('로봇이 응답하지 않습니다 — 로봇 소프트웨어가 이 점검을 지원하는지 확인하세요')
    : cur?.reason === 1 ? t('서 있는 상태(모델 보행)에서만 시작할 수 있습니다')
    : cur?.reason === 2 ? t('로봇이 움직이고 있어 시작하지 않았습니다 — 멈춘 뒤 다시 측정하세요')
    : cur?.reason === 3 ? t('걷기로 전환하지 못했습니다')
    : cur?.reason === 5 ? t('걷기를 마친 뒤 제자리에 서지 못했습니다')
    : t('중지되었습니다 — 다시 측정하세요');
  return (
    <>
      <StageCard no={5} title={t('제자리 걸음 · 밀림')} verdict={verdict} active
        cond={running ? <Text style={{ color: c.accent2, fontSize: 12 }}>{phaseText}</Text>
          : <GuardLine guard={guard} ok={t('서 있음 — 측정할 수 있습니다')} />}>
        <Text style={{ color: c.muted, fontSize: 12, lineHeight: 16 }}>
          {t('로봇이 제자리에서 {s}초 걸은 뒤 다시 섭니다. 시작할 때와 다시 선 뒤의 몸통 위치 차이를 시작 방향 기준으로 잽니다.').replace('{s}', String(DRIFT_SEC))}
        </Text>
        <View style={styles.cells}>
          <Cell label={t('앞뒤 밀림 (+앞)')} value={shown ? cm(shown.x_mm) : '—'}
            basis={t('기준 ±{v} cm 미만').replace('{v}', String(DRIFT_FB_WARN_CM))} tone={shown ? (overFb ? 'warn' : 'ok') : undefined} />
          <Cell label={t('좌우 밀림 (+좌)')} value={shown ? cm(shown.y_mm) : '—'}
            basis={t('기준 ±{v} cm 미만').replace('{v}', String(DRIFT_LR_WARN_CM))} tone={shown ? (overLr ? 'warn' : 'ok') : undefined} />
          <Cell label={t('방향 변화')} value={shown ? `${shown.yaw_deg >= 0 ? '+' : ''}${shown.yaw_deg.toFixed(1)}°` : '—'} basis={t('참고')} />
        </View>
        {running && <ProgressLine percent={cur?.percent ?? 0} />}
        <View style={styles.foot}>
          {!running && guard.blocked && guard.fix && <CalibFixButton fix={guard.fix} />}
          {running
            ? <RunButton label={t('중지')} disabled={false} onPress={() => connection.sendMotion('stand')} />
            : <RunButton label={t('측정')} disabled={guard.blocked} onPress={ask} />}
          <RxButton label={t('육안검사')} enabled={!running} onPress={() => setVisualOpen(true)} />
          {!running && homeGuard.blocked && homeGuard.fix && <CalibFixButton fix={homeGuard.fix} />}
          <RxButton label={t('관절 홈포즈 세팅')} enabled={!running && !homeGuard.blocked} onPress={() => setHomeOpen(true)} />
          <RxButton label={t('무게 중심 보정')} enabled={!running && !zmpGuard.blocked}
            onPress={() => request(calibReq(t('무게 중심 오프셋 자동 보정'),
              t('보정이 끝날 때까지 로봇을 건드리지 마세요. 실행하면 로봇이 한 걸음 걸어 다리를 정렬한 뒤 자동 조정에 들어갑니다.'),
              null, zmpCalibAllowed, () => setZmpOpen(true)))} />
          <Text style={{ color: failed || (done && over) ? c.amberTx : c.muted, fontSize: 11.5, flex: 1, minWidth: 160 }}>
            {failed ? failText
              : done && over ? t('{d} 밀렸습니다 — 육안검사 → 관절 홈포즈 세팅 → 무게 중심 보정 순서로 진행하세요').replace('{d}', drifted)
              : t('앞뒤나 좌우로 밀리면 육안검사 → 관절 홈포즈 세팅 → 무게 중심 보정 순서로 진행하세요')}
          </Text>
        </View>
      </StageCard>
      {visualOpen && <VisualCheckModal onClose={() => setVisualOpen(false)} />}
      {homeOpen && <LegHomeSetFlow onClose={() => setHomeOpen(false)} />}
      {zmpOpen && <ZmpCalibModal onClose={() => setZmpOpen(false)} />}
    </>
  );
}

function CameraStage({ no, stage }: { no: number; stage: (typeof CAMERA_STAGES)[number] }) {
  const { c, radius } = useTheme();
  const [open, setOpen] = useState(false);
  return (
    <>
      <StageCard no={no} title={t(stage.title)} verdict="wait" active>
        <View style={[styles.result, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
          <Text style={{ color: c.muted, fontSize: 12.5, lineHeight: 18 }}>{t(stage.measure)}{'\n'}{t(stage.judge)}</Text>
        </View>
        <View style={styles.foot}>
          <RunButton label={t('측정')} disabled onPress={() => {}} />
          <RxButton label={t(stage.rx)} enabled onPress={() => setOpen(true)} />
          <Text style={{ color: c.muted, fontSize: 11.5, flex: 1, minWidth: 160 }}>{t('측정·판정은 준비 중입니다')}</Text>
        </View>
      </StageCard>
      {open && stage.modal === 'basic' && <CameraBasicCheckModal onClose={() => setOpen(false)} />}
      {open && stage.modal === 'onchip' && <DepthOnChipModal onClose={() => setOpen(false)} />}
      {open && stage.modal === 'extrinsic' && <DepthExtrinsicModal onClose={() => setOpen(false)} />}
    </>
  );
}

const verdictOf = (s: SampleState, ok: boolean): Verdict =>
  s.phase === 'measuring' ? 'measuring' : s.phase === 'done' ? (ok ? 'ok' : 'warn') : 'wait';

function GuardLine({ guard, ok }: { guard: CalibGuard; ok: string }) {
  const { c } = useTheme();
  return guard.blocked
    ? <Text style={{ color: c.redbright, fontSize: 12 }}>⛔ {guard.reason}</Text>
    : <Text style={{ color: c.muted, fontSize: 12 }}>✓ {ok}</Text>;
}

function FootNote({ state, text }: { state: SampleState; text: string }) {
  const { c } = useTheme();
  const failed = state.phase === 'failed';
  return (
    <Text style={{ color: failed ? c.amberTx : c.muted, fontSize: 11.5, flex: 1, minWidth: 160 }}>
      {failed ? t('로봇 상태 수신이 부족해 판정하지 못했습니다 — 연결을 확인하고 다시 측정하세요') : text}
    </Text>
  );
}

function ProgressLine({ percent }: { percent: number }) {
  const { c, radius } = useTheme();
  const p = Math.max(0, Math.min(100, Math.round(percent)));
  return (
    <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
      <View style={{ flex: 1, height: 6, borderRadius: radius.sm, backgroundColor: c.elev, overflow: 'hidden' }}>
        <View style={{ width: `${p}%`, height: '100%', backgroundColor: c.accent }} />
      </View>
      <Text style={{ color: c.accent2, fontSize: 12, fontWeight: '700', minWidth: 40, textAlign: 'right' }}>{p}%</Text>
    </View>
  );
}

function StageCard({ no, title, cond, verdict, active, dim, children }: {
  no: number; title: string; cond?: React.ReactNode; verdict: Verdict; active?: boolean; dim?: boolean; children?: React.ReactNode;
}) {
  const { c, radius } = useTheme();
  const tone = verdict === 'ok' ? { bg: 'rgba(63,185,80,0.14)', line: 'rgba(63,185,80,0.55)', tx: c.greenTx, label: t('정상') }
    : verdict === 'warn' ? { bg: 'rgba(210,153,34,0.12)', line: 'rgba(210,153,34,0.5)', tx: c.amberTx, label: t('주의') }
    : verdict === 'measuring' ? { bg: 'rgba(77,156,245,0.12)', line: 'rgba(77,156,245,0.5)', tx: c.accent2, label: t('측정 중') }
    : verdict === 'info' ? { bg: 'rgba(77,156,245,0.12)', line: 'rgba(77,156,245,0.5)', tx: c.accent2, label: t('측정 완료') }
    : verdict === 'fail' ? { bg: 'rgba(231,51,28,0.10)', line: 'rgba(255,107,94,0.55)', tx: c.redTx, label: t('중단') }
    : { bg: c.elev, line: c.line, tx: c.dim, label: t('대기') };
  const noSkin = verdict === 'ok' ? tone
    : active ? { bg: 'rgba(77,156,245,0.18)', line: 'rgba(77,156,245,0.6)', tx: c.accent2 }
    : { bg: c.elev, line: c.line, tx: c.muted };
  return (
    <View style={[styles.card, { backgroundColor: c.panel, borderColor: active ? 'rgba(77,156,245,0.6)' : c.line, borderRadius: radius.md, opacity: dim ? 0.6 : 1 }]}>
      <View style={[styles.no, { backgroundColor: noSkin.bg, borderColor: noSkin.line }]}>
        <Text style={{ color: noSkin.tx, fontSize: 14, fontWeight: '800' }}>{no}</Text>
      </View>
      <View style={{ flex: 1, minWidth: 0, gap: 6 }}>
        <View style={styles.head}>
          <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{title}</Text>
          {cond}
          <View style={{ flex: 1 }} />
          <View style={[styles.pill, { backgroundColor: tone.bg, borderColor: tone.line }]}>
            <Text style={{ color: tone.tx, fontSize: 11, fontWeight: '700' }}>{tone.label}</Text>
          </View>
        </View>
        {children}
      </View>
    </View>
  );
}

function Cell({ label, value, basis, tone }: { label: string; value: string; basis: string; tone?: 'ok' | 'warn' }) {
  const { c, fonts, radius } = useTheme();
  const skin = tone === 'ok' ? { bg: 'rgba(63,185,80,0.10)', line: 'rgba(63,185,80,0.5)' }
    : tone === 'warn' ? { bg: 'rgba(210,153,34,0.10)', line: 'rgba(210,153,34,0.5)' } : { bg: c.elev, line: c.line };
  return (
    <View style={[styles.cell, { backgroundColor: skin.bg, borderColor: skin.line, borderRadius: radius.sm }]}>
      <Text style={{ color: c.muted, fontSize: 11 }} numberOfLines={1}>{label}</Text>
      <Text style={{ color: c.text, fontSize: 13, fontFamily: fonts.mono, marginTop: 1 }}>{value}</Text>
      <Text style={{ color: c.dim, fontSize: 10.5, marginTop: 1 }}>{basis}</Text>
    </View>
  );
}

const measureLabel = (s: SampleState) => s.phase === 'measuring' ? `${Math.round(s.progress * 100)}%` : t('측정');

function RunButton({ label, disabled, onPress }: { label: string; disabled: boolean; onPress: () => void }) {
  const { c, radius } = useTheme();
  return (
    <Tappable disabled={disabled} onPress={disabled ? undefined : onPress}
      style={[styles.run, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm, opacity: disabled ? 0.4 : 1 }]}>
      <Text style={{ color: c.text, fontSize: 15, fontWeight: '800' }}>{label}</Text>
    </Tappable>
  );
}

function RxButton({ label, enabled, onPress }: { label: string; enabled: boolean; onPress: () => void }) {
  const { c, radius } = useTheme();
  return (
    <Tappable disabled={!enabled} onPress={enabled ? onPress : undefined}
      style={[styles.rx, { borderRadius: radius.sm, backgroundColor: 'rgba(77,156,245,0.12)', borderColor: 'rgba(77,156,245,0.6)', opacity: enabled ? 1 : 0.4 }]}>
      <Icon name="wrench" size={15} color={c.accent2} />
      <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '800' }}>{label}</Text>
    </Tappable>
  );
}

const styles = StyleSheet.create({
  stages: { gap: 10, marginLeft: 14 },
  result: { borderWidth: 1, paddingHorizontal: 12, paddingVertical: 10, minHeight: 44, justifyContent: 'center' },
  card: { borderWidth: 1, paddingHorizontal: 14, paddingVertical: 12, flexDirection: 'row', gap: 12, alignItems: 'flex-start' },
  no: { width: 34, height: 34, borderRadius: 17, borderWidth: 1.5, alignItems: 'center', justifyContent: 'center' },
  head: { flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap', minHeight: 34 },
  pill: { height: 20, paddingHorizontal: 8, borderRadius: 6, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  cells: { flexDirection: 'row', flexWrap: 'wrap', gap: 6 },
  cell: { flexGrow: 1, flexBasis: 150, borderWidth: 1, paddingHorizontal: 9, paddingVertical: 6 },
  foot: { flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap', marginTop: 2 },
  run: { height: 36, minWidth: 96, paddingHorizontal: 14, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  rx: { height: 36, paddingHorizontal: 12, borderWidth: 1, flexDirection: 'row', alignItems: 'center', gap: 6 },
});
