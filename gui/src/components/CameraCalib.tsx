import { useEffect, useRef, useState, type ReactNode } from 'react';
import { View, Text, StyleSheet, Image, useWindowDimensions } from 'react-native';
import { create } from 'zustand';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { Select } from '@/components/ui/controls';
import { Frame } from '@/components/CameraCheckModals';
import { SBtn } from '@/components/panels/settings/common';
import { useFeatures } from '@/store/capability';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { visionRequest } from '@/lib/vision';
import { INI_SOURCES } from '@/lib/iniSetting';
import { actions, type OakCalibResult, type JigCalibResult, type JigCalibRow, type CrossValidResult } from '@/lib/rest';
import { connection } from '@/lib/connection';
import { cameraName } from '@/lib/cameraCheckView';
import { t } from '@/lib/i18n';

type Dialog = null | 'oak' | 'jig' | 'cancel';
type OakMode = '1' | '2' | '3';
const OAK_MODES: { key: OakMode; label: string }[] = [
  { key: '1', label: 'RL Walk' }, { key: '2', label: 'Wave' }, { key: '3', label: 'Stairs' },
];
type OakCams = '3' | '1' | '2';
const OAK_CAMS: { key: OakCams; label: string }[] = [
  { key: '3', label: '전방·후방' }, { key: '1', label: '전방' }, { key: '2', label: '후방' },
];
type OakTypes = { front: boolean | null; rear: boolean | null };
const camsAllowed = (k: OakCams, ty: OakTypes | null) =>
  !ty || ((Number(k) & 1 ? ty.front !== false : true) && (Number(k) & 2 ? ty.rear !== false : true));
type OakResult = { front: boolean; rear: boolean; cams: number; failedAt: 'start' | 'search' | 'approach' | null; sat: boolean };
type CalibUi = {
  dialog: Dialog; oakMode: OakMode; oakCams: OakCams; oakTypes: OakTypes | null;
  oakStarting: boolean; oakNoAnswer: boolean; oakResult: OakResult | null;
  oakFigures: OakCalibResult | null;
  jigStarting: boolean; jigNoAnswer: boolean; jigReject: string; jigDone: boolean; jigResult: JigCalibResult | null;
  cv: CrossValidResult | null; cvError: string;
  result: string; resultOk: boolean; owners: object[];
};
const useCalibUi = create<CalibUi>(() => ({
  dialog: null, oakMode: '1', oakCams: '3', oakTypes: null,
  oakStarting: false, oakNoAnswer: false, oakResult: null, oakFigures: null,
  jigStarting: false, jigNoAnswer: false, jigReject: '', jigDone: false, jigResult: null, cv: null, cvError: '',
  result: '', resultOk: false, owners: [],
}));

let cancelling = false;
let clearTimer: ReturnType<typeof setTimeout> | null = null;
let calibSensors: number[] = [];
let lastOakStep = 0;
let jigStartedAt = 0;

const CALIB_SENSORS = 6;
const runningSensors = (s: ReturnType<typeof useTelemetry.getState>) => {
  const out: number[] = [];
  for (let i = 0; i < CALIB_SENSORS; i++) if (s.sensors?.[i]?.camCalibRunning) out.push(i);
  return out;
};
const isRunning = (s: ReturnType<typeof useTelemetry.getState>) => runningSensors(s).length > 0;
const OAK_FRONT = 4, OAK_REAR = 5;
const isOakRun = (sensors: number[]) => sensors.length > 0 && sensors.every((i) => i === OAK_FRONT || i === OAK_REAR);
const isOakRunning = (s: ReturnType<typeof useTelemetry.getState>) => isOakRun(runningSensors(s));
const oakStepByte = (s: ReturnType<typeof useTelemetry.getState>) =>
  s.sensors?.[OAK_FRONT]?.camCalibStep || s.sensors?.[OAK_REAR]?.camCalibStep || 0;
const oakCamMask = (sensors: number[]) => (sensors.includes(OAK_FRONT) ? 1 : 0) | (sensors.includes(OAK_REAR) ? 2 : 0);
const oakRunCams = (s: ReturnType<typeof useTelemetry.getState>) => oakCamMask(runningSensors(s));
const OAK_FRONT_APPROACH = 0x10;
const oakInPre = (b: number) => b === 0 || (b & 0xf8) === OAK_FRONT_APPROACH;

function dismissResult() {
  if (clearTimer) { clearTimeout(clearTimer); clearTimer = null; }
  useCalibUi.setState({ result: '' });
}

export const useCamCalibRunning = () => useTelemetry(isRunning);

export function openCameraCalib(kind: 'jig' | 'oak') {
  const tel = useTelemetry.getState();
  if (isRunning(tel)) { useCalibUi.setState({ dialog: isOakRunning(tel) ? 'oak' : 'jig' }); return; }
  if (kind === 'jig') {
    useCalibUi.setState({ dialog: 'jig', jigStarting: false, jigNoAnswer: false, jigReject: '', jigDone: false, jigResult: null, cv: null, cvError: '' });
    return;
  }
  const oakMode: OakMode = useFeatures.getState().features?.wheel === false ? '1' : '3';
  useCalibUi.setState({ dialog: kind, oakMode, oakCams: '3', oakTypes: null, oakStarting: false, oakNoAnswer: false, oakResult: null, oakFigures: null });
  if (kind === 'oak') loadOakTypes();
}
function loadOakTypes() {
  const hal = INI_SOURCES.find((x) => x.key === 'hal');
  if (!hal) return;
  hal.get(useRobot.getState().ip).then((f) => {
    const isOak = (n: number) => {
      const v = f.sections.find((x) => x.name === `SENSOR_${n}`)?.keys.find((k) => k.key === 'SensorType')?.value;
      return v === undefined ? null : Number(v) === 3;
    };
    const oakTypes: OakTypes = { front: isOak(OAK_FRONT), rear: isOak(OAK_REAR) };
    const { oakCams } = useCalibUi.getState();
    const ok = OAK_CAMS.find((o) => camsAllowed(o.key, oakTypes));
    useCalibUi.setState({ oakTypes, oakCams: camsAllowed(oakCams, oakTypes) || !ok ? oakCams : ok.key });
  }).catch(() => {});
}
function stopCameraCalibNow() {
  cancelling = true;
  visionRequest.stopCameraCalib();
}
export function askCancelCameraCalib() {
  useCalibUi.setState({ dialog: 'cancel' });
}
function onBannerRunning() {
  useCalibUi.setState({ dialog: isOakRunning(useTelemetry.getState()) ? 'oak' : 'jig' });
}

let startTimer: ReturnType<typeof setTimeout> | null = null;
const OAK_START_WAIT_MS = 10000;
function startOak(mode: number, cams: number) {
  visionRequest.startOakCalib(mode, cams);
  useCalibUi.setState({ oakStarting: true, oakNoAnswer: false, oakResult: null });
  if (startTimer) clearTimeout(startTimer);
  startTimer = setTimeout(() => {
    startTimer = null;
    if (useCalibUi.getState().oakStarting && !isRunning(useTelemetry.getState())) useCalibUi.setState({ oakStarting: false, oakNoAnswer: true });
  }, OAK_START_WAIT_MS);
}
const JIG_REJECT_POLL_MS = 1000;
async function startJig() {
  const ip = useRobot.getState().ip;
  useCalibUi.setState({ jigStarting: true, jigNoAnswer: false, jigReject: '', jigDone: false, jigResult: null, cv: null, cvError: '' });
  const before = await actions.jigCalibGet(ip).then((r) => r?.time ?? '', () => null);
  visionRequest.startJigCalib();
  const t0 = Date.now();
  if (startTimer) clearTimeout(startTimer);
  const check = async () => {
    startTimer = null;
    const ui = useCalibUi.getState();
    if (!ui.jigStarting || isRunning(useTelemetry.getState())) return;
    if (before !== null) {
      const r = await actions.jigCalibGet(ip).catch(() => null);
      if (r && r.state === 'aborted' && (r.time ?? '') !== before && useCalibUi.getState().jigStarting) {
        useCalibUi.setState({ jigStarting: false, jigReject: r.message || t('보정이 도중에 끝났습니다. 로봇과 지그를 확인하고 다시 보정하세요.') });
        return;
      }
    }
    if (Date.now() - t0 >= OAK_START_WAIT_MS) {
      if (useCalibUi.getState().jigStarting && !isRunning(useTelemetry.getState())) useCalibUi.setState({ jigStarting: false, jigNoAnswer: true });
      return;
    }
    startTimer = setTimeout(check, before !== null ? JIG_REJECT_POLL_MS : OAK_START_WAIT_MS - (Date.now() - t0));
  };
  startTimer = setTimeout(check, before !== null ? JIG_REJECT_POLL_MS : OAK_START_WAIT_MS);
}

const DLG = {
  oak: {
    title: 'OAK-D Intrinsic 보정',
    checks: [
      '로봇이 **서 있을 것**',
      '전방 카메라가 **1 m 이내**에서 **ChArUco 보드**를 향할 것',
      '로봇 반경 **1.5 m** 이내에 **장애물이 없을 것**',
    ],
    notes: [
      '결과는 각 카메라의 **EEPROM**에 기록된다.',
    ],
    warn: '로봇이 약 **10분간** 스스로 보행한다.',
    ok: '시작', cancel: '취소', danger: false,
  },
  cancel: {
    title: '캘리브레이션 취소',
    checks: [],
    notes: ['진행 중인 캘리브레이션을 **중단한다**.'],
    ok: '예', cancel: '아니오', danger: true,
  },
} as const;

function Rich({ text, color, accent, size }: { text: string; color: string; accent: string; size: number }) {
  return (
    <Text style={{ color, fontSize: size, lineHeight: size * 1.55 }}>
      {text.split('**').map((seg, i) =>
        i % 2 ? <Text key={i} style={{ color: accent, fontWeight: '700' }}>{seg}</Text> : seg)}
    </Text>
  );
}

const CALIB_RUNNING = '#FBBA16';
const CALIB_DONE    = '#00C853';

export function CameraCalibBanner() {
  const { radius } = useTheme();
  const running = useTelemetry(isRunning);
  const { result, resultOk, dialog } = useCalibUi();
  if (!running && result === '') return null;
  if (dialog === 'oak' || dialog === 'jig') return null;

  const tone = running ? CALIB_RUNNING : resultOk ? CALIB_DONE : '#C44';
  const label = running ? t('카메라 캘리브레이션 진행 중') : result;
  return (
    <Tappable onPress={running ? onBannerRunning : dismissResult}
      style={[styles.banner, { borderColor: tone, borderRadius: radius.md }]}>
      <Icon name="track" size={14} color={tone} />
      <Text style={{ color: tone, fontSize: 11.5, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
}

export function CameraCalibOverlays() {
  const { dialog } = useCalibUi();
  const running = useTelemetry(isRunning);

  const me = useRef({}).current;
  useEffect(() => {
    useCalibUi.setState((s) => ({ owners: [...s.owners, me] }));
    return () => useCalibUi.setState((s) => ({ owners: s.owners.filter((o) => o !== me) }));
  }, [me]);
  const owner = useCalibUi((s) => s.owners[0] === me);

  const prev = useRef(false);
  const owned = useRef(false);
  useEffect(() => {
    if (!owner) { owned.current = false; return; }
    if (!owned.current) {
      owned.current = true;
      if (running && calibSensors.length > 0) prev.current = true;
    }
    if (running === prev.current) return;
    prev.current = running;
    if (clearTimer) { clearTimeout(clearTimer); clearTimer = null; }
    if (running) {
      cancelling = false;
      lastOakStep = 0;
      calibSensors = runningSensors(useTelemetry.getState());
      jigStartedAt = Date.now();
      useCalibUi.setState({ result: '', resultOk: false, oakStarting: false, oakResult: null, oakFigures: null,
        jigStarting: false, jigDone: false, jigResult: null, cv: null, cvError: '' });
      return;
    }
    if (cancelling) { cancelling = false; return; }
    const sensors = useTelemetry.getState().sensors;
    if (isOakRun(calibSensors)) {
      const failedAt = lastOakStep === 0 ? 'start' : lastOakStep === OAK_FRONT_APPROACH ? 'search'
        : oakInPre(lastOakStep) ? 'approach' : null;
      useCalibUi.setState({ oakResult: {
        front: !!sensors?.[OAK_FRONT]?.camCalibSuccess, rear: !!sensors?.[OAK_REAR]?.camCalibSuccess, cams: oakCamMask(calibSensors),
        failedAt, sat: lastOakStep >> 4 >= 4,
      } });
      if (!failedAt) {
        actions.oakCalibGet(useRobot.getState().ip)
          .then((oakFigures) => { if (useCalibUi.getState().oakResult) useCalibUi.setState({ oakFigures }); })
          .catch(() => {});
      }
    } else {
      useCalibUi.setState({ jigDone: true });
      actions.jigCalibGet(useRobot.getState().ip)
        .then((jigResult) => { if (useCalibUi.getState().jigDone) useCalibUi.setState({ jigResult }); })
        .catch(() => {});
    }
    const failed = calibSensors
      .filter((i) => !sensors?.[i]?.camCalibSuccess)
      .map((i) => sensors?.[i]?.name || String(i));
    const ok = failed.length === 0;
    const partial = failed.length > 0 && failed.length < calibSensors.length;
    const msg = ok ? t('카메라 캘리브레이션 완료')
      : partial ? t('카메라 캘리브레이션 실패 - {s}').replace('{s}', failed.join(', '))
        : t('카메라 캘리브레이션 실패');
    useCalibUi.setState({ result: msg, resultOk: ok });
    if (ok) clearTimer = setTimeout(() => useCalibUi.setState({ result: '' }), 3000);
  }, [running, owner]);
  const oakStep = useTelemetry(oakStepByte);
  useEffect(() => { if (running) lastOakStep = oakStep; }, [running, oakStep]);

  const closeDialog = () => useCalibUi.setState({ dialog: null });

  return (
    <>
      {owner && dialog === 'oak' && <OakDialog onClose={closeDialog} />}
      {owner && dialog === 'cancel' && <CancelDialog onClose={closeDialog} />}
      {owner && dialog === 'jig' && <JigDialog onClose={closeDialog} />}
    </>
  );
}

type OakStage = 'pre' | 'capture' | 'calib' | 'result';
const OAK_STAGES: { key: OakStage; label: string }[] = [
  { key: 'pre', label: '선행 점검' }, { key: 'capture', label: '촬영' }, { key: 'calib', label: '보정' }, { key: 'result', label: '결과' },
];
const OAK_ROW = ['정면 0.5m', '정면 0.75m', '정면 1m', '우측 0.7m', '좌측 0.7m', '복귀', '180° 회전'];
function oakRows(cams: number): [string, string[], number][] {
  if ((cams & 3) === 1) return [['전방 카메라', OAK_ROW.slice(0, -1), 0]];
  if ((cams & 3) === 2) return [['후방 카메라', ['180° 회전', ...OAK_ROW], OAK_ROW.length - 1]];
  return [['전방 카메라', OAK_ROW, 0], ['후방 카메라', OAK_ROW, OAK_ROW.length]];
}
const OAK_CAPTURE_NOTE: Record<number, string> = {
  1: '자리마다 자세를 바꿔 가며 보드를 찍습니다.',
  2: '돌아서서 자리마다 자세를 바꿔 가며 후방 카메라로 보드를 찍습니다.',
  3: '자리마다 자세를 바꿔 가며 보드를 찍습니다. 전방 카메라를 마치면 돌아서서 후방 카메라로 찍습니다.',
};
const OAK_DEVICES: [string, number][] = [['전방 OAK-D', 1], ['후방 OAK-D', 2]];
function oakSeq(b: number) {
  const step = b >> 4, base = (b >> 3) & 1 ? OAK_ROW.length : 0, n = b & 7;
  if (step === 1) return base;
  if (step === 2) return base + Math.max(0, Math.min(n, 6) - 1);
  if (step === 3) return base + 6;
  return step >= 4 ? 2 * OAK_ROW.length : 0;
}

function OakDialog({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const { oakMode, oakCams, oakTypes, oakStarting, oakNoAnswer, oakResult, oakFigures } = useCalibUi();
  const running = useTelemetry(isOakRunning);
  const runCams = useTelemetry(oakRunCams);
  const b = useTelemetry(oakStepByte);
  const d = DLG.oak;
  const seq = oakSeq(b);
  const busy = running || oakStarting;
  const cams = running ? runCams : !busy && oakResult ? oakResult.cams : Number(oakCams);
  const camOptions = OAK_CAMS.filter((o) => camsAllowed(o.key, oakTypes)).map((o) => ({ key: o.key, label: t(o.label) }));
  const notOak = [oakTypes?.front === false ? t('전방') : '', oakTypes?.rear === false ? t('후방') : ''].filter(Boolean);
  const stage: OakStage = busy
    ? (b >> 4 >= 4 ? 'calib' : oakInPre(b) ? 'pre' : 'capture')
    : oakResult && !oakResult.failedAt ? 'result' : 'pre';
  const close = () => { if (oakResult) dismissResult(); onClose(); };

  const rich = (text: string) => <View style={{ flex: 1 }}><Rich text={t(text)} color={c.text} accent={c.text} size={14} /></View>;
  const line = (mark: string, color: string, text: string) => <Line c={c} radius={radius} mark={mark} color={color}>{rich(text)}</Line>;
  const chip = (label: string, state: 'done' | 'on' | 'todo') => (
    <View style={[styles.chip, { borderColor: state === 'done' ? c.green : state === 'on' ? c.accent2 : c.line }]}>
      <Text style={{ color: state === 'done' ? c.greenTx : state === 'on' ? c.accent2 : c.muted, fontSize: 13, fontWeight: '700' }}>{label}</Text>
    </View>
  );
  const chips = (labels: string[], cur: number, base = 0) => labels.map((x, i) => (
    <View key={i} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
      {i > 0 && <Text style={{ color: c.muted }}>→</Text>}
      {chip(t(x), cur > base + i ? 'done' : cur === base + i ? 'on' : 'todo')}
    </View>
  ));
  const banner = (ok: boolean, head: string, body: string) => (
    <View style={{ backgroundColor: ok ? 'rgba(63,185,80,0.12)' : 'rgba(231,51,28,0.10)', borderRadius: radius.sm, paddingHorizontal: 12, paddingVertical: 7, flexDirection: 'row', gap: 10, flexWrap: 'wrap' }}>
      <Text style={{ color: ok ? c.greenTx : c.redTx, fontSize: 14, fontWeight: '700' }}>{head}</Text>
      <Text style={{ color: c.text, fontSize: 14, flexShrink: 1 }}>{body}</Text>
    </View>
  );

  const px = (v: number | null) => (v == null ? '—' : v.toFixed(3));
  const figure = (label: string, value: string, bad = false) => (
    <View style={{ flexDirection: 'row', gap: 8, justifyContent: 'space-between' }}>
      <Text style={{ color: c.muted, fontSize: 13 }}>{label}</Text>
      <Text style={{ color: bad ? c.redTx : c.text, fontSize: 13, fontWeight: '600', flexShrink: 1, textAlign: 'right' }}>{value}</Text>
    </View>
  );

  const cur = OAK_STAGES.findIndex((x) => x.key === stage);
  const stepBar = (
    <View style={[styles.steps, { borderBottomColor: c.line }]}>
      {OAK_STAGES.map((s, i) => {
        const done = i < cur, on = i === cur;
        return (
          <View key={s.key} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
            {i > 0 && <View style={{ width: 20, height: 1.5, backgroundColor: c.line }} />}
            <View style={[styles.dot, styles.dotLg, { backgroundColor: done ? c.green : 'transparent', borderColor: done ? c.green : on ? c.accent : c.line }]}>
              <Text style={{ color: done ? '#fff' : on ? c.accent : c.muted, fontSize: 13, fontWeight: '800' }}>{done ? '✓' : i + 1}</Text>
            </View>
            <Text style={{ color: on ? c.text : c.muted, fontSize: 16, fontWeight: on ? '700' : '600' }}>{t(s.label)}</Text>
          </View>
        );
      })}
    </View>
  );

  const walking = (
    <>
      <View style={{ backgroundColor: c.elev, borderRadius: radius.sm, padding: 12, gap: 10 }}>
        {oakRows(cams).map(([side, labels, base]) => (
          <View key={side} style={{ flexDirection: 'row', alignItems: 'center', gap: 6, flexWrap: 'wrap' }}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', width: 92 }}>{t(side)}</Text>
            {chips(labels, seq, base)}
          </View>
        ))}
      </View>
      {line('i', c.accent2, OAK_CAPTURE_NOTE[cams & 3] ?? OAK_CAPTURE_NOTE[3])}
      {line('!', c.amberTx, '로봇이 스스로 걷습니다. 끝날 때까지 반경 **1.5 m** 안에 들어가지 마세요.')}
    </>
  );

  let body: ReactNode = null, actions: ReactNode = null;
  const stop = <ActBtn label={t('정지')} c={c} radius={radius} ghost onPress={stopCameraCalibNow} />;
  if (stage === 'pre') {
    body = (
      <>
        <View style={{ flexDirection: 'row', gap: 16, alignItems: 'center', flexWrap: 'wrap' }}>
          <Image source={OAK_EXAMPLE} style={styles.example} resizeMode="contain" />
          <View style={{ flex: 1, minWidth: 260, gap: 8 }}>
            <Text style={{ color: c.muted, fontSize: 13, fontWeight: '700' }}>{t('시작 전 확인')}</Text>
            {d.checks.map((s, i) => (
              <View key={s} style={{ flexDirection: 'row', gap: 8 }}>
                <View style={[styles.dot, { borderColor: c.accent }]}><Text style={{ color: c.accent, fontSize: 12, fontWeight: '800' }}>{i + 1}</Text></View>
                {rich(s)}
              </View>
            ))}
          </View>
        </View>
        {line('!', c.amberTx, d.warn)}
        {d.notes.map((s) => <View key={s}>{line('i', c.accent2, s)}</View>)}
        {!busy && (
          <Line c={c} radius={radius} mark="·" color={c.muted}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', flex: 1 }}>{t('보행 모드')}</Text>
            <View style={{ width: 180 }}>
              <Select options={OAK_MODES} value={oakMode} onChange={(m) => useCalibUi.setState({ oakMode: m })} />
            </View>
          </Line>
        )}
        {!busy && camOptions.length > 0 && (
          <Line c={c} radius={radius} mark="·" color={c.muted}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', flex: 1 }}>{t('보정 카메라')}</Text>
            <View style={{ width: 180 }}>
              <Select options={camOptions} value={oakCams} onChange={(k) => useCalibUi.setState({ oakCams: k })} />
            </View>
          </Line>
        )}
        {!busy && notOak.length > 0 && line('!', c.redTx, t('{s} 카메라가 OAK-D 가 아니라 보정할 수 없습니다.').replace('{s}', notOak.join(' · ')))}
        {(busy || !!oakResult?.failedAt) && (
          <View style={{ backgroundColor: c.elev, borderRadius: radius.sm, padding: 12, gap: 8 }}>
            <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, flexWrap: 'wrap' }}>
              <Text style={{ color: c.text, fontSize: 14, fontWeight: '700', marginRight: 2 }}>{t('보정 위치 잡기')}</Text>
              {chips(['보드 찾는 중', '접근', '완료'], !busy ? -1 : (b & 7) === 2 ? 3 : b & 7)}
              {busy && (b & 7) === 2 && <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '700' }}>{t('촬영 준비 중..')}</Text>}
            </View>
            {!busy && oakResult?.failedAt && (oakResult.failedAt === 'start'
              ? banner(false, t('시작하지 못했습니다'), t('로봇이 보행을 시작하지 못했습니다. 로봇이 서 있는지 확인하고 다시 시작하세요.'))
              : banner(false, t('보정 위치를 잡지 못했습니다'), oakResult.failedAt === 'search'
                ? t('전방 카메라가 ChArUco 보드를 찾지 못했습니다. 보드를 전방 카메라 1 m 이내 정면에 두고 다시 시작하세요.')
                : t('로봇이 보드 앞 보정 위치까지 가지 못했습니다. 로봇 주변을 비우고 다시 시작하세요.')))}
          </View>
        )}
        {oakNoAnswer && line('!', c.redTx, '로봇이 시작하지 않았습니다. 비전 연결을 확인하고 다시 시작하세요.')}
      </>
    );
    actions = busy ? stop : camOptions.length > 0
      ? <ActBtn label={t(d.ok)} c={c} radius={radius} onPress={() => startOak(Number(oakMode), Number(oakCams))} />
      : null;
  } else if (stage === 'capture') {
    body = walking;
    actions = stop;
  } else if (stage === 'calib') {
    const devices = OAK_DEVICES.filter(([, bit]) => cams & bit);
    const at = devices.findIndex(([, bit]) => bit === ((b >> 3) & 1 ? 2 : 1));
    body = (
      <>
        <View style={{ backgroundColor: c.elev, borderRadius: radius.sm, padding: 12, flexDirection: 'row', alignItems: 'center', gap: 6, flexWrap: 'wrap' }}>
          {chips(devices.map(([name]) => name), at)}
        </View>
        {line('i', c.accent2, '로봇이 앉아서 기다리는 동안 보정값을 계산해 각 카메라 **EEPROM**에 기록합니다. 몇 분 걸립니다.')}
      </>
    );
    actions = stop;
  } else if (oakResult) {
    const done = ([['전방', oakResult.front, 1], ['후방', oakResult.rear, 2]] as const).filter(([, , bit]) => oakResult.cams & bit);
    const failed = done.filter(([, ok]) => !ok).map(([side]) => t(side));
    body = (
      <>
        {failed.length === 0
          ? banner(true, t('보정했습니다'), done.length > 1 ? t('전방·후방 OAK-D 를 보정해 EEPROM 에 기록했습니다.')
            : t('{s} OAK-D 를 보정해 EEPROM 에 기록했습니다.').replace('{s}', done.map(([side]) => t(side)).join('')))
          : banner(false, t('보정하지 못했습니다'), t('{s} OAK-D 보정에 실패했습니다.').replace('{s}', failed.join(' · ')))}
        <View style={{ flexDirection: 'row', gap: 8 }}>
          {done.map(([side, ok]) => {
            const name = `${side} OAK-D`;
            const cam = oakFigures?.[side === '전방' ? 'front' : 'rear'];
            return (
              <View key={name} style={[styles.tile, { borderColor: ok ? c.green : c.red, borderRadius: radius.sm }]}>
                <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t(name)}</Text>
                <Text style={{ color: ok ? c.greenTx : c.redTx, fontSize: 14, fontWeight: '700' }}>{ok ? `✓ ${t('성공')}` : `✗ ${t('실패')}`}</Text>
                {cam?.calibrated && (
                  <View style={{ marginTop: 4, gap: 2 }}>
                    {figure(t('보드 검출'), `L ${cam.views_l} · R ${cam.views_r} / ${cam.images}`)}
                    {figure(t('단안 RMS'), `L ${px(cam.rms_l)} · R ${px(cam.rms_r)} px`)}
                    {figure(t('스테레오 RMS'), `${px(cam.rms_stereo)} px`)}
                    {figure('Epipolar L-R', `${px(cam.epi_lr)} px`)}
                    {!ok && !!cam.message && figure(t('실패 사유'), cam.message, true)}
                  </View>
                )}
              </View>
            );
          })}
        </View>
        {!!oakFigures?.error && done.some(([side]) => !oakFigures[side === '전방' ? 'front' : 'rear']?.calibrated)
          && line('!', c.redTx, `${t('실패 사유')}: ${oakFigures.error}`)}
        {oakResult.sat && line('!', c.amberTx, '로봇이 일어납니다.')}
      </>
    );
    actions = (
      <>
        <ActBtn label={t('다시 보정')} c={c} radius={radius} ghost onPress={() => { dismissResult(); useCalibUi.setState({ oakResult: null, oakFigures: null }); }} />
        <ActBtn label={t('완료')} c={c} radius={radius} onPress={close} />
      </>
    );
  }
  return (
    <Frame title={t(d.title)} width={980} onClose={close} head={stepBar} actions={actions}>
      {body}
      {busy && <Text style={{ color: c.muted, fontSize: 13 }}>{t('창을 닫아도 로봇은 계속합니다.')}</Text>}
    </Frame>
  );
}

type JigStage = 'pre' | 'calib' | 'result';
const JIG_STAGES: { key: JigStage; label: string }[] = [
  { key: 'pre', label: '선행 점검' }, { key: 'calib', label: '보정' }, { key: 'result', label: '결과' },
];
const JIG_PHASES = ['마커 TF', 'depth 평면', '계산 · 저장'];
const JIG_PHASE_HEAD = ['마커 TF 찾는 중', 'depth 평면 찾는 중', '계산 · 저장 중'];
const JIG_BAR: [number, number, number][] = [[0, 45, 12], [45, 85, 3], [85, 98, 2]];
const JIG_DEPTH_SENSORS = 4;
const CV_LIMIT_MM = 10;
const CV_POLL_MS = 1000, CV_POLL_MAX = 30;
type JigVerdict = 'ok' | 'check' | 'fail';
const jigVerdict = (r: JigCalibRow): JigVerdict => (r.result === 'no_marker' ? 'fail' : r.result === 'fail' || r.result === 'no_depth' ? 'check' : 'ok');
const JIG_WORD: Record<JigVerdict, string> = { ok: '정상', check: '확인 필요', fail: '실패' };
const jigRunSensors = (s: ReturnType<typeof useTelemetry.getState>) => (isOakRunning(s) ? [] : runningSensors(s));

function runCrossValid() {
  const ip = useRobot.getState().ip;
  useCalibUi.setState({ cv: { state: 'running', pairs: [] }, cvError: '' });
  const fail = () => useCalibUi.setState({ cv: null, cvError: t('교차 검증 결과를 받지 못했습니다. 비전 연결을 확인하고 다시 누르세요.') });
  let n = 0;
  const poll = () => {
    if (!useCalibUi.getState().jigDone) return;
    actions.crossValidGet(ip).then((cv) => {
      if (cv.state === 'running' && ++n < CV_POLL_MAX) { useCalibUi.setState({ cv }); setTimeout(poll, CV_POLL_MS); return; }
      if (cv.state === 'running') { fail(); return; }
      if (cv.error) { useCalibUi.setState({ cv: null, cvError: cv.error }); return; }
      useCalibUi.setState({ cv });
    }).catch(fail);
  };
  actions.crossValidStart(ip).then(() => setTimeout(poll, CV_POLL_MS)).catch(fail);
}

function JigDialog({ onClose }: { onClose: () => void }) {
  const { c, radius, fonts } = useTheme();
  const { jigStarting, jigNoAnswer, jigReject, jigDone, jigResult, cv, cvError } = useCalibUi();
  const sensors = useTelemetry((s) => s.sensors);
  const runKey = useTelemetry((s) => jigRunSensors(s).join(','));
  const runIdx = runKey ? runKey.split(',').map(Number) : [];
  const running = runIdx.length > 0;
  const busy = running || jigStarting;
  const stage: JigStage = busy ? (running ? 'calib' : 'pre') : jigDone ? 'result' : 'pre';
  const close = () => { if (jigDone) dismissResult(); onClose(); };
  const [, tick] = useState(0);
  useEffect(() => {
    if (!running) return;
    const h = setInterval(() => tick((n) => n + 1), 500);
    return () => clearInterval(h);
  }, [running]);
  const phaseAt = useRef({ phase: 0, at: Date.now() });
  const { width: winW } = useWindowDimensions();
  const tileBasis = Math.min(winW - 24, 980) >= 860 ? '15%' : '30%';

  const line = (mark: string, color: string, text: string, done = false) => (
    <Line c={c} radius={radius} mark={mark} color={color}><View style={{ flex: 1 }}><Rich text={done ? text : t(text)} color={c.text} accent={c.text} size={14} /></View></Line>
  );
  const banner = (tone: 'ok' | 'warn' | 'bad', head: string, body: string) => {
    const [bg, tx] = { ok: ['rgba(63,185,80,0.12)', c.greenTx], warn: ['rgba(210,153,34,0.12)', c.amberTx], bad: ['rgba(231,51,28,0.10)', c.redTx] }[tone];
    return (
      <View style={{ backgroundColor: bg, borderRadius: radius.sm, paddingHorizontal: 12, paddingVertical: 7, flexDirection: 'row', gap: 10, flexWrap: 'wrap' }}>
        <Text style={{ color: tx, fontSize: 14, fontWeight: '700' }}>{head}</Text>
        <Text style={{ color: c.text, fontSize: 14, flexShrink: 1 }}>{body}</Text>
      </View>
    );
  };
  const tone = {
    ok: { line: c.green, tx: c.greenTx }, check: { line: c.amberTx, tx: c.amberTx }, fail: { line: c.red, tx: c.redTx },
    none: { line: c.line, tx: c.muted },
  };
  const title = (text: string, note: string) => (
    <View style={{ flexDirection: 'row', alignItems: 'baseline', gap: 6, flexWrap: 'wrap', flexShrink: 1 }}>
      <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t(text)}</Text>
      <Text style={{ color: c.muted, fontSize: 12 }}>{t(note)}</Text>
    </View>
  );
  const mono = (v: string, color = c.text, flex = 1, bold = false) => (
    <Text style={{ flex, color, fontSize: 13, fontFamily: fonts.mono, textAlign: 'right', fontWeight: bold ? '700' : '400' }} numberOfLines={1}>{v}</Text>
  );
  const head = (cols: [string, number, boolean?][]) => (
    <View style={styles.trow}>
      {cols.map(([label, flex, left]) => (
        <Text key={label} style={{ flex, color: c.muted, fontSize: 12, fontWeight: '700', textAlign: left ? 'left' : 'right' }}>{t(label)}</Text>
      ))}
    </View>
  );
  const rowLine = { borderTopWidth: StyleSheet.hairlineWidth, borderTopColor: c.line };
  const camLabel = (id: number, flex: number) => (
    <Text style={{ flex, color: c.text, fontSize: 13, fontWeight: '700', fontFamily: fonts.mono }} numberOfLines={1}>{cameraName(id)}</Text>
  );

  const cur = JIG_STAGES.findIndex((x) => x.key === stage);
  const stepBar = (
    <View style={[styles.steps, { borderBottomColor: c.line }]}>
      {JIG_STAGES.map((s, i) => {
        const done = i < cur, on = i === cur;
        return (
          <View key={s.key} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
            {i > 0 && <View style={{ width: 20, height: 1.5, backgroundColor: c.line }} />}
            <View style={[styles.dot, styles.dotLg, { backgroundColor: done ? c.green : 'transparent', borderColor: done ? c.green : on ? c.accent : c.line }]}>
              <Text style={{ color: done ? '#fff' : on ? c.accent : c.muted, fontSize: 13, fontWeight: '800' }}>{done ? '✓' : i + 1}</Text>
            </View>
            <Text style={{ color: on ? c.text : c.muted, fontSize: 16, fontWeight: on ? '700' : '600' }}>{t(s.label)}</Text>
          </View>
        );
      })}
      <View style={{ flex: 1 }} />
      {running && <Text style={{ color: c.amberTx, fontSize: 13, fontWeight: '700' }}>{t('끝날 때까지 로봇을 움직이지 마세요')}</Text>}
    </View>
  );

  let body: ReactNode = null, act: ReactNode = null;
  if (stage === 'pre') {
    const step = (n: number, text: string, extra?: ReactNode) => (
      <View style={{ backgroundColor: c.elev, borderRadius: radius.sm, paddingHorizontal: 12, paddingVertical: 10, gap: 8 }}>
        <View style={{ flexDirection: 'row', gap: 8, alignItems: 'center' }}>
          <View style={[styles.dot, { borderColor: c.accent }]}><Text style={{ color: c.accent, fontSize: 12, fontWeight: '800' }}>{n}</Text></View>
          <View style={{ flex: 1 }}><Rich text={t(text)} color={c.text} accent={c.text} size={14} /></View>
        </View>
        {extra}
      </View>
    );
    body = (
      <>
        <Text style={{ color: c.muted, fontSize: 13, fontWeight: '700' }}>{t('시작 전 확인')}</Text>
        {step(1, '로봇을 **앉힌 뒤** 관절을 **고정**하세요.', (
          <View style={{ flexDirection: 'row', gap: 8, paddingLeft: 32 }}>
            <SBtn kind="primary" icon="sit" label={t('앉기')} onPress={() => connection.sendMotion('sit')} />
            <SBtn kind="primary" icon="sliders" label={t('관절 고정')} onPress={() => connection.sendMotion('lock')} />
          </View>
        ))}
        {step(2, '로봇을 **지그**에 올려 고정하세요.')}
        {!!jigReject && banner('bad', t('시작하지 못했습니다'), jigReject)}
        {jigNoAnswer && line('!', c.redTx, '로봇이 시작하지 않았습니다. 비전 연결을 확인하고 다시 시작하세요.')}
      </>
    );
    act = <ActBtn label={jigStarting ? t('시작 중..') : t('시작')} c={c} radius={radius} onPress={jigStarting ? () => {} : () => { void startJig(); }} />;
  } else if (stage === 'calib') {
    const bytes = runIdx.map((i) => sensors?.[i]?.camCalibStep ?? 0);
    const phase = Math.max(1, Math.min(3, ...bytes.map((b) => b >> 4)));
    if (phaseAt.current.phase !== phase) phaseAt.current = { phase, at: Date.now() };
    const [p0, p1, dur] = JIG_BAR[phase - 1];
    const pct = p0 + Math.min(1, (Date.now() - phaseAt.current.at) / 1000 / dur) * (p1 - p0);
    const elapsed = Math.max(0, Math.round((Date.now() - (jigStartedAt || Date.now())) / 1000));
    const mark = (ok: boolean, doing: boolean) => (ok ? [`✓ ${t('OK')}`, c.greenTx] : doing ? [t('찾는 중..'), c.accent2] : ['—', c.dim]) as [string, string];
    body = (
      <>
        <View style={{ backgroundColor: c.elev, borderRadius: radius.sm, padding: 14, gap: 10 }}>
          <View style={{ flexDirection: 'row', justifyContent: 'space-between' }}>
            <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{t(JIG_PHASE_HEAD[phase - 1])}</Text>
            <Text style={{ color: c.muted, fontSize: 13 }}>{t('{s}초').replace('{s}', String(elapsed))}</Text>
          </View>
          <View style={{ height: 6, borderRadius: 3, backgroundColor: c.elev2, overflow: 'hidden' }}>
            <View style={{ width: `${pct}%`, height: '100%', backgroundColor: c.accent2 }} />
          </View>
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, flexWrap: 'wrap' }}>
            {JIG_PHASES.map((x, i) => (
              <View key={x} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
                {i > 0 && <Text style={{ color: c.muted }}>→</Text>}
                <View style={[styles.chip, { borderColor: i + 1 < phase ? c.green : i + 1 === phase ? c.accent2 : c.line }]}>
                  <Text style={{ color: i + 1 < phase ? c.greenTx : i + 1 === phase ? c.accent2 : c.muted, fontSize: 13, fontWeight: '700' }}>{i + 1 < phase ? '✓ ' : ''}{t(x)}</Text>
                </View>
              </View>
            ))}
          </View>
        </View>
        <View style={{ flexDirection: 'row', gap: 8, flexWrap: 'wrap' }}>
          {runIdx.map((i) => {
            const b = sensors?.[i]?.camCalibStep ?? 0;
            const depth = i < JIG_DEPTH_SENSORS;
            const [mT, mC] = mark(!!(b & 1), phase === 1);
            const [dT, dC] = depth ? mark(!!(b & 2), phase === 2) : [t('— 안 씀'), c.dim];
            const done = !!(b & 1) && (!depth || !!(b & 2));
            return (
              <View key={i} style={[styles.jigTile, { flexBasis: tileBasis, borderColor: done ? c.green : c.line, borderRadius: radius.sm }]}>
                <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', fontFamily: fonts.mono }}>{cameraName(i)}</Text>
                {[[t('마커 TF'), mT, mC], [t('depth 평면'), dT, dC]].map(([k, v, col]) => (
                  <View key={k} style={{ flexDirection: 'row', justifyContent: 'space-between', gap: 6 }}>
                    <Text style={{ color: c.muted, fontSize: 12.5 }}>{k}</Text>
                    <Text style={{ color: col, fontSize: 12.5, fontWeight: '700' }}>{v}</Text>
                  </View>
                ))}
              </View>
            );
          })}
        </View>
        {line('i', c.accent2, '**마커 TF**는 프로젝터를 끈 left IR 영상에서 마커로 구한 카메라 위치입니다. **depth 평면**은 프로젝터를 켠 depth 로 추정한 마커판 평면입니다.')}
        {line('i', c.accent2, '창을 닫아도 보정은 계속합니다.')}
      </>
    );
    act = <ActBtn label={t('정지')} c={c} radius={radius} ghost onPress={stopCameraCalibNow} />;
  } else {
    const rows = jigResult?.state === 'done' ? [...jigResult.rows].sort((a, b) => a.idx - b.idx)
      : jigResult?.state === 'aborted' ? []
      : calibSensors.map((idx): JigCalibRow => ({ idx, result: sensors?.[idx]?.camCalibSuccess ? 'ir_only' : 'no_marker' }));
    const lim = jigResult?.limits;
    const checks = rows.filter((r) => jigVerdict(r) !== 'ok');
    const failed = rows.filter((r) => jigVerdict(r) === 'fail');
    const head1 = jigResult?.state === 'aborted' ? banner('bad', t('보정하지 못했습니다'), jigResult.message || t('보정이 도중에 끝났습니다. 로봇과 지그를 확인하고 다시 보정하세요.'))
      : failed.length > 0 ? banner('bad', t('일부 센서를 보정하지 못했습니다'), t('{s} 은 저장하지 않았습니다.').replace('{s}', failed.map((r) => cameraName(r.idx)).join(' · ')))
      : checks.length > 0 ? banner('warn', t('보정했습니다'), t('{n}대 모두 저장했습니다. 확인할 센서가 {m}대 있습니다.').replace('{n}', String(rows.length)).replace('{m}', String(checks.length)))
      : banner('ok', t('보정했습니다'), t('{n}대 모두 정상입니다.').replace('{n}', String(rows.length)));
    const report = (r: JigCalibRow) => {
      const name = `**${cameraName(r.idx)}**`;
      if (r.result === 'no_marker') return t('{s} — 마커를 찾지 못해 저장하지 않았습니다. 마커판이 카메라 정면에 있는지 확인하세요.').replace('{s}', name);
      if (r.result === 'no_depth') {
        if (!r.plate || r.frames === 0) return t('{s} — depth 영상이 오지 않아 마커 TF 로 저장했습니다. depth 스트림을 확인하세요.').replace('{s}', name);
        return t('{s} — depth 평면을 찾지 못했습니다 (채움 {f} %, 기준 {l} %). 마커판을 가리는 것이 없는지 확인하세요.')
          .replace('{s}', name).replace('{f}', (r.plate.fill * 100).toFixed(0)).replace('{l}', ((lim?.min_fill ?? 0.6) * 100).toFixed(0));
      }
      const overDeg = !lim || !r.corr || r.corr.deg > lim.corr_deg;
      return overDeg
        ? t('{s} — 마커 TF 와 depth 평면의 기울기 차이 **{d}°** (기준 {l}°). 카메라 점검 ② 로 depth 를 확인하세요.')
          .replace('{s}', name).replace('{d}', r.corr ? r.corr.deg.toFixed(2) : '—').replace('{l}', String(lim?.corr_deg ?? 5))
        : t('{s} — 마커 TF 와 depth 평면의 거리 차이 **{d} mm** (기준 {l} mm). 카메라 점검 ② 로 depth 를 확인하세요.')
          .replace('{s}', name).replace('{d}', r.corr!.mm.toFixed(1)).replace('{l}', String(lim.corr_mm));
    };
    const deltas = rows.filter((r) => r.tf_delta && r.tf_delta.length === 6);
    const fmt = (v: number, k: number) => { const x = k < 3 ? v * 1000 : v; const s = x.toFixed(k < 3 ? 1 : 2); return x > 0 ? `+${s}` : s; };
    const pairs = cv?.state === 'done' ? [...cv.pairs].filter((p) => p.markers.length > 0).sort((a, b) => a.a - b.a || a.b - b.b) : [];
    body = (
      <>
        {head1}
        <View style={{ flexDirection: 'row', gap: 8, flexWrap: 'wrap' }}>
          {rows.map((r) => {
            const v = jigVerdict(r);
            return (
              <View key={r.idx} style={[styles.jigTile, { flexBasis: tileBasis, borderColor: tone[v].line, borderRadius: radius.sm }]}>
                <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', fontFamily: fonts.mono }}>{cameraName(r.idx)}</Text>
                <Text style={{ color: tone[v].tx, fontSize: 14, fontWeight: '700' }}>{t(JIG_WORD[v])}</Text>
              </View>
            );
          })}
        </View>
        {checks.length > 0 && (
          <View style={{ gap: 6 }}>
            {title('확인할 센서', 'depth 평면을 반영하지 못하고 마커 TF 로 저장한 센서')}
            {checks.map((r) => <View key={r.idx}>{line('!', jigVerdict(r) === 'fail' ? c.redTx : c.amberTx, report(r), true)}</View>)}
          </View>
        )}
        {deltas.length > 0 && (
          <View style={{ gap: 2 }}>
            {title('저장한 보정값', '설계 위치 대비 보정량 · HAL config.ini [CALIB] TF_SENSOR_DELTA')}
            {head([['센서', 1.6, true], ['x mm', 1], ['y mm', 1], ['z mm', 1], ['roll °', 1], ['pitch °', 1], ['yaw °', 1]])}
            {deltas.map((r) => (
              <View key={r.idx} style={[styles.trow, rowLine]}>
                <View style={{ flex: 1.6, flexDirection: 'row', alignItems: 'baseline', gap: 6 }}>
                  <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', fontFamily: fonts.mono }}>{cameraName(r.idx)}</Text>
                  <Text style={{ color: c.muted, fontSize: 11.5, fontFamily: fonts.mono }}>DELTA{r.idx}</Text>
                </View>
                {r.tf_delta!.map((v, k) => <View key={k} style={{ flex: 1 }}>{mono(fmt(v, k))}</View>)}
              </View>
            ))}
          </View>
        )}
        {jigResult?.state === 'done' && (
          <View style={{ gap: 2 }}>
            <View style={{ flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', gap: 8 }}>
              {title('교차 검증', '저장한 보정값으로, 같은 마커를 두 카메라가 본 위치가 몸통 기준에서 얼마나 떨어지는지')}
              {cv?.state === 'done' && <ActBtn label={t('다시 검증')} c={c} radius={radius} ghost small onPress={runCrossValid} />}
            </View>
            {!cv ? (
              <View style={{ flexDirection: 'row', alignItems: 'center', gap: 10, paddingVertical: 6 }}>
                <ActBtn label={t('교차 검증')} c={c} radius={radius} ghost small onPress={runCrossValid} />
                <Text style={{ color: !!cvError ? c.redTx : c.muted, fontSize: 13, flexShrink: 1 }}>
                  {cvError || t('누르면 몇 초 동안 마커를 모아 카메라 쌍마다 위치 차를 잽니다.')}
                </Text>
              </View>
            ) : cv.state === 'running' ? (
              <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '700', paddingVertical: 6 }}>{t('마커를 모으는 중..')}</Text>
            ) : pairs.length === 0 ? (
              <Text style={{ color: c.muted, fontSize: 13, paddingVertical: 6 }}>{t('같은 마커를 본 카메라 쌍이 없습니다.')}</Text>
            ) : (
              <>
                {head([['카메라 쌍', 2.2, true], ['같이 본 마커', 1.2], ['위치 차 (평균)', 1.3], ['위치 차 (최대)', 1.3], ['판정', 1]])}
                <View style={styles.trow}>
                  <Text style={{ flex: 2.2, color: c.muted, fontSize: 11.5 }}>{t('기준')}</Text>
                  <View style={{ flex: 2.5 }} />
                  {mono(`≤ ${CV_LIMIT_MM} mm`, c.muted, 1.3)}
                  <View style={{ flex: 1 }} />
                </View>
                {pairs.map((p) => {
                  const mm = p.markers.map((m) => m.dist * 1000);
                  const mean = mm.reduce((x, y) => x + y, 0) / mm.length, max = Math.max(...mm);
                  const over = max > CV_LIMIT_MM;
                  return (
                    <View key={`${p.a}-${p.b}`} style={[styles.trow, rowLine]}>
                      <Text style={{ flex: 2.2, color: c.text, fontSize: 13, fontWeight: '700', fontFamily: fonts.mono }} numberOfLines={1}>{cameraName(p.a)} ↔ {cameraName(p.b)}</Text>
                      {mono(t('{n}개').replace('{n}', String(mm.length)), c.text, 1.2)}
                      {mono(`${mean.toFixed(1)} mm`, c.text, 1.3)}
                      {mono(`${max.toFixed(1)} mm`, over ? c.amberTx : c.text, 1.3, over)}
                      <Text style={{ flex: 1, color: over ? c.amberTx : c.greenTx, fontSize: 13, fontWeight: '700', textAlign: 'right' }}>{t(over ? '확인 필요' : '정상')}</Text>
                    </View>
                  );
                })}
                <Text style={{ color: c.muted, fontSize: 12.5, paddingTop: 4 }}>{t('같은 마커를 본 쌍만 나옵니다. 마커가 여러 개면 평균과 최대를 씁니다.')}</Text>
              </>
            )}
          </View>
        )}
      </>
    );
    act = (
      <>
        <ActBtn label={t('다시 보정')} c={c} radius={radius} ghost onPress={() => { dismissResult(); useCalibUi.setState({ jigDone: false, jigResult: null, cv: null, cvError: '' }); }} />
        <ActBtn label={t('완료')} c={c} radius={radius} onPress={close} />
      </>
    );
  }
  return (
    <Frame title={t('Jig Extrinsic 보정')} width={980} onClose={close} head={stepBar} actions={act}>
      {body}
    </Frame>
  );
}

function CancelDialog({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const d = DLG.cancel;
  const yes = <ActBtn label={t(d.ok)} c={c} radius={radius} bg={c.red} onPress={() => { stopCameraCalibNow(); onClose(); }} />;
  return (
    <Frame title={t(d.title)} width={480} onClose={onClose} head={<TitleRule />} actions={yes}>
      {d.notes.map((s) => (
        <Line key={s} c={c} radius={radius} mark="!" color={c.redTx}>
          <View style={{ flex: 1 }}><Rich text={t(s)} color={c.text} accent={c.text} size={14} /></View>
        </Line>
      ))}
    </Frame>
  );
}

type Theme = ReturnType<typeof useTheme>;
function TitleRule() {
  const { c } = useTheme();
  return <View style={{ height: StyleSheet.hairlineWidth, backgroundColor: c.line }} />;
}
function Line({ c, radius, mark, color, children }: { c: Theme['c']; radius: Theme['radius']; mark: string; color: string; children: ReactNode }) {
  return (
    <View style={[styles.line, { backgroundColor: c.elev, borderRadius: radius.sm }]}>
      <Text style={{ color, fontWeight: '800', width: 14, textAlign: 'center' }}>{mark}</Text>
      {children}
    </View>
  );
}
function ActBtn({ label, c, radius, onPress, ghost, bg, small }: {
  label: string; c: Theme['c']; radius: Theme['radius']; onPress: () => void; ghost?: boolean; bg?: string; small?: boolean;
}) {
  const fill = ghost ? c.elev : bg ?? c.accent;
  return (
    <Tappable onPress={onPress} style={[styles.startBtn, small && styles.smallBtn, { backgroundColor: fill, borderColor: ghost ? c.line : fill, borderRadius: radius.md }]}>
      <Text style={{ color: ghost ? c.text : '#fff', fontSize: small ? 13 : 14, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
}

const styles = StyleSheet.create({
  banner: {
    flexDirection: 'row', alignItems: 'center', gap: 8,
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
    backgroundColor: 'rgba(0,0,0,0.55)',
  },
  example: { width: 340, height: 212 },
  dot: { width: 24, height: 24, borderRadius: 12, borderWidth: 1.5, alignItems: 'center', justifyContent: 'center' },
  dotLg: { width: 28, height: 28, borderRadius: 14 },
  steps: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingBottom: 12, borderBottomWidth: StyleSheet.hairlineWidth, flexWrap: 'wrap' },
  chip: { borderWidth: 1, borderRadius: 999, paddingHorizontal: 10, paddingVertical: 3 },
  tile: { flex: 1, borderWidth: 1, paddingHorizontal: 12, paddingVertical: 8, gap: 2 },
  line: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 12, paddingVertical: 8 },
  startBtn: { minWidth: 120, height: 40, paddingHorizontal: 18, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  smallBtn: { minWidth: 0, height: 32, paddingHorizontal: 14 },
  jigTile: { flexGrow: 1, borderWidth: 1, paddingHorizontal: 10, paddingVertical: 8, gap: 4 },
  trow: { flexDirection: 'row', alignItems: 'center', paddingVertical: 5, gap: 6 },
});

const OAK_EXAMPLE = require('@/assets/images/oak-calib-example.png');
