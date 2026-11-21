import { useEffect, useRef, type ReactNode } from 'react';
import { View, Text, StyleSheet, Image } from 'react-native';
import { create } from 'zustand';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { Select } from '@/components/ui/controls';
import { Frame } from '@/components/CameraCheckModals';
import { useFeatures } from '@/store/capability';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { visionRequest } from '@/lib/vision';
import { INI_SOURCES } from '@/lib/iniSetting';
import { actions, type OakCalibResult } from '@/lib/rest';
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
  result: string; resultOk: boolean; owners: object[];
};
const useCalibUi = create<CalibUi>(() => ({
  dialog: null, oakMode: '1', oakCams: '3', oakTypes: null,
  oakStarting: false, oakNoAnswer: false, oakResult: null, oakFigures: null, result: '', resultOk: false, owners: [],
}));

let cancelling = false;
let clearTimer: ReturnType<typeof setTimeout> | null = null;
let calibSensors: number[] = [];
let lastOakStep = 0;

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
  if (isRunning(tel)) { useCalibUi.setState({ dialog: isOakRunning(tel) ? 'oak' : 'cancel' }); return; }
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
  useCalibUi.setState({ dialog: isOakRunning(useTelemetry.getState()) ? 'oak' : 'cancel' });
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
  jig: {
    title: 'Jig Extrinsic 보정',
    checks: [
      '로봇이 **캘리브 지그**에 올바르게 안착되어 있을 것',
    ],
    notes: [
      '센서별 **base↔센서** 변환을 보정한다.',
      '결과는 **HAL 설정**에 기록된다.',
    ],
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
  if (dialog === 'oak') return null;

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
  const { c } = useTheme();
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
      useCalibUi.setState({ result: '', resultOk: false, oakStarting: false, oakResult: null, oakFigures: null });
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

  const d = dialog !== null ? DLG[dialog] : null;
  const tone = d?.danger ? '#C44' : c.legacyAccent;

  return (
    <>
      {owner && dialog === 'oak' && <OakDialog onClose={closeDialog} />}
      {owner && dialog === 'cancel' && <CancelDialog onClose={closeDialog} />}
      {d !== null && dialog === 'jig' && (
        <View style={styles.dim}>
          <View style={[styles.card, { backgroundColor: c.legacyPanelBg, borderColor: c.line }]}>
            <Text style={[styles.title, { color: c.text }]}>{t(d.title)}</Text>
            {d.checks.length > 0 && (
              <View style={[styles.checkBox, { backgroundColor: c.elev, borderColor: c.line }]}>
                <Text style={[styles.checkHd, { color: tone }]}>{t('시작 전 확인')}</Text>
                {d.checks.map((s) => (
                  <View key={s} style={styles.checkRow}>
                    <View style={[styles.bullet, { backgroundColor: tone }]} />
                    <View style={{ flex: 1 }}><Rich text={t(s)} color={c.muted} accent={c.text} size={12.5} /></View>
                  </View>
                ))}
              </View>
            )}
            <View style={styles.notes}>
              {d.notes.map((s) => (
                <Rich key={s} text={t(s)} color={c.muted} accent={c.text} size={12.5} />
              ))}
            </View>
            <View style={styles.row}>
              <DlgBtn
                label={t(d.ok)} bg={tone} fg="white"
                onPress={() => {
                  visionRequest.startJigCalib();
                  closeDialog();
                }}
              />
              <DlgBtn label={t(d.cancel)} bg={c.elev} fg={c.text} onPress={closeDialog} />
            </View>
          </View>
        </View>
      )}
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
function ActBtn({ label, c, radius, onPress, ghost, bg }: {
  label: string; c: Theme['c']; radius: Theme['radius']; onPress: () => void; ghost?: boolean; bg?: string;
}) {
  const fill = ghost ? c.elev : bg ?? c.accent;
  return (
    <Tappable onPress={onPress} style={[styles.startBtn, { backgroundColor: fill, borderColor: ghost ? c.line : fill, borderRadius: radius.md }]}>
      <Text style={{ color: ghost ? c.text : '#fff', fontSize: 14, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
}

function DlgBtn({ label, bg, fg, onPress }: { label: string; bg: string; fg: string; onPress: () => void }) {
  return (
    <Tappable onPress={onPress} style={[styles.dlgBtn, { backgroundColor: bg }]}>
      <Text style={{ color: fg, fontSize: 13, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
}

const styles = StyleSheet.create({
  banner: {
    flexDirection: 'row', alignItems: 'center', gap: 8,
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
    backgroundColor: 'rgba(0,0,0,0.55)',
  },
  dim: {
    position: 'absolute', top: 0, bottom: 0, left: 0, right: 0,
    backgroundColor: 'rgba(0,0,0,0.45)', alignItems: 'center', justifyContent: 'center', zIndex: 200,
  },
  card: { width: 420, maxWidth: '88%', borderWidth: 1, borderRadius: 10, paddingVertical: 18, paddingHorizontal: 22 },
  title: { fontSize: 16, fontWeight: '700', textAlign: 'center', marginBottom: 12 },
  checkBox: { borderWidth: 1, borderRadius: 9, paddingVertical: 12, paddingHorizontal: 14, gap: 7 },
  checkHd: { fontSize: 10.5, fontWeight: '700', letterSpacing: 0.6, textTransform: 'uppercase', marginBottom: 1 },
  checkRow: { flexDirection: 'row', gap: 9, alignItems: 'flex-start' },
  bullet: { width: 5, height: 5, borderRadius: 3, marginTop: 7 },
  notes: { gap: 3, marginTop: 13, marginBottom: 16, paddingHorizontal: 2 },
  row: { flexDirection: 'row', justifyContent: 'center', gap: 32 },
  dlgBtn: { minWidth: 96, height: 36, borderRadius: 6, alignItems: 'center', justifyContent: 'center' },
  example: { width: 340, height: 212 },
  dot: { width: 24, height: 24, borderRadius: 12, borderWidth: 1.5, alignItems: 'center', justifyContent: 'center' },
  dotLg: { width: 28, height: 28, borderRadius: 14 },
  steps: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingBottom: 12, borderBottomWidth: StyleSheet.hairlineWidth, flexWrap: 'wrap' },
  chip: { borderWidth: 1, borderRadius: 999, paddingHorizontal: 10, paddingVertical: 3 },
  tile: { flex: 1, borderWidth: 1, paddingHorizontal: 12, paddingVertical: 8, gap: 2 },
  line: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 12, paddingVertical: 8 },
  startBtn: { minWidth: 120, height: 40, paddingHorizontal: 18, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});

const OAK_EXAMPLE = require('@/assets/images/oak-calib-example.png');
