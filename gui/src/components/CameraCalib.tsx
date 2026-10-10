import { useEffect, useRef } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { create } from 'zustand';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { useTelemetry } from '@/store/telemetry';
import { visionRequest } from '@/lib/vision';
import { t } from '@/lib/i18n';

type Dialog = null | 'oak' | 'jig' | 'cancel';
type CalibUi = { dialog: Dialog; result: string; resultOk: boolean };
const useCalibUi = create<CalibUi>(() => ({ dialog: null, result: '', resultOk: false }));

let cancelling = false;
let clearTimer: ReturnType<typeof setTimeout> | null = null;
let calibSensors: number[] = [];

const CALIB_SENSORS = 6;
const runningSensors = (s: ReturnType<typeof useTelemetry.getState>) => {
  const out: number[] = [];
  for (let i = 0; i < CALIB_SENSORS; i++) if (s.sensors?.[i]?.camCalibRunning) out.push(i);
  return out;
};
const isRunning = (s: ReturnType<typeof useTelemetry.getState>) => runningSensors(s).length > 0;

function dismissResult() {
  if (clearTimer) { clearTimeout(clearTimer); clearTimer = null; }
  useCalibUi.setState({ result: '' });
}

export const useCamCalibRunning = () => useTelemetry(isRunning);

export function openCameraCalib(kind: 'jig' | 'oak') {
  useCalibUi.setState({ dialog: isRunning(useTelemetry.getState()) ? 'cancel' : kind });
}
export function askCancelCameraCalib() {
  useCalibUi.setState({ dialog: 'cancel' });
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
      '**전방·후방** 카메라만 보정된다.',
      '로봇이 약 **10분간** 스스로 보행한다.',
      '결과는 각 카메라의 **EEPROM**에 기록된다.',
    ],
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
  const { result, resultOk } = useCalibUi();
  if (!running && result === '') return null;

  const tone = running ? CALIB_RUNNING : resultOk ? CALIB_DONE : '#C44';
  const label = running ? t('카메라 캘리브레이션 진행 중') : result;
  return (
    <Tappable onPress={running ? askCancelCameraCalib : dismissResult}
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

  const prev = useRef(false);
  useEffect(() => {
    if (running === prev.current) return;
    prev.current = running;
    if (clearTimer) { clearTimeout(clearTimer); clearTimer = null; }
    if (running) {
      cancelling = false;
      calibSensors = runningSensors(useTelemetry.getState());
      useCalibUi.setState({ result: '', resultOk: false });
      return;
    }
    if (cancelling) { cancelling = false; return; }
    const sensors = useTelemetry.getState().sensors;
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
  }, [running]);

  const closeDialog = () => useCalibUi.setState({ dialog: null });

  const d = dialog !== null ? DLG[dialog] : null;
  const tone = d?.danger ? '#C44' : c.legacyAccent;

  return (
    <>
      {d !== null && (
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
                  if (dialog === 'oak') visionRequest.startOakCalib();
                  else if (dialog === 'jig') visionRequest.startJigCalib();
                  else { cancelling = true; visionRequest.stopCameraCalib(); }
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
});
