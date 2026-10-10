import { useEffect, useState } from 'react';
import { View, Text, StyleSheet, ScrollView } from 'react-native';
import { useTheme } from '@/theme';
import { useRb } from '@/rb/theme';
import { Tappable } from '@/components/anim';
import { H2, Desc } from '@/components/panels/settings/common';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { RBBadge } from '@/rb/components/RBBadge';
import { RBLabelButton } from '@/rb/components/RBLabelButton';
import { Icon, type IconName } from '@/components/Icon';
import type { BadgeColor } from '@/rb/vendor/badge.variants';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useFeatureCanFd, useFeatures } from '@/store/capability';
import { Segmented } from '@/components/ui/controls';
import { useLegQc, legQc, QC_JOINTS, JUDGED_STAGES, BODY_TILT_LIMIT_DEG, STAGE_METRICS, type QcStage } from '@/lib/legQc';
import { LEG_INFO } from '@/lib/legHomeCalib';
import { RobotTopViewArt } from '@/components/JointCalibArt';
import { RobotModel3D } from '@/components/RobotModel3D';
import { CalibRow, type ConfirmReq } from '@/components/panels/PowerPanel';
import { ImuNullModal } from '@/components/ImuNullModal';
import { ImuLevelHint } from '@/components/BubbleLevel';
import { useCalibGuard, calibAllowed, commissioning } from '@/lib/commissioning';
import { connection } from '@/lib/connection';
import { useCompactW } from '@/lib/layout';
import { t } from '@/lib/i18n';

const r2d = (r: number) => (r * 180) / Math.PI;
const LEG_OFF_OK = ['SITTING', 'CONTROL_OFF', 'FALL_MODE'];
const HOME_LEFT = [3, 1];
const HOME_RIGHT = [2, 0];
const legName = (leg: number) => t(LEG_INFO[leg].label);
const legTag = (leg: number) => `${LEG_INFO[leg].name}(${leg})`;

type Tone = 'success' | 'caution' | 'danger' | 'information' | 'neutral';
function devText(v: number | null | undefined, ref: number | null | undefined): string {
  if (v == null || ref == null || !isFinite(ref) || Math.abs(ref) < 1e-9) return '';
  const d = (100 * (v - ref)) / Math.abs(ref);
  return `${d >= 0 ? '+' : ''}${d.toFixed(0)}% · `;
}
function blend(base: string, over: string): string {
  const b = /^#([0-9a-f]{6})$/i.exec(base), o = /^#([0-9a-f]{6})([0-9a-f]{2})$/i.exec(over);
  if (!b || !o) return over;
  const a = parseInt(o[2], 16) / 255;
  const ch = (k: number) => Math.round(parseInt(b[1].slice(k, k + 2), 16) * (1 - a) + parseInt(o[1].slice(k, k + 2), 16) * a);
  return `#${[0, 2, 4].map((k) => ch(k).toString(16).padStart(2, '0')).join('')}`;
}
const VERDICT_RANK = [0, 3, 5, 4, 1, 2];
function worstOf(list: (number | null | undefined)[]): number | null {
  let w: number | null = null;
  for (const v of list) if (v != null && v >= 0 && v <= 5 && (w == null || VERDICT_RANK[v] > VERDICT_RANK[w])) w = v;
  return w;
}
const ERR_NAMES = ['JAM', 'CUR', 'BIG', 'INP', 'FLT', 'TMP', 'PS1', 'PS2'];
const errNames = (bits: number) => ERR_NAMES.filter((_, b) => bits & (1 << b)).join(' ') || `0x${bits.toString(16)}`;

const verdictTone = (v: number | null | undefined): Tone =>
  v === 0 ? 'success' : v === 1 ? 'caution' : v === 2 ? 'danger' : v === 4 ? 'information' : 'neutral';
const TONE_BADGE: Record<Tone, BadgeColor> = { success: 'green', caution: 'orange', danger: 'red', information: 'blue', neutral: 'gray' };

function verdictText(v: number | null | undefined): string {
  switch (v) {
    case 0: return 'PASS';
    case 1: return 'WARN';
    case 2: return 'FAIL';
    case 3: return 'INVALID';
    case 4: return t('저장됨');
    case 5: return t('기준 부족');
    default: return '---';
  }
}

type Ask = { title: string; message: string; label: string; run: () => void };
const Badge = ({ tone, label }: { tone: Tone; label: string }) => <RBBadge type="soft" color={TONE_BADGE[tone]}>{label}</RBBadge>;
const OkBadge = ({ label }: { label: string }) => <RBBadge type="capsule" color="green">{label}</RBBadge>;

function BigBtn({ kind, icon, label, onPress, disabled, fill, small }: { kind: 'primary' | 'ghost'; icon: IconName; label: string; onPress: () => void; disabled?: boolean; fill?: boolean; small?: boolean }) {
  const { c } = useRb();
  return (
    <View style={fill ? { flex: 1 } : undefined}>
      <RBLabelButton size={small ? 'sm' : 'md'} variant={kind === 'primary' ? 'brand' : 'outlined'} disabled={disabled} onPress={onPress} style={fill ? { width: '100%' } : undefined}
        leadingIcon={<Icon name={icon} size={small ? 15 : 18} color={kind === 'primary' ? c('fg-static-white') : c('fg-default')} />}>{label}</RBLabelButton>
    </View>
  );
}

type Mode = 'verify' | 'measure';
const MODE_INFO: Record<Mode, { name: string; desc: string; start: string; go: string }> = {
  verify: { name: '검증 모드', desc: '기준 셋과 비교해 판정합니다. 기준이 모자라면 판정하지 않습니다.', start: '검증 모드입니다. 측정 후 기존에 저장된 샘플과 비교합니다.', go: '검증 시작' },
  measure: { name: '측정 모드', desc: '판정하지 않고, 결과를 기준 셋에 저장합니다.', start: '측정 모드입니다. 측정 결과를 정상 기준값으로 저장합니다.', go: '측정 시작' },
};

function ModeCard({ mode, on, onPress }: { mode: Mode; on: boolean; onPress: () => void }) {
  const { c, radius } = useTheme();
  const { c: rc } = useRb();
  const info = MODE_INFO[mode];
  return (
    <Tappable onPress={onPress} accessibilityLabel={t(info.name)}
      style={[styles.mode, { borderRadius: radius.sm, borderWidth: on ? 2 : 1,
        borderColor: on ? rc('border-brand') : c.line, backgroundColor: on ? rc('fill-brand-subtlest') : c.panel }]}>
      <View style={[styles.row, { justifyContent: 'space-between', gap: 8 }]}>
        <View style={[styles.row, { gap: 8 }]}>
          <View style={[styles.radio, { borderColor: on ? rc('border-brand') : c.muted }]}>
            {on && <View style={[styles.radioDot, { backgroundColor: rc('fill-brand') }]} />}
          </View>
          <Text style={{ color: on ? c.text : c.muted, fontSize: 16, fontWeight: '700' }}>{t(info.name)}</Text>
        </View>
        {on && <RBBadge type="capsule" color="blue">{t('✓ 선택됨')}</RBBadge>}
      </View>
      <Text style={{ color: on ? c.muted : c.dim, fontSize: 13, marginTop: 4 }}>{t(info.desc)}</Text>
    </Tappable>
  );
}

function Card({ n, title, right, children, style }: { n: string; title: string; right?: React.ReactNode; children: React.ReactNode; style?: object }) {
  const { c, radius } = useTheme();
  const { c: rc } = useRb();
  return (
    <View style={[styles.card, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }, style]}>
      <View style={styles.cardHead}>
        <View style={[styles.num, { backgroundColor: rc('fill-brand') }]}>
          <Text style={{ color: rc('fg-static-white'), fontSize: 15, fontWeight: '800' }}>{n}</Text>
        </View>
        <Text style={{ color: c.text, fontSize: 17, fontWeight: '700', flex: 1 }}>{title}</Text>
        {right}
      </View>
      {children}
    </View>
  );
}

function Tile({ name, state, action, ok }: { name: string; state: React.ReactNode; action: React.ReactNode; ok?: boolean }) {
  const { c, radius } = useTheme();
  const { c: rc } = useRb();
  return (
    <View style={[styles.tile, { backgroundColor: ok ? rc('fill-success-subtler') : c.panel, borderColor: ok ? rc('border-success') : c.line, borderRadius: radius.sm }]}>
      <View style={[styles.row, { gap: 6 }]}>
        <Text style={{ color: c.muted, fontSize: 13, fontWeight: '600' }}>{name}</Text>
        <View style={{ flexShrink: 1 }}>{state}</View>
      </View>
      <View style={{ alignItems: 'flex-start' }}>{action}</View>
    </View>
  );
}

function CanTile({ touched, onCheck }: { touched: boolean; onCheck: () => void }) {
  const { c } = useTheme();
  const legOn = useTelemetry((s) => !!s.pdu?.fetLeg);
  const failed = useTelemetry((s) => {
    const js = s.robot?.joints;
    if (!js) return null;
    return js.slice(0, 12).map((j, i) => (j.connected ? -1 : i)).filter((i) => i >= 0).join(',J');
  });
  const shown = touched && legOn;
  const ok = shown && failed === '';
  const state = !shown ? <Text style={{ color: c.dim, fontSize: 13 }}>{t('미확인')}</Text>
    : failed == null ? <Text style={{ color: c.dim, fontSize: 13 }}>{t('수신 없음')}</Text>
    : ok ? <OkBadge label="CAN OK" /> : <Badge tone="danger" label={`No CAN: J${failed}`} />;
  return <Tile name={t('CAN 통신')} ok={ok} state={state} action={<BigBtn small kind="ghost" icon="pulse" label="CAN Check" onPress={onCheck} />} />;
}

function TiltReadout() {
  const { c, fonts, radius } = useTheme();
  const { c: rc } = useRb();
  const roll = useTelemetry((s) => (s.robot ? Math.round(r2d(s.robot.imu.rpy[0]) * 100) / 100 : null));
  const pitch = useTelemetry((s) => (s.robot ? Math.round(r2d(s.robot.imu.rpy[1]) * 100) / 100 : null));
  const box = (label: string, v: number | null) => {
    const ok = v != null && Math.abs(v) <= BODY_TILT_LIMIT_DEG;
    return (
      <View style={[styles.tilt, { borderRadius: radius.sm, backgroundColor: v == null ? c.panel : ok ? rc('fill-success-subtler') : rc('fill-danger-subtlest'),
        borderColor: v == null ? c.line : ok ? rc('border-success') : rc('border-danger') }]}>
        <Text style={{ color: c.muted, fontSize: 13, fontWeight: '600' }}>{t('몸통')} {label}</Text>
        <Text style={{ color: v == null ? c.dim : ok ? c.greenTx : c.redTx, fontFamily: fonts.mono, fontSize: 22, fontWeight: '700' }}>
          {v == null ? '—' : `${v.toFixed(2)}°`}
        </Text>
      </View>
    );
  };
  return (
    <View style={{ gap: 6 }}>
      <View style={[styles.row, { flexWrap: 'nowrap', gap: 10 }]}>
        {box('Roll', roll)}
        {box('Pitch', pitch)}
      </View>
      <Text style={{ color: c.dim, fontSize: 12 }}>{t('Init 허용 ±{l}°').replace('{l}', BODY_TILT_LIMIT_DEG.toFixed(1))}</Text>
    </View>
  );
}

function StageBox({ title, desc, badge, children }: { title: string; desc: string; badge?: React.ReactNode; children: React.ReactNode }) {
  const { c, radius } = useTheme();
  return (
    <View style={[styles.stage, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.sm }]}>
      <View style={[styles.row, { gap: 8 }]}>
        <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{title}</Text>
        <Text style={{ color: c.dim, fontSize: 12, flex: 1 }}>{desc}</Text>
        {badge}
      </View>
      {children}
    </View>
  );
}

function JointTable() {
  const { c, fonts } = useTheme();
  const { c: rc } = useRb();
  const joints = useTelemetry((s) => s.robot?.joints);
  const dot = (on: boolean) => <View style={[styles.dot, { backgroundColor: on ? c.green : c.line }]} />;
  const num = { color: c.text, fontSize: 12, fontFamily: fonts.mono } as const;
  const head = { color: c.dim, fontSize: 11, fontWeight: '700' } as const;
  return (
    <View>
      <View style={[styles.jRow, { borderColor: c.line }]}>
        <Text style={[styles.jName, head]}>{t('관절')}</Text>
        <Text style={[styles.jDot, head]}>C</Text><Text style={[styles.jDot, head]}>H</Text><Text style={[styles.jDot, head]}>R</Text>
        <Text style={[styles.jNum, head]}>{t('각도')}</Text><Text style={[styles.jNum, head]}>{t('전류')}</Text><Text style={[styles.jTemp, head]}>{t('모터/보드')}</Text>
      </View>
      {Array.from({ length: 12 }, (_, i) => {
        const j = joints?.[i];
        const bad = !!j && j.errors.length > 0;
        return (
          <View key={i} style={[styles.jRow, { borderColor: c.line, backgroundColor: bad ? rc('fill-danger-subtlest') : 'transparent' }]}>
            <Text style={[styles.jName, { color: c.text, fontSize: 12 }]} numberOfLines={1}>
              <Text style={{ fontWeight: '700' }}>{LEG_INFO[Math.floor(i / 3)].name}</Text> {QC_JOINTS[i % 3]}
              {bad && <Text style={{ color: c.redTx, fontWeight: '700' }}> {j!.errors.join(' ')}</Text>}
            </Text>
            <View style={styles.jDot}>{dot(!!j?.connected)}</View>
            <View style={styles.jDot}>{dot(!!j?.calib)}</View>
            <View style={styles.jDot}>{dot(!!j?.run)}</View>
            <Text style={[styles.jNum, num]}>{j ? `${r2d(j.position).toFixed(1)}°` : '—'}</Text>
            <Text style={[styles.jNum, num]}>{j ? `${j.current.toFixed(2)} A` : '—'}</Text>
            <Text style={[styles.jTemp, num]}>{j ? `${j.statorTemp.toFixed(0)} / ${j.temperature.toFixed(0)}°C` : '—'}</Text>
          </View>
        );
      })}
    </View>
  );
}

export function LegQcPanel() {
  const { c, fonts, radius } = useTheme();
  const { c: rc } = useRb();
  const q = useRobot((s) => s.robot?.leg_qc);
  const aging = useRobot((s) => s.robot?.leg_aging);
  const check = useRobot((s) => s.robot?.leg_check);
  const canBus = useRobot((s) => !!s.robot?.can_bus);
  const findPose = useRobot((s) => !!s.robot?.find_pose);
  const controlOn = useRobot((s) => !!s.robot?.control_started);
  const compactW = useCompactW();
  const [confirmReq, setConfirmReq] = useState<ConfirmReq | null>(null);
  const [imuOpen, setImuOpen] = useState(false);
  const result = useRobot((s) => s.robot?.leg_result);
  const robotTick = useRobot((s) => s.robot);
  const gait = useRobot((s) => s.gait);
  const hasPdu = useTelemetry((s) => !!s.pdu);
  const legOn = useTelemetry((s) => !!s.pdu?.fetLeg);
  const canFd = useFeatureCanFd();
  const qcOn = useFeatures((s) => s.features?.qc === true);
  const imuGuard = useCalibGuard({ ignoreFall: qcOn });
  const qcOff = useFeatures((s) => s.features?.qc === false);
  const [qcOffAsk, setQcOffAsk] = useState(false);
  useEffect(() => { if (qcOff) setQcOffAsk(true); }, [qcOff]);
  const st = useLegQc();

  const [mode, setMode] = useState<Mode>('verify');
  const [statusOverride, setStatusOverride] = useState<string | null>(null);
  const [romBusy, setRomBusy] = useState(false);
  const [ask, setAsk] = useState<Ask | null>(null);
  const [tiltWarn, setTiltWarn] = useState<string | null>(null);
  const [err, setErr] = useState('');

  useEffect(() => { if (result) st.ingest(result); }, [result]); // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => { setStatusOverride(null); }, [robotTick]);

  const cj = q?.current_joint ?? -1;
  const running = q?.status === 1 || q?.status === 2;
  const statusText = statusOverride ?? (
    !q || q.status === 0 ? t('대기중')
      : q.status === 1 ? t('{j} 측정 중...').replace('{j}', cj >= 0 && cj < 3 ? QC_JOINTS[cj] : '?')
      : q.status === 2 ? t('관절 완료') : t('전체 완료'));
  const run = (p: Promise<unknown>) => { setErr(''); p.catch((e) => setErr(String(e?.message ?? e))); };
  const [pend, setPend] = useState(0);
  const robotBusy = check?.status === 1 || aging?.status === 1 || running;
  const runStage = check?.status === 1 ? 1 : aging?.status === 1 ? 2 : running ? 3 : pend;
  useEffect(() => {
    if (!pend) return;
    if (robotBusy) { setPend(0); return; }
    const id = setTimeout(() => setPend(0), 3000);
    return () => clearTimeout(id);
  }, [pend, robotBusy]);
  const moving = t('로봇 다리가 움직입니다. 주변을 비우고 로봇을 거치대에 올린 상태에서 실행하세요.');

  const onPower = () => {
    if (!legOn) { run(legQc.legPower(true)); return; }
    if (!LEG_OFF_OK.includes(gait)) { setErr(t('서 있는 중에는 LEG 전원을 끌 수 없습니다 — 앉거나 제어 OFF 후 다시 시도하세요.')); return; }
    setAsk({ title: t('LEG 전원 끄기'), message: t('다리 모터 전원을 끄고 이 화면의 결과를 지웁니다.'), label: t('끄기'),
      run: () => { run(legQc.legPower(false)); st.reset(); } });
  };
  const onCan = () => { useLegQc.setState({ canTouched: true }); run(legQc.canCheck()); };
  const onRom = () => setAsk({
    title: 'ROM Setting', message: t('모든 관절 보드에 FOC 게인과 전류 한계를 씁니다(약 7초).'), label: t('쓰기'),
    run: () => {
      setRomBusy(true);
      setErr('');
      legQc.romSetting()
        .then(() => useLegQc.setState({ rom: true }))
        .catch((e) => setErr(String(e?.message ?? e)))
        .finally(() => setRomBusy(false));
    },
  });
  const onHome = (leg: number) => setAsk({
    title: t('{leg} 홈포즈 세팅').replace('{leg}', legName(leg)), message: t('치구에 끼운 자세를 이 다리 관절 3개의 영점으로 저장합니다(roll → pitch/knee 순).'), label: t('저장'),
    run: () => {
      const home = [...useLegQc.getState().home];
      home[leg] = null;
      useLegQc.setState({ home, homeBusy: leg, homeErr: '' });
      legQc.legHome(leg)
        .then((ok) => { const h = [...useLegQc.getState().home]; h[leg] = ok; useLegQc.setState({ home: h }); })
        .catch((e) => {
          const h = [...useLegQc.getState().home]; h[leg] = false;
          useLegQc.setState({ home: h, homeErr: `${legName(leg)}: ${String(e?.message ?? e)}` });
        })
        .finally(() => useLegQc.setState({ homeBusy: -1 }));
    },
  });
  const onInit = () => {
    const rpy = useTelemetry.getState().robot?.imu.rpy;
    if (!rpy) { setTiltWarn(t('로봇 상태를 받고 있지 않습니다.')); return; }
    const roll = r2d(rpy[0]);
    const pitch = r2d(rpy[1]);
    if (Math.abs(roll) > BODY_TILT_LIMIT_DEG || Math.abs(pitch) > BODY_TILT_LIMIT_DEG) {
      setTiltWarn(t('로봇 몸통이 기울어져 있습니다. Roll: {r}°  Pitch: {p}° (허용: ±{l}°) — 수평을 맞춘 후 다시 시도하세요.')
        .replace('{r}', roll.toFixed(2)).replace('{p}', pitch.toFixed(2)).replace('{l}', BODY_TILT_LIMIT_DEG.toFixed(2)));
      return;
    }
    run(legQc.init());
  };
  const onStart = () => setAsk({
    title: `Start — ${t(MODE_INFO[mode].name)}`, message: `${t(MODE_INFO[mode].start)}\n\n${moving}`, label: t(MODE_INFO[mode].go),
    run: () => { setPend(3); run(legQc.start(mode === 'measure' ? 0 : 1)); setStatusOverride(t('실행 중...')); },
  });
  const onStop = () => { setPend(0); run(legQc.stop()); setStatusOverride(t('중지됨')); };
  const onReadyPose = () => setAsk({
    title: t('기본 자세로'), message: t('네 다리가 준비 자세로 움직입니다. 주변을 비웠는지 확인하세요.'), label: t('실행'),
    run: () => { setErr(''); connection.sendMotion('pos_stand'); },
  });
  const onAging = () => setAsk({
    title: t('에이징 시작'), message: t('발끝 추를 단 네 다리가 몇 분 동안 크게 움직입니다. 주변을 비웠는지 확인하세요. 처음 돌릴 때는 모터 온도를 지켜보세요.'), label: t('시작'),
    run: () => { setPend(2); run(legQc.agingStart()); },
  });
  const onCheck = () => setAsk({
    title: `${t('기본 동작 검사 시작')} — ${t(MODE_INFO[mode].name)}`, message: t('발끝 추를 뗀 상태에서 돌립니다. 네 다리가 관절별로(Roll → Pitch → Knee) 천천히 왕복합니다. 주변을 비웠는지 확인하세요.'), label: t('시작'),
    run: () => { setPend(1); run(legQc.checkStart(mode === 'measure' ? 0 : 1)); },
  });
  const checkOn = check?.status === 1;
  const checkPct = check && check.total_s > 0 ? Math.min(100, (check.elapsed_s / check.total_s) * 100) : 0;
  const agingOn = aging?.status === 1;
  const agingPct = aging && aging.total_s > 0 ? Math.min(100, (aging.elapsed_s / aging.total_s) * 100) : 0;
  const mmss = (s: number) => `${Math.floor(s / 60)}:${String(Math.floor(s % 60)).padStart(2, '0')}`;

  const sr = st.results[st.stage];

  const flagged: { fail: boolean; text: string }[] = [];
  let judgedAny = false;
  JUDGED_STAGES.forEach((stg) => st.results[stg].forEach((r, j) => {
    if (!r) return;
    for (let i = 0; i < 4; i++) {
      const where = `${t('{n}단계').replace('{n}', String(stg))} ${QC_JOINTS[j]} · ${legName(i)} ${legTag(i)}`;
      if (r.leg[i] === 0 || r.leg[i] === 1 || r.leg[i] === 2) judgedAny = true;
      STAGE_METRICS[stg].forEach((m, k) => {
        const vd = r.verdict[k]?.[i];
        if (vd !== 1 && vd !== 2) return;
        if (r.val[k]?.[i] == null) { flagged.push({ fail: false, text: `${where} · ${t(m.label)} ${t('계산 불가')}` }); return; }
        flagged.push({ fail: vd === 2, text: `${where} · ${t(m.label)} ${devText(r.val[k]?.[i], r.ref?.[k])}z ${r.z[k][i].toFixed(1)}` });
      });
      if (r.err[i] !== 0) flagged.push({ fail: true, text: `${where} · ${t('동작 중 에러')} ${errNames(r.err[i])}` });
      if (r.jump[i] !== 0) flagged.push({ fail: true, text: `${where} · ${t('위치 튐 {n}회').replace('{n}', String(r.jump[i]))}` });
      if (r.temp_warn[i]) flagged.push({ fail: false, text: `${where} · ${t('온도')} ${r.temp[i][0].toFixed(0)} / ${r.temp[i][1].toFixed(0)}°C` });
    }
  }));
  flagged.sort((a, b) => Number(b.fail) - Number(a.fail));

  const cellStyle = (v: number | null | undefined) => {
    const tone = verdictTone(v);
    return {
      backgroundColor: tone === 'success' ? rc('fill-success-subtlest') : tone === 'caution' ? rc('fill-caution-subtlest')
        : tone === 'danger' ? rc('fill-danger-subtlest') : tone === 'information' ? rc('fill-information-subtlest') : 'transparent',
      color: tone === 'success' ? c.greenTx : tone === 'caution' ? c.amberTx : tone === 'danger' ? c.redTx : tone === 'information' ? c.cyanTx : c.dim,
    };
  };
  const legHead = (i: number, key?: string) => (
    <View key={key ?? i} style={styles.tCell}>
      <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{legName(i)}</Text>
      <Text style={{ color: c.dim, fontSize: 12, fontFamily: fonts.mono }}>{legTag(i)}</Text>
    </View>
  );

  const verdictGrid = (
    <View style={[styles.tbl, { borderColor: c.line, borderRadius: radius.sm }]}>
      <View style={[styles.tRow, { borderColor: c.line }]}>
        <Text style={[styles.tLabel, { color: c.dim, fontSize: 12 }]}>{t('1·3단계 종합')}</Text>
        {[0, 1, 2, 3].map((i) => legHead(i))}
      </View>
      {QC_JOINTS.map((name, j) => {
        const chip = (v: number | null | undefined, label: string, k: string) => {
          const cs = cellStyle(v);
          return <Text key={k} style={[styles.chip, { backgroundColor: cs.backgroundColor, color: cs.color, borderRadius: radius.sm }]}>{label}</Text>;
        };
        return (
          <Tappable key={name} onPress={() => st.setTab(j)} accessibilityLabel={name}
            style={[styles.tRow, { borderColor: c.line, backgroundColor: st.tab === j ? rc('fill-brand-subtlest') : 'transparent' }, j === 2 && { borderBottomWidth: 0 }]}>
            <Text style={[styles.tLabel, { color: st.tab === j ? c.accent2 : c.text, fontSize: 15, fontWeight: '700' }]}>{name}</Text>
            {[0, 1, 2, 3].map((i) => {
              const v = JUDGED_STAGES.map((stg) => ({ stg, v: st.results[stg][j]?.leg[i] }));
              const hit = v.filter((x) => x.v === 1 || x.v === 2);
              const worst = worstOf(v.map((x) => x.v));
              return (
                <View key={i} style={[styles.tCell, { gap: 3 }]}>
                  {hit.length > 0
                    ? hit.map((x) => chip(x.v, `${t('{n}단계').replace('{n}', String(x.stg))} ${verdictText(x.v)}`, String(x.stg)))
                    : chip(worst, verdictText(worst), 'w')}
                </View>
              );
            })}
          </Tappable>
        );
      })}
    </View>
  );

  const detailTable = (j: number) => {
    const row = sr[j];
    const metrics = STAGE_METRICS[st.stage];
    const mono = (text: string, bad = false) => (
      <Text style={{ color: !row ? c.dim : bad ? c.redTx : c.text, fontSize: 14, fontFamily: fonts.mono, fontWeight: bad ? '700' : '400' }}>{text}</Text>
    );
    const rows: [string, (i: number) => React.ReactNode][] = [
      ...metrics.map((m, k): [string, (i: number) => React.ReactNode] => [`${t(m.label)} [${m.unit}]`, (i) => {
        const v = row?.val[k]?.[i];
        const vd = row?.verdict[k]?.[i];
        const judged = !m.info && v != null && (vd === 0 || vd === 1 || vd === 2);
        if (row && v == null && vd === 1) return <Text style={{ color: c.amberTx, fontSize: 14, fontWeight: '700' }}>{t('계산 불가')}</Text>;
        return (
          <>
            <Text style={{ color: v == null ? c.dim : vd === 2 ? c.redTx : vd === 1 ? c.amberTx : c.text, fontSize: 17, fontWeight: '600', fontFamily: fonts.mono }}>
              {v == null ? '-.---' : v.toFixed(3)}
            </Text>
            {judged && <Text style={{ color: vd === 0 ? c.dim : vd === 1 ? c.amberTx : c.redTx, fontSize: 12, fontFamily: fonts.mono }}>{devText(v, row!.ref?.[k])}z {row!.z[k][i].toFixed(1)}</Text>}
          </>
        );
      }]),
    ];
    if (st.stage !== 2) {
      rows.push([t('판정'), (i) => {
        const cs = cellStyle(row?.leg[i]);
        return <Text style={[styles.chip, { backgroundColor: cs.backgroundColor, color: cs.color, borderRadius: radius.sm }]}>{verdictText(row?.leg[i])}</Text>;
      }]);
      rows.push([t('온도 모터 / 보드 [°C]'), (i) => (
        <Text style={{ color: !row ? c.dim : row.temp_warn[i] ? c.amberTx : c.text, fontSize: 14, fontFamily: fonts.mono, fontWeight: row?.temp_warn[i] ? '700' : '400' }}>
          {row ? `${row.temp[i][0].toFixed(0)} / ${row.temp[i][1].toFixed(0)}` : '-- / --'}
        </Text>
      )]);
      rows.push([t('에러 비트 / 위치 튐'), (i) => mono(row ? `0x${row.err[i].toString(16).padStart(2, '0')} / ${row.jump[i]}` : '-- / --', !!row && (row.err[i] !== 0 || row.jump[i] !== 0))]);
    }
    rows.push([`CAN ${t('에러 프레임')}`, (i) => {
      const n = row?.can_err[i];
      return <Text style={{ color: n == null ? c.dim : n === 0 ? c.greenTx : c.text, fontSize: 14, fontFamily: fonts.mono }}>{n == null ? '--' : n}</Text>;
    }]);
    return (
      <View>
        <View style={[styles.tRow, { borderColor: c.line }]}>
          <View style={styles.tLabel} />
          {[0, 1, 2, 3].map((i) => legHead(i))}
        </View>
        {rows.map(([label, cells], k) => (
          <View key={label} style={[styles.tRow, { borderColor: c.line }, k === rows.length - 1 && { borderBottomWidth: 0 }]}>
            <Text style={[styles.tLabel, { color: c.muted, fontSize: 14, fontWeight: '600' }]}>{label}</Text>
            {[0, 1, 2, 3].map((i) => <View key={i} style={styles.tCell}>{cells(i)}</View>)}
          </View>
        ))}
      </View>
    );
  };

  const homeTile = (leg: number) => {
    const h = st.home[leg];
    const busy = st.homeBusy === leg;
    const locked = st.homeBusy >= 0;
    return (
      <Tappable key={leg} onPress={() => onHome(leg)} disabled={locked} accessibilityLabel={`${legName(leg)} ${t('세팅')}`}
        style={[styles.leg, { backgroundColor: h === true && !busy ? rc('fill-success-subtler') : c.panel, borderRadius: radius.sm, opacity: locked && !busy ? 0.5 : 1,
          borderColor: busy ? rc('border-information') : h == null ? c.line : h ? rc('border-success') : rc('border-danger') }]}>
        <View style={{ flex: 1, minWidth: 0 }}>
          <Text numberOfLines={1} style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{legName(leg)}</Text>
          <Text style={{ color: c.dim, fontSize: 12, fontFamily: fonts.mono }}>{legTag(leg)}</Text>
        </View>
        {busy ? <Badge tone="information" label={t('진행 중')} />
          : h != null ? (h ? <OkBadge label="OK" /> : <Badge tone="danger" label="NG" />)
          : <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '700' }}>{t('세팅')}</Text>}
      </Tappable>
    );
  };

  const folderFill = blend(c.panel, rc('fill-brand-subtlest'));
  const folderLine = blend(c.panel, rc('border-brand'));
  const stateBadge = (status: number | undefined, on: boolean) => (status == null || status === 0 ? undefined
    : on ? <Badge tone="information" label={t('진행 중')} /> : status === 2 ? <OkBadge label={t('완료')} /> : <Badge tone="caution" label={t('중지됨')} />);
  const progress = (pct: number, done: boolean, text: string) => (
    <View style={{ flex: 3, minWidth: 220, gap: 4 }}>
      <View style={[styles.prog, { backgroundColor: c.elev, borderColor: c.line }]}>
        <View style={{ width: `${pct}%`, height: '100%', backgroundColor: done ? c.green : rc('fill-brand') }} />
      </View>
      <Text style={{ color: c.muted, fontSize: 12 }}>{text}</Text>
    </View>
  );
  const bootStat = (label: string, on: boolean, onText: string) => (
    <View style={[styles.row, { gap: 5 }]}>
      <Text style={{ color: c.muted, fontSize: 12 }}>{label}</Text>
      {on ? <OkBadge label={onText} /> : <Badge tone="neutral" label="—" />}
    </View>
  );

  const left = (
    <>
      <H2>{t('다리 QC')}</H2>
      <Desc>{t('관절별 사인파 구동으로 전류·추종 오차를 기준값과 비교한다. 측정 모드는 기준값 샘플을 쌓고, 검증 모드는 판정한다.')}</Desc>

      {!!err && <Text style={{ color: c.redTx, fontSize: 13, marginBottom: 6 }}>{err}</Text>}

      <View style={[styles.row, { alignItems: 'stretch', gap: 12 }]}>
        <Card n="1" title={t('준비')} style={{ flex: 1, minWidth: 340, padding: 10 }}>
          <View style={[styles.row, { alignItems: 'stretch', gap: 8 }]}>
            <Tile name={t('LEG 전원')} ok={!canFd && hasPdu && legOn}
              state={canFd ? <Text style={{ color: c.dim, fontSize: 12 }}>{t('CAN-FD 기체는 전원 / 시스템 탭에서')}</Text>
                : !hasPdu ? <Text style={{ color: c.dim, fontSize: 13 }}>{t('수신 없음')}</Text>
                : legOn ? <OkBadge label="ON" /> : <Badge tone="danger" label="OFF" />}
              action={<BigBtn small kind="ghost" icon="power" label={legOn ? t('끄기') : t('켜기')} onPress={onPower} disabled={canFd || !hasPdu} />} />
            <CanTile touched={st.canTouched} onCheck={onCan} />
            <Tile name={t('ROM 설정')} ok={!romBusy && st.rom}
              state={romBusy ? <Badge tone="information" label={t('쓰는 중…')} /> : st.rom ? <OkBadge label={t('적용됨')} />
                : <Text style={{ color: c.dim, fontSize: 13 }}>{t('미적용')}</Text>}
              action={<BigBtn small kind="ghost" icon="save" label="ROM Setting" onPress={onRom} disabled={romBusy} />} />
          </View>
        </Card>

        <Card n="2" title={t('홈포즈 세팅')} style={{ flex: 1, minWidth: 400, padding: 10 }}>
          <View style={[styles.row, { flexWrap: 'nowrap', gap: 8 }]}>
            <View style={{ flex: 1, gap: 6 }}>{HOME_LEFT.map(homeTile)}</View>
            <View style={{ alignItems: 'center' }}>
              <Text style={{ color: c.accent2, fontSize: 12, fontWeight: '700' }}>{t('앞')}</Text>
              <RobotTopViewArt width={58} legColor={[0, 1, 2, 3].map((leg) =>
                st.homeBusy === leg ? c.cyan : st.home[leg] == null ? c.elev : st.home[leg] ? c.green : c.red)} />
            </View>
            <View style={{ flex: 1, gap: 6 }}>{HOME_RIGHT.map(homeTile)}</View>
          </View>
          {!!st.homeErr && <Text style={{ color: c.redTx, fontSize: 12, marginTop: 6 }}>{st.homeErr}</Text>}
        </Card>
      </View>

      <Card n="3" title={t('모드')} style={{ marginTop: 12 }}>
        <View style={[styles.row, { flexWrap: 'nowrap', alignItems: 'stretch', gap: 10 }]}>
          <ModeCard mode="verify" on={mode === 'verify'} onPress={() => setMode('verify')} />
          <ModeCard mode="measure" on={mode === 'measure'} onPress={() => setMode('measure')} />
        </View>
      </Card>

      <Card n="4" title={t('검사')} style={{ marginTop: 12 }}>
        <View style={[styles.row, { gap: 10, marginBottom: 10 }]}>
          <BigBtn kind="ghost" icon="stand" label="Init" onPress={onInit} disabled={runStage !== 0} />
          <BigBtn kind="ghost" icon="recover" label={t('기본 자세로')} onPress={onReadyPose} disabled={runStage !== 0} />
          <View style={[styles.row, { gap: 12, marginLeft: 'auto' }]}>
            {bootStat('CAN Check', canBus, 'OK')}
            {bootStat('Find Home', findPose, 'OK')}
            {bootStat('Control Start', controlOn, 'ON')}
          </View>
        </View>
        {!controlOn && <Text style={{ color: c.amberTx, fontSize: 12, marginBottom: 8 }}>{t('Init 을 눌러 Control Start 가 켜진 뒤에 검사·에이징을 시작할 수 있습니다.')}</Text>}
        <View style={{ gap: 8 }}>
          <StageBox title={t('① 기본 검사')} desc={t('추 없이 · Roll → Pitch → Knee 등속 왕복')} badge={stateBadge(check?.status, checkOn)}>
            <View style={[styles.row, { gap: 12 }]}>
              <View style={[styles.row, { flexWrap: 'nowrap', gap: 8, flex: 2, minWidth: 260 }]}>
                <BigBtn fill kind="primary" icon="play2" label={t('검사 시작')} onPress={onCheck} disabled={!controlOn || runStage !== 0} />
                <BigBtn fill kind="ghost" icon="pause" label={t('검사 중지')} onPress={() => { setPend(0); run(legQc.checkStop()); }} disabled={runStage !== 1} />
              </View>
              {progress(checkPct, check?.status === 2, check && check.status !== 0
                ? t('{j} · {e} / {tot} · 저장된 기록 {n}개')
                    .replace('{j}', QC_JOINTS[Math.max(0, Math.min(2, check.joint))])
                    .replace('{e}', mmss(check.elapsed_s)).replace('{tot}', mmss(check.total_s)).replace('{n}', String(check.log_count))
                : t('추를 달기 전에 관절이 정상으로 도는지 봅니다.'))}
            </View>
          </StageBox>
          <StageBox title={t('② 에이징')} desc={t('추 달고 · 세 관절 동시')} badge={stateBadge(aging?.status, agingOn)}>
            <View style={[styles.row, { gap: 12 }]}>
              <View style={[styles.row, { flexWrap: 'nowrap', gap: 8, flex: 2, minWidth: 260 }]}>
                <BigBtn fill kind="primary" icon="play2" label={t('에이징 시작')} onPress={onAging} disabled={!controlOn || runStage !== 0} />
                <BigBtn fill kind="ghost" icon="pause" label={t('에이징 중지')} onPress={() => { setPend(0); run(legQc.agingStop()); }} disabled={runStage !== 2} />
              </View>
              {progress(agingPct, aging?.status === 2, aging
                ? t('{lap}/{laps}바퀴 · {st}/{sts}단계 · {e} / {tot}')
                    .replace('{lap}', String(Math.min(aging.lap + 1, aging.laps))).replace('{laps}', String(aging.laps))
                    .replace('{st}', String(aging.stage + 1)).replace('{sts}', String(aging.stages))
                    .replace('{e}', mmss(aging.elapsed_s)).replace('{tot}', mmss(aging.total_s))
                : t('QC 전에 관절을 풀어 주는 동작입니다.'))}
            </View>
          </StageBox>
          <StageBox title={t('③ QC')} desc={t('추 달고 · Roll → Pitch → Knee 코사인')}
            badge={!q || q.status === 0 ? undefined : running ? <Badge tone="information" label={t('진행 중')} /> : q.status === 3 ? <OkBadge label={t('완료')} /> : undefined}>
            <View style={[styles.row, { gap: 12 }]}>
              <View style={[styles.row, { flexWrap: 'nowrap', gap: 8, flex: 2, minWidth: 260 }]}>
                <BigBtn fill kind="primary" icon="play2" label={t('검사 시작')} onPress={onStart} disabled={!controlOn || runStage !== 0} />
                <BigBtn fill kind="ghost" icon="pause" label={t('검사 중지')} onPress={onStop} disabled={runStage !== 3} />
              </View>
              <View style={{ flex: 3, minWidth: 220 }}>
                <Text style={{ color: c.muted, fontSize: 13 }}>
                  {t('상태')}  <Text style={{ color: running ? c.accent2 : c.text, fontWeight: '700' }}>{statusText}</Text>
                  {'   '}{t('기준 표본:')} <Text style={{ color: c.text, fontFamily: fonts.mono, fontWeight: '700' }}>{q?.n_samples ?? 0}</Text>
                </Text>
              </View>
            </View>
          </StageBox>
        </View>
      </Card>

      <Card n="5" title={t('결과')} style={{ marginTop: 12 }}>
        {(flagged.length > 0 || judgedAny) && (
          <View style={[styles.flag, { borderRadius: radius.sm, borderColor: flagged.length ? (flagged[0].fail ? rc('border-danger') : rc('border-caution')) : c.line,
            backgroundColor: flagged.length ? (flagged[0].fail ? rc('fill-danger-subtlest') : rc('fill-caution-subtlest')) : c.panel }]}>
            <Text style={{ color: c.muted, fontSize: 12, fontWeight: '700' }}>{t('걸린 항목')} {flagged.length > 0 ? flagged.length : ''}</Text>
            {flagged.length === 0
              ? <Text style={{ color: c.muted, fontSize: 13 }}>{t('걸린 항목이 없습니다.')}</Text>
              : flagged.map((f, k) => (
                <View key={k} style={[styles.row, { gap: 8, flexWrap: 'nowrap', alignItems: 'flex-start' }]}>
                  <Badge tone={f.fail ? 'danger' : 'caution'} label={f.fail ? 'FAIL' : 'WARN'} />
                  <Text style={{ color: c.text, fontSize: 13, flex: 1 }}>{f.text}</Text>
                </View>
              ))}
          </View>
        )}
        {verdictGrid}

        <View style={[styles.row, { gap: 14, marginTop: 16 }]}>
          <View style={{ width: 600, maxWidth: '100%' }}>
            <Segmented options={[{ key: '1', label: t('1단계 기본 검사 (추 없음)') }, { key: '2', label: t('2단계 에이징') }, { key: '3', label: t('3단계 QC (추 있음)') }]}
              value={String(st.stage)} onChange={(v) => st.setStage(Number(v) as QcStage)} />
          </View>
          {st.stage !== 2 && sr[st.tab] && (
            <Text style={{ color: c.muted, fontSize: 13 }}>
              {t(sr[st.tab]!.mode === 0 ? '측정 모드' : '검증 모드')} · {t('기준 표본:')} <Text style={{ fontFamily: fonts.mono, color: c.text }}>{sr[st.tab]!.n_ref}</Text>
            </Text>
          )}
        </View>

        <View style={[styles.row, { flexWrap: 'nowrap', alignItems: 'flex-end', gap: 6, marginTop: 10, zIndex: 2 }]}>
          {QC_JOINTS.map((name, j) => {
            const on = st.tab === j;
            return (
              <Tappable key={name} onPress={() => st.setTab(j)} accessibilityLabel={name}
                style={[styles.fTab, { borderTopLeftRadius: radius.sm, borderTopRightRadius: radius.sm },
                  on ? { borderColor: folderLine, backgroundColor: folderFill, marginBottom: -2, paddingBottom: 14 }
                     : { borderColor: c.line, backgroundColor: c.panel, borderWidth: 1, borderBottomWidth: 0 }]}>
                <Text style={{ color: on ? c.accent2 : c.muted, fontSize: 17, fontWeight: on ? '800' : '600' }}>{name}</Text>
              </Tappable>
            );
          })}
        </View>
        <View style={[styles.fBox, { borderColor: folderLine, backgroundColor: folderFill, borderRadius: radius.sm },
          st.tab === 0 && { borderTopLeftRadius: 0 }, st.tab === 2 && { borderTopRightRadius: 0 }]}>
          {detailTable(st.tab)}
        </View>
      </Card>
    </>
  );

  const right = (
    <>
      <TiltReadout />
      <CalibRow nm={t('IMU 롤/피치 영점')} sub={t('현재 자세를 수평 기준으로 영점 — 약 0.5초, 자세 유지')}
        blocked={imuGuard.blocked} blockReason={imuGuard.reason} fix={imuGuard.fix} allow={() => calibAllowed({ ignoreFall: useFeatures.getState().features?.qc === true })}
        confirmMsg={t('지금 자세가 수평 기준이 됩니다. 기울어진 상태로 실행하면 이후 제어가 전부 틀어집니다.')}
        confirmExtra={<ImuLevelHint />}
        onRequest={commissioning.standIfStanding}
        fire={() => setImuOpen(true)} request={setConfirmReq} />

      <View style={[styles.viewer, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
        <RobotModel3D bodyFixed listenPresets={false} showPresetRow={false} gridVisible={false} />
      </View>

      <Text style={{ color: c.dim, fontSize: 12, marginTop: 12, marginBottom: 4 }}>{t('관절 상태 · C 통신 · H 홈 · R 제어')}</Text>
      <JointTable />
    </>
  );

  const modals = (
    <>
      {ask && (
        <ConfirmModal title={ask.title} message={ask.message} confirmLabel={ask.label}
          onConfirm={() => { const r = ask.run; setAsk(null); r(); }} onClose={() => setAsk(null)} />
      )}
      {tiltWarn != null && (
        <ConfirmModal title="Leg QC" message={tiltWarn} confirmLabel={t('확인')} danger={false}
          onConfirm={() => setTiltWarn(null)} onClose={() => setTiltWarn(null)} />
      )}
      {imuOpen && <ImuNullModal onClose={() => setImuOpen(false)} />}
      {qcOffAsk && (
        <ConfirmModal title={t('QC 기능 꺼짐')} confirmLabel={t('확인')} danger={false}
          message={t('로봇 설정 [FEATURE] 의 QC 가 꺼져 있습니다. QC 를 켜고 로봇을 다시 기동한 뒤 이 탭을 쓰세요. 그때까지 이 탭의 버튼은 눌리지 않습니다.')}
          onConfirm={() => setQcOffAsk(false)} onClose={() => setQcOffAsk(false)} />
      )}
      {confirmReq && (
        <ConfirmModal title={confirmReq.title} message={confirmReq.message} confirmLabel={confirmReq.confirmLabel} danger={confirmReq.danger}
          onConfirm={() => { confirmReq.run(); setConfirmReq(null); }} onClose={() => setConfirmReq(null)}>
          {confirmReq.extra}
        </ConfirmModal>
      )}
    </>
  );

  const lock = { pointerEvents: (qcOff ? 'none' : 'auto') as 'none' | 'auto', opacity: qcOff ? 0.4 : 1 };
  const notice = qcOff && (
    <Text style={{ color: c.amberTx, fontSize: 13, fontWeight: '700', marginBottom: 10 }}>{t('로봇의 QC 기능이 꺼져 있어 이 탭을 쓸 수 없습니다.')}</Text>
  );

  if (compactW) {
    return (
      <>
        {notice}
        <ScrollView showsVerticalScrollIndicator={false} pointerEvents={lock.pointerEvents} style={{ opacity: lock.opacity }}>{left}<View style={{ height: 16 }} />{right}</ScrollView>
        {modals}
      </>
    );
  }
  return (
    <>
      {notice}
      <View style={styles.split}>
        <ScrollView style={{ flex: 2, opacity: lock.opacity }} pointerEvents={lock.pointerEvents} showsVerticalScrollIndicator={false}>{left}</ScrollView>
        <ScrollView style={{ flex: 1, opacity: lock.opacity }} pointerEvents={lock.pointerEvents} showsVerticalScrollIndicator={false}>{right}</ScrollView>
      </View>
      {modals}
    </>
  );
}

const styles = StyleSheet.create({
  split: { flex: 1, flexDirection: 'row', gap: 16 },
  flag: { borderWidth: 1, padding: 10, gap: 6, marginBottom: 10 },
  stage: { borderWidth: 1, padding: 10, gap: 8 },
  viewer: { height: 220, borderWidth: 1, overflow: 'hidden', marginTop: 10 },
  jRow: { flexDirection: 'row', alignItems: 'center', borderBottomWidth: 1, paddingVertical: 5, paddingHorizontal: 4 },
  jName: { flex: 1.5, minWidth: 0 },
  jDot: { width: 20, alignItems: 'center', textAlign: 'center' },
  jNum: { flex: 1, textAlign: 'right' },
  jTemp: { flex: 1.3, textAlign: 'right' },
  dot: { width: 9, height: 9, borderRadius: 5 },
  row: { flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap' },
  card: { borderWidth: 1, padding: 14 },
  cardHead: { flexDirection: 'row', alignItems: 'center', gap: 10, marginBottom: 12 },
  num: { width: 30, height: 30, borderRadius: 15, alignItems: 'center', justifyContent: 'center' },
  mode: { flex: 1, paddingVertical: 10, paddingHorizontal: 12 },
  radio: { width: 18, height: 18, borderRadius: 9, borderWidth: 2, alignItems: 'center', justifyContent: 'center' },
  radioDot: { width: 8, height: 8, borderRadius: 4 },
  prog: { height: 12, borderRadius: 6, borderWidth: 1, overflow: 'hidden' },
  tilt: { flex: 1, borderWidth: 1, paddingVertical: 8, paddingHorizontal: 12, gap: 2 },
  tile: { flex: 1, minWidth: 130, borderWidth: 1, padding: 8, gap: 6 },
  leg: { flexDirection: 'row', alignItems: 'center', gap: 8, borderWidth: 1, paddingHorizontal: 10, paddingVertical: 6 },
  fTab: { flex: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 8, paddingVertical: 12, borderWidth: 2, borderBottomWidth: 0 },
  fBox: { borderWidth: 2, paddingHorizontal: 8, paddingVertical: 4, zIndex: 1 },
  tbl: { borderWidth: 1, overflow: 'hidden' },
  tRow: { flexDirection: 'row', alignItems: 'center', borderBottomWidth: 1, paddingVertical: 9 },
  tLabel: { width: 190, paddingHorizontal: 12 },
  tCell: { flex: 1, alignItems: 'center', justifyContent: 'center', paddingHorizontal: 4 },
  chip: { minWidth: 76, textAlign: 'center', fontSize: 14, fontWeight: '700', paddingVertical: 3, paddingHorizontal: 8, overflow: 'hidden' },
});
