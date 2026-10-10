import { createContext, useContext, useEffect, useRef, useState, type ReactNode } from 'react';
import { View, Text, StyleSheet, ScrollView, Image, useWindowDimensions,
         type LayoutChangeEvent } from 'react-native';
import { Modal } from '@/components/ui/overlays';
import { Tappable } from '@/components/anim';
import { useTheme } from '@/theme';
import { useRobot } from '@/store/robot';
import { useFeatureWheel } from '@/store/capability';
import { INIT_STATE, INIT_STEPS, type AutostartInfo, type InitState } from '@/types/robot';
import { t } from '@/lib/i18n';

type Variant = 'modern' | 'legacy';

const LEGS = [
  { cap: 'HR', name: '후방-우측', base: 0 },
  { cap: 'HL', name: '후방-좌측', base: 3 },
  { cap: 'FR', name: '전방-우측', base: 6 },
  { cap: 'FL', name: '전방-좌측', base: 9 },
] as const;
const AX = ['R', 'P', 'K'] as const;
const MAX_MC = 12;
const WHEELS = [
  { name: 'HRW', idx: 12 }, { name: 'HLW', idx: 13 },
  { name: 'FRW', idx: 14 }, { name: 'FLW', idx: 15 },
] as const;

export function jointName(i: number): string {
  if (i < 0) return '';
  if (i >= 12) return WHEELS[i - 12] ? `${WHEELS[i - 12].name}${i}` : `CH${i}`;
  return `${LEGS[Math.floor(i / 3)].cap}${AX[i % 3]}${i}`;
}

function useStateColor() {
  const { c } = useTheme();
  return (s: InitState | undefined) => {
    switch (s) {
      case INIT_STATE.pass: return c.greenTx;
      case INIT_STATE.fail: return c.redTx;
      case INIT_STATE.warn: return c.amberTx;
      case INIT_STATE.run:  return c.accent;
      default:              return c.dim;
    }
  };
}

function stateLabel(s: InitState | undefined): string {
  switch (s) {
    case INIT_STATE.pass: return t('통과');
    case INIT_STATE.fail: return t('실패');
    case INIT_STATE.warn: return t('경고');
    case INIT_STATE.run:  return t('진행 중');
    default:              return t('대기');
  }
}

function failReason(a: AutostartInfo): { title: string; hint: string } | null {
  if (!a.steps[a.step] || a.steps[a.step] !== INIT_STATE.fail) return null;
  const ch = a.fail_ch >= 0 ? jointName(a.fail_ch) : '';
  switch (INIT_STEPS[a.step]) {
    case 'precheck':
      return { title: t('이미 서 있는 상태입니다'),
               hint: t('로봇을 앉힌 뒤 다시 시작하세요.') };
    case 'leg_power':
      return { title: t('다리 48V 전원이 올라오지 않습니다'),
               hint: t('배터리 잔량과 퓨즈 상태를 확인하세요.') };
    case 'comm':
      return { title: `${ch} ${t('보드가 응답하지 않습니다')}`,
               hint: t('해당 다리의 전원과 CAN 커넥터를 확인한 뒤 다시 시작하세요.') };
    case 'param':
      return { title: `${ch} ${t('모터 설정값이 기대값과 다릅니다')}`,
               hint: t('보드 교체나 펌웨어 갱신 직후라면 토크상수·감속비 설정을 확인하세요.') };
    case 'homing': {
      const bad = a.ch_home
        .map((s, i) => (s === INIT_STATE.fail ? i : -1))
        .filter((i) => i >= 0);
      if (!bad.length) {
        return { title: t('홈 위치를 확인하지 못했습니다'),
                 hint: t('요청된 홈 자세 값이 올바르지 않습니다. 로봇 로그를 확인하세요.') };
      }
      return {
        title: `${bad.map(jointName).join(', ')} ${t('관절이 홈 범위를 벗어났습니다')}`,
        hint: t('로봇을 평평한 바닥에 내려 네 다리를 접은 자세로 만든 뒤 다시 시작하세요.'),
      };
    }
    case 'imu':
      return { title: t('IMU가 연결되지 않았습니다'),
               hint: t('IF 보드 연결을 확인하세요.') };
    default:
      return { title: t('기동에 실패했습니다'), hint: '' };
  }
}


function Cell({ i, state, err, size }: { i: number; state: InitState; err?: number; size: number }) {
  const { c, radius } = useTheme();
  const col = useStateColor()(state);
  const on = state !== INIT_STATE.idle;
  return (
    <View style={[styles.cell, {
      width: size, height: size, borderRadius: radius.sm,
      borderColor: on ? col : c.line,
      backgroundColor: on ? `${col}1F` : 'transparent',
    }]}>
      <Text style={{ fontSize: size > 30 ? 11 : 9.5, fontWeight: '700', color: on ? col : c.dim }}>
        {i >= 12 ? 'W' : AX[i % 3]}
      </Text>
      {err !== undefined && state === INIT_STATE.fail && (
        <Text style={{ fontSize: 7.5, color: col }} numberOfLines={1}>
          {err > 0 ? '+' : ''}{err.toFixed(0)}°
        </Text>
      )}
    </View>
  );
}

function ChannelGrid({ states, errs, wheels, size = 26 }:
  { states: InitState[]; errs?: number[]; wheels: boolean; size?: number }) {
  const { c } = useTheme();
  return (
    <View style={styles.legs}>
      {LEGS.map((lg, legIdx) => (
        <View key={lg.cap} style={styles.leg}>
          <Text style={[styles.legCap, { color: c.muted }]}>{t(lg.name)}</Text>
          <View style={styles.cells}>
            {[lg.base, lg.base + 1, lg.base + 2, ...(wheels ? [MAX_MC + legIdx] : [])].map((i) => (
              <View key={i} style={styles.cellCol}>
                <Cell i={i} size={size} state={states[i] ?? 0} err={errs?.[i]} />
                <Text style={[styles.legNum, { color: c.dim, width: size }]}>{i}</Text>
              </View>
            ))}
          </View>
        </View>
      ))}
    </View>
  );
}

function CheckItem({ label, state }: { label: string; state: InitState }) {
  const { c, radius } = useTheme();
  const col = useStateColor()(state);
  const on = state !== INIT_STATE.idle;
  return (
    <View style={styles.check}>
      <View style={[styles.checkBox, {
        borderRadius: radius.sm, borderColor: on ? col : c.line,
        backgroundColor: on ? `${col}1F` : 'transparent',
      }]}>
        <Text style={{ fontSize: 10, fontWeight: '700', color: on ? col : c.dim }}>
          {state === INIT_STATE.fail ? '✕' : on ? '✓' : ''}
        </Text>
      </View>
      <Text style={{ fontSize: 11, color: on ? c.text : c.dim }}>{t(label)}</Text>
    </View>
  );
}

function GyroAxes({ v }: { v?: number[] }) {
  const { c, radius } = useTheme();
  return (
    <View style={styles.axes}>
      {['X', 'Y', 'Z'].map((ax, i) => {
        const val = v?.[i] ?? 0;
        const bad = val > 1.0;
        return (
          <View key={ax} style={[styles.axis, { borderColor: bad ? c.redTx : c.line, borderRadius: radius.sm }]}>
            <Text style={{ fontSize: 8.5, color: c.dim }}>{ax}</Text>
            <Text style={{ fontSize: 10.5, fontWeight: '600', color: bad ? c.redTx : c.text }}>
              {val.toFixed(2)}
            </Text>
          </View>
        );
      })}
      <Text style={{ fontSize: 9, color: c.dim }}>°/s</Text>
    </View>
  );
}

function Bar({ pct, state, right }: { pct: number; state: InitState; right: string }) {
  const { c, radius } = useTheme();
  const col = useStateColor()(state);
  return (
    <View style={styles.barRow}>
      <View style={[styles.barTrack, { borderColor: c.line, borderRadius: radius.sm }]}>
        <View style={{ width: `${Math.max(0, Math.min(100, pct))}%`, height: '100%', backgroundColor: col }} />
      </View>
      <Text style={{ fontSize: 10.5, color: c.muted, width: 96, textAlign: 'right' }}>{right}</Text>
    </View>
  );
}

const POSE_STEPS = [
  { img: require('@/assets/images/pose-knee.png'),   ratio: 900 / 718, cap: '무릎 조인트를 모두 접으세요' },
  { img: require('@/assets/images/pose-ground.png'), ratio: 900 / 593, cap: '발과 팔꿈치가 바닥에 모두 닿게 해주세요' },
] as const;

function PoseHint() {
  const { c, radius } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const [open, setOpen] = useState(false);
  const colW = Math.min(380, (winW - 96) / 2);
  const imgH = Math.min(winH - 190, colW / POSE_STEPS[0].ratio);
  return (
    <>
      <Tappable onPress={() => setOpen(true)}
                style={[styles.thumb, { borderColor: c.line, borderRadius: radius.sm }]}>
        <View style={styles.thumbRow}>
          {POSE_STEPS.map((p, i) => (
            <Image key={i} source={p.img} style={styles.thumbImg} resizeMode="contain" />
          ))}
        </View>
        <Text style={{ fontSize: 8.5, color: c.muted, textAlign: 'center' }}>{t('그림 보기')}</Text>
      </Tappable>
      {open && (
        <Modal onClose={() => setOpen(false)}>
          <View style={[styles.poseWrap, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
            <Text style={{ fontSize: 13, fontWeight: '700', color: c.text, marginBottom: 10 }}>
              {t('로봇을 평평한 바닥에 두고 아래 자세로 맞춰 주세요')}
            </Text>
            <View style={styles.poseRow}>
              {POSE_STEPS.map((p, i) => (
                <View key={i} style={{ width: colW, gap: 6 }}>
                  <Image source={p.img} resizeMode="contain" style={{ width: colW, height: imgH }} />
                  <Text style={{ fontSize: 11.5, color: c.text, textAlign: 'center' }}>{t(p.cap)}</Text>
                </View>
              ))}
            </View>
          </View>
        </Modal>
      )}
    </>
  );
}

function Row({ label, sub, state, ms, idleLabel, step, onMeasure, children }:
  { label: string; sub: string; state: InitState; ms?: number; idleLabel?: string;
    step?: number; onMeasure?: (step: number, y: number, h: number) => void; children?: ReactNode }) {
  const { c } = useTheme();
  const col = useStateColor()(state);
  const idle = state === INIT_STATE.idle;
  const measure = (e: LayoutChangeEvent) => {
    if (step !== undefined && onMeasure) onMeasure(step, e.nativeEvent.layout.y, e.nativeEvent.layout.height);
  };
  const fold = useContext(FoldCtx);
  const foldable = !!fold && !!children && step !== undefined;
  const open = !foldable || fold!.isOpen(step!, state);
  const Head = foldable ? Tappable : View;
  return (
    <View onLayout={measure} style={[styles.row, { borderTopColor: c.line, opacity: idle ? 0.45 : 1 }]}>
      <Head style={styles.rowHead} {...(foldable ? { onPress: () => fold!.toggle(step!, state) } : {})}>
        {foldable && <Text style={{ fontSize: 10, color: c.dim, width: 14 }}>{open ? '▾' : '▸'}</Text>}
        <View style={{ flex: 1 }}>
          <Text style={{ fontSize: 11.5, fontWeight: '700', color: c.text }}>{t(label)}</Text>
          {!!sub && <Text style={{ fontSize: 9.5, color: c.dim }}>{t(sub)}</Text>}
        </View>
        {ms !== undefined && ms > 0 && (
          <Text style={{ fontSize: 9.5, color: c.dim, marginRight: 8 }}>{(ms / 1000).toFixed(1)}s</Text>
        )}
        <Text style={{ fontSize: 10, fontWeight: '700', color: col }}>
          {idle ? (idleLabel ?? stateLabel(state)) : stateLabel(state)}
        </Text>
      </Head>
      {children && open ? <View style={{ marginTop: 6 }}>{children}</View> : null}
    </View>
  );
}

type Fold = { isOpen: (step: number, state: InitState) => boolean; toggle: (step: number, state: InitState) => void };
const FoldCtx = createContext<Fold | null>(null);


export function AutostartDetail({ variant = 'modern', maxHeight, compact = false, fill = false }:
  { variant?: Variant; maxHeight?: number; compact?: boolean; fill?: boolean }) {
  const { c, radius } = useTheme();
  const a = useRobot((s) => s.robot?.autostart);
  const hasWheel = useFeatureWheel();

  const scRef = useRef<ScrollView>(null);
  const [visH, setVisH] = useState(maxHeight ?? 300);
  const rowPos = useRef<Record<number, { y: number; h: number }>>({});
  const onMeasure = (step: number, y: number, h: number) => { rowPos.current[step] = { y, h }; };
  const curStep = a?.step ?? 0;
  const running = !!a?.running;
  const [flip, setFlip] = useState<Record<number, boolean>>({});
  const autoOpen = (step: number, state: InitState) =>
    (running && step === curStep) || state === INIT_STATE.fail || state === INIT_STATE.warn;
  const fold: Fold | null = compact ? {
    isOpen: (step, state) => flip[step] ?? autoOpen(step, state),
    toggle: (step, state) => setFlip((f) => ({ ...f, [step]: !(f[step] ?? autoOpen(step, state)) })),
  } : null;
  const [contentH, setContentH] = useState(0);
  useEffect(() => {
    if (!running || !scRef.current) return;
    const r = rowPos.current[curStep];
    if (!r) return;
    scRef.current.scrollTo({ y: Math.max(0, r.y + r.h - visH), animated: true });
  }, [curStep, running, visH, contentH]);

  if (!a) return null;

  const fail = failReason(a);
  const imuWarn = a.steps[6] === INIT_STATE.warn;
  const cellSize = variant === 'legacy' ? 24 : 26;

  const isClassic = a.can_fd === false;
  const idleTxt = a.running ? t('대기') : t('미실행');
  const naTxt = t('해당 없음');
  const railTxt = a.leg_rail_v && a.leg_rail_v > 0 ? `${a.leg_rail_v.toFixed(1)} V` : '';
  const accNow = a.acc_norm ?? 0;
  const accWas = a.acc_norm_before ?? 0;
  const gravityText =
    a.acc_pct > 0 && a.acc_pct < 100 ? `${t('보정')} ${a.acc_pct}%`
    : accWas > 0 && Math.abs(accWas - accNow) > 0.005 ? `${accWas.toFixed(2)} → ${accNow.toFixed(2)}`
    : accNow > 0 ? `${accNow.toFixed(2)} m/s²`
    : '—';

  return (
    <View style={[styles.wrap, fill ? { flex: 1, minHeight: 0 } : { flexShrink: 1, minHeight: 0 }, {
      borderColor: c.line, borderRadius: variant === 'legacy' ? 0 : radius.md,
      backgroundColor: variant === 'legacy' ? 'transparent' : c.panel2,
    }]}>
      <View style={styles.head}>
        <Text style={{ fontSize: 10.5, fontWeight: '700', color: c.muted, letterSpacing: 0.6 }}>
          {t('기동 시퀀스')}
        </Text>
        <View style={{ flex: 1 }} />
        <Text style={{ fontSize: 11, color: a.running ? c.accent : c.muted, fontWeight: '600' }}>
          {a.running ? t('진행 중') : fail ? t('중단됨') : t('완료')} · {(a.elapsed_ms / 1000).toFixed(1)}s
        </Text>
      </View>

      <FoldCtx.Provider value={fold}>
      <ScrollView ref={scRef} style={fill ? { flex: 1 } : { maxHeight, flexShrink: 1 }} nestedScrollEnabled
                  scrollEnabled={!a.running} onLayout={(e) => setVisH(e.nativeEvent.layout.height)}
                  onContentSizeChange={(_w, h) => setContentH(h)}>
        <Row label="사전 확인" sub="" state={a.steps[1]} ms={a.step_ms[1]}
             step={1} onMeasure={onMeasure} idleLabel={idleTxt}>
          {a.steps[1] !== INIT_STATE.idle && (
            <CheckItem label="앉은 상태" state={a.pre_standing ? INIT_STATE.fail : INIT_STATE.pass} />
          )}
        </Row>

        <Row label="Leg 전원" state={a.steps[2]} ms={a.step_ms[2]} step={2} onMeasure={onMeasure} idleLabel={idleTxt}
             sub="">
          {a.steps[2] !== INIT_STATE.idle && (
            <Text style={{ fontSize: 11, color: c.muted }}>
              {isClassic ? t('레그 레일')
                         : (a.power_retry > 0 ? t('재시도 1회') : t('한 번에 인가'))}
              {railTxt ? ` · ${railTxt}` : ' —'}
            </Text>
          )}
        </Row>

        <Row label="관절 통신" sub="" state={a.steps[3]} ms={a.step_ms[3]}
             step={3} onMeasure={onMeasure} idleLabel={idleTxt}>
          {a.steps[3] !== INIT_STATE.idle && (
            <ChannelGrid states={a.ch_comm} wheels={hasWheel} size={cellSize} />
          )}
        </Row>

        <Row label="모터 파라미터" state={a.steps[4]} ms={a.step_ms[4]}
             step={4} onMeasure={onMeasure} idleLabel={isClassic ? naTxt : idleTxt}
             sub="">
          {a.steps[4] !== INIT_STATE.idle && (
            <ChannelGrid states={a.ch_param} wheels={hasWheel} size={cellSize} />
          )}
        </Row>

        <Row label="홈 위치" sub="" state={a.steps[5]} ms={a.step_ms[5]}
             step={5} onMeasure={onMeasure} idleLabel={idleTxt}>
          {a.steps[5] !== INIT_STATE.idle && (
            <ChannelGrid states={a.ch_home} errs={a.home_err_deg} wheels={false} size={cellSize} />
          )}
        </Row>

        <Row label="IMU" sub="" state={a.steps[6]} ms={a.step_ms[6]}
             step={6} onMeasure={onMeasure} idleLabel={idleTxt}>
          {a.steps[6] !== INIT_STATE.idle && (
            <View style={{ gap: 6 }}>
              <View style={styles.barRow}>
                <Text style={[styles.gk, { color: c.muted }]}>{t('자이로')}</Text>
                <GyroAxes v={a.gyro_bias_dps} />
                <Text style={{ fontSize: 10.5, color: c.muted }}>
                  {a.gyro_try > 0 ? t('리셋 1회') : t('통과')}
                </Text>
              </View>
              <View style={styles.barRow}>
                <Text style={[styles.gk, { color: c.muted }]}>{t('가속도')}</Text>
                <Bar pct={a.acc_pct} state={a.steps[6]} right={gravityText} />
              </View>
            </View>
          )}
        </Row>

        <Row label="제어 활성" sub="" state={a.steps[7]} ms={a.step_ms[7]}
             step={7} onMeasure={onMeasure} idleLabel={idleTxt}>
          {a.emo_blocked && (
            <Text style={{ fontSize: 11, color: c.amberTx }}>
              {t('E-STOP이 눌려 있어 제어가 시작되지 않았습니다 — 해제하면 바로 시작됩니다.')}
            </Text>
          )}
        </Row>

      {fail && (
        <View style={[styles.note, { borderColor: c.redTx, backgroundColor: 'rgba(231,51,28,0.10)' }]}>
          <View style={styles.noteRow}>
            <View style={{ flex: 1 }}>
              <Text style={{ fontSize: 12, fontWeight: '700', color: c.redTx }}>{fail.title}</Text>
              {!!fail.hint && <Text style={{ fontSize: 11, color: c.text, marginTop: 3 }}>{fail.hint}</Text>}
              {a.fail_code > 0 && (
                <Text style={{ fontSize: 9.5, color: c.dim, marginTop: 4 }}>{t('오류 코드')} {a.fail_code}</Text>
              )}
            </View>
            {INIT_STEPS[a.step] === 'homing' && <PoseHint />}
          </View>
        </View>
      )}

      {!fail && imuWarn && (
        <View style={[styles.note, { borderColor: c.amberTx, backgroundColor: 'rgba(240,136,62,0.10)' }]}>
          {a.gyro_warn !== false && (
            <Text style={{ fontSize: 11.5, fontWeight: '700', color: c.amberTx }}>
              {t('자이로 편차가 기준을 넘습니다')} ({Math.max(...(a.gyro_bias_dps ?? [0])).toFixed(2)}°/s)
            </Text>
          )}
          {a.acc_warn && (
            <Text style={{ fontSize: 11.5, fontWeight: '700', color: c.amberTx }}>
              {t('가속도 크기가 정상 범위를 벗어납니다')} ({(a.acc_norm ?? 0).toFixed(2)} m/s²)
            </Text>
          )}
          <Text style={{ fontSize: 11, color: c.text, marginTop: 3 }}>
            {t('기동은 계속됩니다. 흔들리지 않는 평평한 바닥에서 다시 시작하면 사라집니다.')}
          </Text>
        </View>
      )}
      {!fail && a.can_mode_warn && (
        <View style={[styles.note, { borderColor: c.amberTx, backgroundColor: 'rgba(240,136,62,0.10)' }]}>
          <Text style={{ fontSize: 11, color: c.amberTx }}>
            {t('모터 CAN 모드가 Classic이라 CAN-FD로 다시 설정했습니다.')}
          </Text>
        </View>
      )}
      </ScrollView>
      </FoldCtx.Provider>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { borderWidth: 1, padding: 10, gap: 2, width: '100%' },
  head: { flexDirection: 'row', alignItems: 'center', paddingBottom: 6 },
  row: { borderTopWidth: 1, paddingVertical: 7 },
  rowHead: { flexDirection: 'row', alignItems: 'center' },
  legs: { flexDirection: 'row', gap: 8, alignItems: 'flex-end' },
  leg: { gap: 3 },
  legCap: { fontSize: 9, fontWeight: '700', textAlign: 'center' },
  cellCol: { alignItems: 'center', gap: 1 },
  legNum: { fontSize: 7.5, textAlign: 'center' },
  checks: { flexDirection: 'row', gap: 18 },
  check: { flexDirection: 'row', alignItems: 'center', gap: 5 },
  checkBox: { width: 15, height: 15, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  cells: { flexDirection: 'row', gap: 3 },
  cell: { borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  barRow: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  barTrack: { flex: 1, height: 12, borderWidth: 1, overflow: 'hidden' },
  gk: { fontSize: 10, width: 34 },
  axes: { flexDirection: 'row', alignItems: 'center', gap: 3 },
  axis: { borderWidth: 1, paddingHorizontal: 4, paddingVertical: 2, alignItems: 'center' },
  note: { borderWidth: 1, borderRadius: 4, padding: 8, marginTop: 8 },
  noteRow: { flexDirection: 'row', alignItems: 'flex-start', gap: 10 },
  thumb: { width: 116, borderWidth: 1, padding: 3, gap: 2 },
  thumbRow: { flexDirection: 'row', gap: 3 },
  thumbImg: { flex: 1, height: 44 },
  poseWrap: { padding: 16, borderWidth: 1, alignItems: 'center' },
  poseRow: { flexDirection: 'row', gap: 16, alignItems: 'flex-start' },
});
