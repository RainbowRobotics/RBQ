import { useEffect, useRef, useState, type ReactNode } from 'react';
import { View, Text, StyleSheet, ScrollView, Image, useWindowDimensions } from 'react-native';
import { create } from 'zustand';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Modal } from '@/components/ui/overlays';
import { Toggle } from '@/components/ui/controls';
import { useVisionToggles } from '@/store/visionToggles';
import { useHasProjector } from '@/store/capability';
import { t } from '@/lib/i18n';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { CameraView } from '@/components/CameraView';
import { webrtcClient } from '@/lib/webrtcClient';
import { CAMERA_CHECK_STREAM_ID } from '@/lib/visionSources';
import { useCameraLive, startCameraLive, stopCameraLive, setCameraLiveViews, useDepthCheck, startDepthCheck, useUserExtrinsic, startUserExtrinsic } from '@/lib/cameraCheck';
import { CAMERA_NAMES, CHECK_VIEW_COLS, CHECK_VIEW_ROWS, STREAM_KEYS, cameraName, defaultView, depthVerdict, emitCell, occCell, beforeCell, statusCell, streamLabel, usbLabel } from '@/lib/cameraCheckView';
import type { CameraLiveSensor, CameraLiveView, CameraStreamKey, DepthCheckSensor, UserExtrinsic, UserExtrinsicAction, UserExtrinsicSensor } from '@/lib/rest';

export function Frame({ title, width, onClose, children, head, actions, closeLabel, largeTitle, closeHidden }: {
  title: string; width: number; onClose: () => void; children: React.ReactNode; head?: React.ReactNode; actions?: React.ReactNode; closeLabel?: string; largeTitle?: boolean;
  closeHidden?: boolean;
}) {
  const { c, radius } = useTheme();
  const { width: winW, height: winH } = useWindowDimensions();
  const close = (
    <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }, actions ? styles.btnSm : null]}>
      <Text style={{ color: c.text, fontSize: 14, fontWeight: actions ? '700' : '600' }}>{closeLabel ?? t('닫기')}</Text>
    </Tappable>
  );
  return (
    <Modal onClose={onClose}>
      <View style={[styles.box, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg, width: Math.min(winW - 24, width) },
        actions ? { maxHeight: winH - 24 } : null]}>
        <Text style={{ color: c.text, fontSize: head || largeTitle ? 18 : 16, fontWeight: '700' }}>{title}</Text>
        {head}
        {actions ? (
          <>
            <ScrollView style={{ flexShrink: 1 }} contentContainerStyle={{ gap: 12 }}>{children}</ScrollView>
            <View style={[styles.uxbtns, { borderTopWidth: StyleSheet.hairlineWidth, borderTopColor: c.line, paddingTop: 12 }]}>
              {!closeHidden && close}
              <View style={{ flex: 1 }} />
              {actions}
            </View>
          </>
        ) : (
          <>
            {children}
            {close}
          </>
        )}
      </View>
    </Modal>
  );
}

const VIDEO_CHROME = 160, VIDEO_CHROME_LOW = 142, VIDEO_MIN_H = 160, FRAME_INSET_W = 38, VIDEO_LOW_SCREEN = 480;

export function CameraBasicCheckModal({ onClose }: { onClose: () => void }) {
  const { c } = useTheme();
  const ip = useRobot((s) => s.ip);
  const live = useCameraLive((s) => s.live);
  const error = useCameraLive((s) => s.error);
  const [views, setViews] = useState<CameraLiveView[]>(() => Array(CAMERA_NAMES.length).fill('auto'));
  const [focus, setFocus] = useState<number | null>(null);
  const { width: winW, height: winH } = useWindowDimensions();
  const [prevSource] = useState(() => webrtcClient.sourceId());
  const vt = useVisionToggles();
  const hasProjector = useHasProjector();

  useEffect(() => {
    startCameraLive(ip);
    return () => {
      stopCameraLive(ip);
      if (prevSource != null && prevSource !== CAMERA_CHECK_STREAM_ID) webrtcClient.setSource(prevSource);
    };
  }, [ip, prevSource]);
  useEffect(() => {
    if (!live) return;
    const next = views.map((v, i) => {
      const avail = Object.keys(live.sensors[i]?.fps ?? {}) as CameraStreamKey[];
      return v === 'auto' || !avail.includes(v) ? defaultView(avail) : v;
    });
    if (next.some((v, i) => v !== views[i])) setViews(next);
  }, [live, views]);
  const focusIdx = focus ?? -1;
  useEffect(() => {
    if (!live) return;
    const sameFocus = live.focus === undefined || live.focus === focusIdx;
    if (sameFocus && views.every((v, i) => v === 'auto' || live.sensors[i]?.view === v)) return;
    setCameraLiveViews(ip, views, focusIdx);
  }, [live, views, focusIdx, ip]);

  const canFocus = live?.focus !== undefined;
  const status = error || (live && !live.hal ? t('카메라 상태를 받지 못했습니다 — HAL 이 실행 중인지 확인하세요')
    : canFocus ? t('칸마다 위의 버튼으로 볼 영상(RGB·IR·Depth)을 고르고, 칸을 누르면 크게 봅니다')
    : t('칸마다 위의 버튼으로 볼 영상(RGB·IR·Depth)을 고르세요'));
  const lowScreen = winH < VIDEO_LOW_SCREEN;
  const videoH = Math.max(VIDEO_MIN_H, winH - 24 - (lowScreen ? VIDEO_CHROME_LOW : VIDEO_CHROME));
  const videoW = Math.min((videoH * 16) / 9, Math.min(winW - 24, winW * 0.94, 980) - FRAME_INSET_W);
  return (
    <Frame title={focus == null ? t('카메라 실시간 화면') : `${t('카메라 실시간 화면')} · ${CAMERA_NAMES[focus]}`} width={videoW + FRAME_INSET_W}
      onClose={focus == null ? onClose : () => setFocus(null)} closeLabel={focus == null ? undefined : t('전체 보기')} largeTitle>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 12 }}>
        <View style={{ flex: 1, minWidth: 0, gap: 2 }}>
          {!lowScreen && <Text style={{ color: c.text, fontSize: 13 }}>{t('카메라마다 시야가 가려지거나 흐리지 않은지 보고, FPS 가 제대로 나오는지 확인하세요.')}</Text>}
          <Text style={{ color: error || vt.err || (live && !live.hal) ? c.amberTx : c.text, fontSize: 13 }}>{vt.err || status}</Text>
        </View>
        {hasProjector && (
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
            <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{t('IR 프로젝터 (전체)')}</Text>
            <Toggle value={vt.irProjector} onChange={(v) => vt.toProjector(ip, v)} />
          </View>
        )}
      </View>
      <View style={[styles.video, { borderColor: c.line }]}>
        <CameraView streamId={CAMERA_CHECK_STREAM_ID} />
        {CAMERA_NAMES.map((name, i) => (focus == null || focus === i) && (
          <CheckTile key={i} index={i} name={name} sensor={live?.sensors[i]} view={views[i]} full={focus === i}
            onPress={canFocus && focus == null ? () => setFocus(i) : undefined}
            onView={(v) => setViews((cur) => cur.map((x, k) => (k === i ? v : x)))} />
        ))}
      </View>
    </Frame>
  );
}

function CheckTile({ index, name, sensor, view, onView, onPress, full }: {
  index: number; name: string; sensor: CameraLiveSensor | undefined; view: CameraLiveView;
  onView: (v: CameraStreamKey) => void; onPress?: () => void; full?: boolean;
}) {
  const { c, fonts } = useTheme();
  const avail = STREAM_KEYS.filter((k) => sensor?.fps[k] != null);
  const st = sensor ? statusCell(sensor.status) : null;
  const usb = usbLabel(sensor?.usb);
  const emit = sensor ? emitCell(sensor.status) : 'N';
  const pos = full ? { left: 0, top: 0, width: '100%', height: '100%' } as const : {
    left: `${((index % CHECK_VIEW_COLS) * 100) / CHECK_VIEW_COLS}%`,
    top: `${(Math.floor(index / CHECK_VIEW_COLS) * 100) / CHECK_VIEW_ROWS}%`,
    width: `${100 / CHECK_VIEW_COLS}%`, height: `${100 / CHECK_VIEW_ROWS}%`,
  } as const;
  return (
    <View style={[styles.tile, pos]} pointerEvents="box-none">
      {onPress && <Tappable onPress={onPress} accessibilityLabel={name} scaleTo={1} style={StyleSheet.absoluteFill}><View /></Tappable>}
      <View style={styles.tileHead}>
        <View style={styles.namePill}><Text style={[styles.tileName, { fontFamily: fonts.mono }]}>{name}</Text></View>
        {emit !== 'N' && <>
          <View style={{ flex: 1 }} />
          <View style={[styles.chip, emit === 'ON' && { borderColor: OV.on, backgroundColor: 'rgba(63,185,80,0.22)' }]}>
            <Text style={[styles.chipTx, { fontFamily: fonts.mono, color: emit === 'ON' ? OV.on : OV.dim }]}>EMIT {emit}</Text>
          </View>
        </>}
        {avail.length > 1
          ? <View style={styles.seg}>
            {avail.map((k, i) => {
              const on = (view === 'auto' ? avail[0] : view) === k;
              return (
                <Tappable key={k} onPress={() => onView(k)} accessibilityLabel={`${name} ${streamLabel(k)}`} scaleTo={1}
                  style={[styles.segItem, i > 0 && styles.segDivider, on && { backgroundColor: c.accent }]}>
                  <Text style={[styles.toggleTx, on && styles.toggleOn]}>{streamLabel(k)}</Text>
                </Tappable>
              );
            })}
          </View>
          : avail.length === 1 && <Text style={[styles.tileOnly, { marginLeft: 'auto' }]}>{streamLabel(avail[0])}</Text>}
      </View>
      <View style={styles.tileFoot} pointerEvents="none">
        {avail.length === 0 && <Text style={styles.fpsUnit}>— FPS</Text>}
        {avail.map((k) => (
          <View key={k} style={styles.fpsItem}>
            <Text style={styles.fpsKind}>{streamLabel(k)}</Text>
            <Text style={[styles.fpsVal, { fontFamily: fonts.mono }, sensor!.fps[k] === 0 && { color: OV.bad }]}>{sensor!.fps[k]}</Text>
          </View>
        ))}
        {avail.length > 0 && <Text style={styles.fpsUnit}>FPS</Text>}
        <View style={{ flex: 1 }} />
        {st && st.text !== 'RUN' && <View style={[styles.chip, { borderColor: OV.warn }]}><Text style={[styles.chipTx, { color: OV.warn }]}>{st.text}</Text></View>}
        {!!usb && (
          <View style={[styles.chip, usb === '2.0' && { borderColor: OV.warn }]}>
            <Text style={[styles.chipTx, { fontFamily: fonts.mono }, usb === '2.0' && { color: OV.warn }]}>USB {usb}</Text>
          </View>
        )}
      </View>
    </View>
  );
}

function StepBar({ labels, cur, warn, warnOn }: { labels: string[]; cur: number; warn: string; warnOn: boolean }) {
  const { c } = useTheme();
  return (
    <View style={[styles.uxsteps, { borderBottomColor: c.line }]}>
      {labels.map((label, i) => {
        const done = i < cur, on = i === cur;
        return (
          <View key={label} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
            {i > 0 && <View style={{ width: 28, height: 1.5, backgroundColor: c.line }} />}
            <View style={[styles.uxdot, styles.uxdotLg, { backgroundColor: done ? c.green : 'transparent', borderColor: done ? c.green : on ? c.accent : c.line }]}>
              <Text style={{ color: done ? '#fff' : on ? c.accent : c.muted, fontSize: 13, fontWeight: '800' }}>{done ? '✓' : i + 1}</Text>
            </View>
            <Text style={{ color: on ? c.text : c.muted, fontSize: 16, fontWeight: on ? '700' : '600' }}>{t(label)}</Text>
          </View>
        );
      })}
      <View style={{ flex: 1 }} />
      <Text style={{ color: warnOn ? c.amberTx : c.muted, fontSize: 13, fontWeight: warnOn ? '700' : '500' }}>⚠ {warn}</Text>
    </View>
  );
}

const OCC_GUIDE = [
  ['로봇을 평평한 바닥에 세워 두세요.', '보정할 카메라를 고른 뒤 캘리브레이션 시작을 누르세요.'],
  ['측정한 적 없는 카메라는 먼저 측정합니다.', '이어서 로봇이 다리를 벌립니다. 끝나면 바로 보정으로 넘어갑니다.'],
  ['카메라마다 몸을 기울여 보정합니다.', 'good 이 나오지 않으면 그 카메라를 한 번 더 보정합니다(최대 2회).'],
  ['저장할 카메라는 Apply, 다시 보정할 카메라는 Retry 를 누르세요.', '다 고른 뒤 완료를 누르면 로봇이 기본 자세로 돌아옵니다. 그 다음 창을 닫으세요.'],
];
const OCC_STEPS = ['카메라 선택', '선행 점검', 'On-Chip 보정', '결과'];

export function SimFakeNote() {
  const { c } = useTheme();
  return <Text style={{ color: c.amberTx, fontSize: 13, fontWeight: '700' }}>{t('시뮬 가짜 결과 — 실제 카메라 측정 · OCC 가 아닙니다')}</Text>;
}

const useOnChipOpen = create<{ count: number }>(() => ({ count: 0 }));

export function OnChipBanner() {
  const { radius } = useTheme();
  const depth = useDepthCheck((s) => s.depth);
  const open = useOnChipOpen((s) => s.count > 0);
  const [reopen, setReopen] = useState(false);
  const active = depth?.action === 'on_chip' && depth.state !== 'idle';
  if (reopen) return <DepthOnChipModal onClose={() => setReopen(false)} />;
  if (!active || open) return null;
  const tone = '#FBBA16';
  const label = t('카메라 점검 2 진행 중 (조이스틱 잠금)');
  return (
    <Tappable onPress={() => setReopen(true)} style={[styles.banner, { borderColor: tone, borderRadius: radius.md }]}>
      <Icon name="track" size={14} color={tone} />
      <Text style={{ color: tone, fontSize: 11.5, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
}

export function DepthOnChipModal({ onClose }: { onClose: () => void }) {
  const { c, fonts, radius } = useTheme();
  useEffect(() => {
    useOnChipOpen.setState((s) => ({ count: s.count + 1 }));
    return () => useOnChipOpen.setState((s) => ({ count: s.count - 1 }));
  }, []);
  const ip = useRobot((s) => s.ip);
  const depth = useDepthCheck((s) => s.depth);
  const error = useDepthCheck((s) => s.error);
  const last = (id: number) => depth?.sensors.find((s) => s.id === id);
  const [picked, setPicked] = useState<number[]>(() =>
    depth?.sensors.length ? DEPTH_SENSORS.filter((id) => last(id)?.needs_occ) : [...DEPTH_SENSORS]);
  const running = depth?.state === 'running';
  const awaiting = depth?.state === 'awaiting';
  const [runIds, setRunIds] = useState<number[] | null>(() =>
    (running || awaiting) && depth?.action === 'on_chip' ? depth.sensors.map((s) => s.id) : null);
  const robot = useTelemetry((s) => s.robot);
  const sitting = !robot?.isStanding;
  const shown = runIds != null && depth != null && depth.action === 'on_chip';
  const rows = shown ? depth.sensors.filter((s) => runIds.includes(s.id)) : [];
  const saved = rows.some((s) => s.occ === 'saved');
  const worse = awaiting && rows.some((s) => s.occ === 'pending' && !s.improved);
  const close = () => { if (awaiting) void startDepthCheck(ip, 'close'); onClose(); };
  const run = (ids: number[]) => { setRunIds(ids); void startDepthCheck(ip, 'on_chip', ids); };
  const stepKey = running && depth.current ? `${depth.current.sensor}:${depth.current.step}` : '';
  const stepStart = useRef({ key: '', at: 0 });
  if (stepStart.current.key !== stepKey) stepStart.current = { key: stepKey, at: Date.now() };

  const stepText = (step: string) =>
    step === 'pose' ? t('자세 잡는 중')
      : step === 'on_chip' ? t('OCC 계산 중 {s}초').replace('{s}', String(Math.round((Date.now() - stepStart.current.at) / 1000)))
      : step === 'apply' ? t('적용 · 재측정') : step === 'revert' ? t('되돌리는 중') : step === 'save' ? t('저장 중') : t('측정 중');
  const pct = (b: number, a: number) => (b > 0 ? `${a <= b ? '−' : '+'}${Math.abs(((a - b) / b) * 100).toFixed(0)}%` : '');
  const rowBtn = (label: string, primary: boolean, onPress: () => void) => (
    <Tappable onPress={onPress} style={[styles.rowBtn, { borderRadius: radius.md, backgroundColor: primary ? c.accent : c.elev, borderColor: primary ? c.accent : c.line }]}>
      <Text style={{ color: primary ? '#fff' : c.text, fontSize: 13, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
  const row = (s: DepthCheckSensor) => {
    const now = running && depth!.current?.sensor === s.id ? depth!.current : null;
    const b = s.measured?.frames ? s.measured : null, a = s.after?.frames ? s.after : null;
    const cmp = (k: 'subpixel_rms' | 'plane_rms_mm', d: number) => !b ? '—'
      : !a ? b[k].toFixed(d) : `${b[k].toFixed(d)} → ${a[k].toFixed(d)}  ${pct(b[k], a[k])}`;
    const fill = !b ? '—' : !a ? `${b.fill_rate.toFixed(1)}%`
      : `${b.fill_rate.toFixed(1)} → ${a.fill_rate.toFixed(1)}%  ${a.fill_rate >= b.fill_rate ? '+' : '−'}${Math.abs(a.fill_rate - b.fill_rate).toFixed(1)}`;
    const better = s.occ === 'saved' ? true : s.occ === 'kept' ? false : null;
    const status = now ? { text: stepText(now.step), color: c.accent2 }
      : s.result === 'running' || s.occ === 'running' ? { text: t('대기'), color: c.dim }
      : s.result !== 'ok' ? { text: depthVerdict(s).text, color: depthVerdict(s).tone === 'off' ? c.dim : c.redTx }
      : s.occ ? { text: occCell(s).text, color: occCell(s).tone === 'ok' ? c.greenTx : occCell(s).tone === 'fail' ? c.redTx : c.text }
      : { text: '—', color: c.dim };
    return (
      <View key={s.id} style={[styles.drow, { borderTopWidth: StyleSheet.hairlineWidth, borderTopColor: c.line }]}>
        <Text style={[styles.dcell, { flex: OCC_COL_FLEX[0], color: c.text, fontFamily: fonts.mono }]}>{cameraName(s.id)}</Text>
        <Text style={[styles.dcell, { flex: OCC_COL_FLEX[5], color: beforeCell(s).tone === 'ok' ? c.greenTx : beforeCell(s).tone === 'warn' ? c.amberTx : c.dim, fontWeight: '700' }]}>{beforeCell(s).text}</Text>
        <Text style={[styles.dcell, { flex: OCC_COL_FLEX[7], color: a && b && a.fill_rate < b.fill_rate - 1 ? c.amberTx : c.text, fontFamily: fonts.mono }]}>{fill}</Text>
        <Text style={[styles.dcell, { flex: OCC_COL_FLEX[1], color: better == null ? c.text : better ? c.greenTx : c.amberTx, fontFamily: fonts.mono }]}>{cmp('subpixel_rms', 3)}</Text>
        <Text style={[styles.dcell, { flex: OCC_COL_FLEX[2], color: c.text, fontFamily: fonts.mono }]}>{cmp('plane_rms_mm', 2)}</Text>
        <Text style={[styles.dcell, { flex: OCC_COL_FLEX[3], color: c.text, fontFamily: fonts.mono }]}>{s.health ? s.health.toFixed(3) : '—'}</Text>
        <Text style={[styles.dcell, { flex: OCC_COL_FLEX[4], color: status.color, fontWeight: '700' }]} numberOfLines={2}>
          {status.text}{!!s.attempts && <Text style={{ color: c.muted, fontWeight: '400' }}>{'  ' + t('{n}회 시도').replace('{n}', String(s.attempts))}</Text>}
        </Text>
        <View style={{ flex: OCC_COL_FLEX[6], flexDirection: 'row', justifyContent: 'flex-end', gap: 6 }}>
          {awaiting && (s.occ === 'pending' ? rowBtn('Apply', true, () => { void startDepthCheck(ip, 'apply', [s.id]); }) : <View style={styles.rowBtnSlot} />)}
          {awaiting && s.occ !== 'saved' && rowBtn('Retry', false, () => { void startDepthCheck(ip, 'retry', [s.id]); })}
        </View>
      </View>
    );
  };
  const failed = shown && !running && rows.some((s) => s.result === 'failed' || s.result === 'pose_failed' || occCell(s).tone === 'fail');
  const cur = depth?.current?.step;
  const poseIdx = !shown ? -1 : awaiting ? 2 : running
    ? (cur === 'premeasure' ? -1 : cur === 'wide' ? 0 : cur === 'end' ? 3 : 1)
    : depth?.action === 'on_chip' && rows.some((s) => s.attempts) ? 4 : -1;
  const poseChips = [t('넓게 서기'), t('보정 자세'), t('선택 대기'), t('자세 복귀')];
  const stepIdx = !shown ? 0 : !running || cur === 'end' ? 3 : cur === 'premeasure' || cur === 'wide' ? 1 : 2;
  const unsaved = rows.some((s) => s.occ === 'pending');
  const note = error || (awaiting ? (worse ? t('나아지지 않은 후보가 있습니다. Apply 해도 나빠질 수 있습니다. Retry 로 다시 시도해 보세요.')
      : unsaved ? t('Apply 하지 않은 카메라는 완료를 누르면 원래 값으로 되돌립니다.') : '')
    : running ? ''
    : failed ? t('실패한 카메라는 바닥이 잘 보이는지(렌즈 가림·앉음) 확인한 뒤 다시 실행하세요')
    : !shown && sitting ? t('앉아 있음 — 세운 뒤 측정하세요')
    : saved ? t('좋아진 보정값을 카메라에 저장했습니다 — 로봇을 다시 켜도 유지됩니다')
    : '');
  const noteTone: 'warn' | 'ok' | 'info' = error || worse || failed || (!shown && sitting) ? 'warn'
    : saved ? 'ok' : 'info';
  const btn = (label: string, enabled: boolean, onPress: () => void) => (
    <Tappable key={label} onPress={enabled ? onPress : undefined}
      style={[styles.btn, styles.btnSm, { backgroundColor: c.accent, borderColor: c.accent, borderRadius: radius.md, opacity: enabled ? 1 : 0.4 }]}>
      <Text style={{ color: '#fff', fontSize: 14, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
  const returning = running && cur === 'end';
  const actions = (
    <View style={{ flexDirection: 'row', gap: 8 }}>
      {running && shown && !returning && btn(t('중지하고 복귀'), true, () => { void startDepthCheck(ip, 'stop'); })}
      {awaiting ? btn(t('완료'), true, () => { void startDepthCheck(ip, 'close'); })
        : returning ? btn(t('자세 복귀 중'), false, () => {})
        : shown ? btn(t('재보정'), !running, () => setRunIds(null))
        : btn(t('캘리브레이션 시작'), !running && !sitting && picked.length > 0, () => run(picked))}
    </View>
  );
  return (
    <Frame title={t('하단 depth 카메라 On-chip 캘리브레이션')} width={1040} onClose={close} closeHidden={awaiting || returning}
      head={<StepBar labels={OCC_STEPS} cur={stepIdx} warn={t('로봇이 움직입니다 · 조이스틱 잠금')} warnOn={running || awaiting} />} actions={actions}>
      <View style={{ gap: 4 }}>
        {OCC_GUIDE[stepIdx].map((line) => (
          <View key={line} style={{ flexDirection: 'row', gap: 8 }}>
            <Text style={{ color: c.accent, fontSize: 15, lineHeight: 22, fontWeight: '700' }}>•</Text>
            <Text style={{ color: c.text, fontSize: 15, lineHeight: 22, flex: 1 }}>{t(line)}</Text>
          </View>
        ))}
        {stepIdx === 0 && <Text style={{ color: c.amberTx, fontSize: 15, lineHeight: 22, fontWeight: '700' }}>⚠ {t('보정이 끝날 때까지 로봇이 움직이고, 조이스틱은 동작하지 않습니다.')}</Text>}
      </View>
      {depth?.sim && <SimFakeNote />}
      {shown
        ? <View style={{ gap: 4 }}>
          <View style={styles.drow}>
            {[[t('카메라'), 0], [t('보정 전 판정'), 5], [t('FILL 전 → 후'), 7], [t('SUBPIXEL 전 → 후'), 1], [t('PLANE RMS 전 → 후'), 2], ['HEALTH', 3], [t('상태'), 4], ['', 6]].map(([h, i]) => (
              <Text key={h} style={[styles.dcell, { flex: OCC_COL_FLEX[i as number], color: c.muted, fontSize: 11, fontWeight: '700' }]}>{h}</Text>
            ))}
          </View>
          {rows.map(row)}
        </View>
        : <View style={{ flexDirection: 'row', gap: 8 }}>
          {DEPTH_SENSORS.map((id) => {
            const on = picked.includes(id);
            const v = last(id) ? depthVerdict(last(id)!) : null;
            return (
              <Tappable key={id} onPress={() => setPicked((p) => (on ? p.filter((x) => x !== id) : [...p, id].sort()))}
                style={[styles.pick, { borderColor: on ? c.amber : c.line, borderWidth: on ? 2.5 : 1, backgroundColor: c.elev, borderRadius: radius.sm }]}>
                <Text style={{ color: c.text, fontSize: 12.5, fontWeight: '700', fontFamily: fonts.mono }}>{cameraName(id)}</Text>
                {v && <Text style={{ color: v.tone === 'warn' ? c.amberTx : c.dim, fontSize: 11 }}>{v.text}</Text>}
              </Tappable>
            );
          })}
        </View>}
      {shown && poseIdx >= 0 && (
        <View style={{ backgroundColor: c.elev, borderRadius: radius.sm, padding: 12, flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap' }}>
          <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('로봇 자세')}</Text>
          {poseChips.map((x, i) => (
            <View key={i} style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
              {i > 0 && <Text style={{ color: c.muted }}>→</Text>}
              <View style={[styles.uxchip, { borderColor: i < poseIdx ? c.green : i === poseIdx ? c.accent2 : c.line }]}>
                <Text style={{ color: i < poseIdx ? c.greenTx : i === poseIdx ? c.accent2 : c.muted, fontSize: 13, fontWeight: '700' }}>{x}</Text>
              </View>
            </View>
          ))}
        </View>
      )}
      {!!note && (
        <Text style={{ color: noteTone === 'ok' ? c.greenTx : noteTone === 'warn' ? c.amberTx : c.text, fontSize: 15, lineHeight: 22, fontWeight: noteTone === 'info' ? '500' : '700' }}>
          {noteTone === 'warn' ? '⚠ ' : ''}{note}
        </Text>
      )}
    </Frame>
  );
}
const DEPTH_SENSORS = [0, 1, 2, 3] as const;
const OCC_COL_FLEX = [0.9, 1.9, 1.8, 0.7, 1.6, 1.1, 1.7, 1.7];


const UX_PARENT: Record<number, number> = { 0: 1, 1: 2, 3: 2 };
const UX_ROWS = [0, 1, 3];
const UX_PAIRS = [1, 3, 0];
type UxAxis = 'roll' | 'pitch' | 'z' | 'x' | 'y' | 'yaw';
const UX_UNIT: Record<UxAxis, string> = { roll: '°', pitch: '°', yaw: '°', x: ' mm', y: ' mm', z: ' mm' };

function uxValue(s: UserExtrinsicSensor | undefined, k: UxAxis): number | undefined {
  if (!s) return undefined;
  return { roll: s.tilt_roll, pitch: s.tilt_pitch, z: s.id === 2 ? undefined : s.height_mm, x: s.cross_x_mm, y: s.cross_y_mm, yaw: s.cross_yaw_deg }[k];
}
function uxLimit(ux: UserExtrinsic, k: UxAxis): number | undefined {
  const l = ux.limits;
  return l && { roll: l.roll_deg, pitch: l.pitch_deg, z: l.z_mm, x: l.x_mm, y: l.y_mm, yaw: l.yaw_deg }[k];
}
function uxShown(ux: UserExtrinsic, s: UserExtrinsicSensor | undefined, k: UxAxis) {
  if (!s || s.result !== 'ok') return undefined;
  if ((k === 'roll' || k === 'pitch' || k === 'z') && (!ux.ref_ok || s.id === 2)) return undefined;
  return uxValue(s, k);
}
const uxOver = (ux: UserExtrinsic, s: UserExtrinsicSensor | undefined, k: UxAxis) => {
  const v = uxShown(ux, s, k), lim = uxLimit(ux, k);
  return v != null && lim != null && Math.abs(v) > lim;
};
export function uxOverAxes(ux: UserExtrinsic, s: UserExtrinsicSensor | undefined): UxAxis[] {
  return (['roll', 'pitch', 'z', 'x', 'y', 'yaw'] as UxAxis[]).filter((k) => uxOver(ux, s, k));
}
const uxFmt = (k: UxAxis, v: number | undefined) => {
  if (v == null) return '—';
  const r = Math.round(v * 10) / 10 || 0;
  return `${r >= 0 ? '+' : '−'}${Math.abs(r).toFixed(1)}${UX_UNIT[k]}`;
};
export function uxBoardsOk(ux: UserExtrinsic, id: number): boolean | undefined {
  const s = ux.sensors.find((x) => x.id === id);
  if (!s || s.result === 'running') return undefined;
  const used = ux.on_board ? s.boards_used ?? [] : [];
  if (id === 2) return used.length > 0;
  const p = ux.sensors.find((x) => x.id === UX_PARENT[id])?.boards_used ?? [];
  return used.some((b) => p.includes(b));
}
const uxFail = (s: UserExtrinsicSensor) =>
  s.result === 'not_streaming' ? t('꺼짐') : s.result === 'timeout' ? t('응답 없음') : s.result === 'no_board' ? t('보드 안 보임') : t('평면 없음');

export function UxCamTiles({ ux, mode }: { ux: UserExtrinsic | null; mode: 'boards' | 'verdict' }) {
  const { c, fonts, radius } = useTheme();
  const tone = { ok: { bg: 'rgba(63,185,80,0.10)', line: 'rgba(63,185,80,0.5)', tx: c.greenTx }, warn: { bg: 'rgba(210,153,34,0.10)', line: 'rgba(210,153,34,0.5)', tx: c.amberTx },
    none: { bg: c.elev, line: c.line, tx: c.muted } };
  return (
    <View style={{ flexDirection: 'row', gap: 8 }}>
      {[0, 1, 2, 3].map((id) => {
        const s = ux?.sensors.find((x) => x.id === id);
        let word = s?.result === 'running' ? t('측정 중..') : '—', sub = '', k: keyof typeof tone = 'none';
        if (ux && s && s.result !== 'running') {
          if (mode === 'boards') {
            const ok = uxBoardsOk(ux, id);
            if (ok != null) { word = ok ? t('보드 보임') : t('보드 부족'); k = ok ? 'ok' : 'warn'; }
            else if (s.result !== 'ok') { word = uxFail(s); k = 'warn'; }
          } else if (s.result !== 'ok') { word = uxFail(s); k = 'warn'; }
          else {
            const axes = uxOverAxes(ux, s);
            word = s.needs_align ? t('보정 필요') : t('정상');
            k = s.needs_align ? 'warn' : 'ok';
            sub = axes.join(' · ');
          }
        }
        return (
          <View key={id} style={[styles.uxtile, { backgroundColor: tone[k].bg, borderColor: tone[k].line, borderRadius: radius.sm }]}>
            <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', fontFamily: fonts.mono }}>{cameraName(id)}</Text>
            <Text style={{ color: tone[k].tx, fontSize: 14, fontWeight: '700' }}>{word}</Text>
            {!!sub && <Text style={{ color: c.muted, fontSize: 12 }} numberOfLines={1}>{sub}</Text>}
          </View>
        );
      })}
    </View>
  );
}

export function UxTables({ ux, before, verdict }: { ux: UserExtrinsic; before?: UserExtrinsicSensor[]; verdict?: boolean }) {
  const { c, fonts } = useTheme();
  const get = (list: UserExtrinsicSensor[] | undefined, id: number) => list?.find((x) => x.id === id);
  const cell = (s: UserExtrinsicSensor | undefined, k: UxAxis, flex: number) => {
    const v = uxShown(ux, s, k);
    const b = before ? uxShown(ux, get(before, s?.id ?? -1), k) : undefined;
    const changed = b != null && uxFmt(k, b) !== uxFmt(k, v);
    return (
      <View key={k} style={{ flex, alignItems: 'flex-end' }}>
        {changed && <Text style={{ color: c.dim, fontSize: 11.5, fontFamily: fonts.mono, textDecorationLine: 'line-through' }}>{uxFmt(k, b)}</Text>}
        <Text style={{ color: v == null ? c.dim : uxOver(ux, s, k) ? c.amberTx : c.text, fontSize: 13, fontFamily: fonts.mono, fontWeight: uxOver(ux, s, k) ? '700' : '400' }}>{uxFmt(k, v)}</Text>
      </View>
    );
  };
  const judge = (s: UserExtrinsicSensor | undefined, axes: UxAxis[]) => {
    const [text, color] = !s || s.result === 'running' ? [t('측정 중..'), c.muted] : s.result !== 'ok' ? [uxFail(s), c.redTx]
      : axes.some((k) => uxOver(ux, s, k)) ? [t('보정 필요'), c.amberTx]
      : axes.every((k) => uxShown(ux, s, k) == null) ? ['—', c.dim] : [t('정상'), c.greenTx];
    return <Text style={{ flex: 1.1, color, fontSize: 13, fontWeight: '700', textAlign: 'right' }} numberOfLines={1}>{text}</Text>;
  };
  const head = (first: string, axes: UxAxis[]) => (
    <>
      <View style={styles.uxrow}>
        <Text style={[styles.uxhead, { flex: 1.9, color: c.muted }]}>{first}</Text>
        {axes.map((k) => <Text key={k} style={[styles.uxhead, { flex: 1, color: c.muted, textAlign: 'right' }]}>{k}</Text>)}
        {verdict && <Text style={[styles.uxhead, { flex: 1.1, color: c.muted, textAlign: 'right' }]}>{t('판정')}</Text>}
      </View>
      {!!ux.limits && (
        <View style={[styles.uxrow, { paddingVertical: 0 }]}>
          <Text style={{ flex: 1.9, color: c.muted, fontSize: 11.5 }}>{t('기준')}</Text>
          {axes.map((k) => <Text key={k} style={{ flex: 1, color: c.muted, fontSize: 11.5, textAlign: 'right', fontFamily: fonts.mono }}>±{uxLimit(ux, k)}{UX_UNIT[k]}</Text>)}
          {verdict && <View style={{ flex: 1.1 }} />}
        </View>
      )}
    </>
  );
  const title = (text: string, note: string) => (
    <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, flexWrap: 'wrap' }}>
      <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{text}</Text>
      <Text style={{ color: c.muted, fontSize: 12 }}>{note}</Text>
    </View>
  );
  const name = (id: number, vs: string) => (
    <View style={{ flex: 1.9, flexDirection: 'row', alignItems: 'baseline', gap: 5 }}>
      <Text style={{ color: c.text, fontSize: 13, fontWeight: '700', fontFamily: fonts.mono }} numberOfLines={1}>{cameraName(id)}</Text>
      <Text style={{ color: c.muted, fontSize: 12, fontFamily: fonts.mono }} numberOfLines={1}>↔ {vs}</Text>
    </View>
  );
  const A1: UxAxis[] = ['roll', 'pitch', 'z'], A2: UxAxis[] = ['x', 'y', 'yaw'];
  const ref = get(ux.sensors, 2);
  return (
    <View style={{ gap: 12 }}>
      <View style={{ gap: 2 }}>
        {title(t('① 기준 면 대비 — roll · pitch · z'), t('기준 센서 Bottom2 가 본 면과 비교'))}
        {head(t('카메라'), A1)}
        {UX_ROWS.map((id) => {
          const s = get(ux.sensors, id);
          return (
            <View key={id} style={[styles.uxrow, { borderTopWidth: StyleSheet.hairlineWidth, borderTopColor: c.line }]}>
              {name(id, cameraName(2))}
              {A1.map((k) => cell(s, k, 1))}
              {verdict && judge(s, A1)}
            </View>
          );
        })}
        {!!ref && ref.result !== 'ok' && ref.result !== 'running' && (
          <Text style={{ color: c.redTx, fontSize: 12.5, paddingTop: 4 }}>
            {t('{s} (기준 센서) — {r}. 기준 면이 없어 roll · pitch · z 를 잴 수 없습니다.').replace('{s}', cameraName(2)).replace('{r}', uxFail(ref))}
          </Text>
        )}
      </View>
      <View style={{ gap: 2 }}>
        {title(t('② 이웃 카메라 대비 — x · y · yaw'), t('같은 보드를 두 카메라가 본 위치 · 방향 차이, 보드가 있을 때만'))}
        {ux.on_board ? (
          <>
            {head(t('카메라 쌍'), A2)}
            {UX_PAIRS.map((id) => {
              const s = get(ux.sensors, id);
              return (
                <View key={id} style={[styles.uxrow, { borderTopWidth: StyleSheet.hairlineWidth, borderTopColor: c.line }]}>
                  {name(id, cameraName(UX_PARENT[id]))}
                  {A2.map((k) => cell(s, k, 1))}
                  {verdict && judge(s, A2)}
                </View>
              );
            })}
          </>
        ) : (
          <Text style={{ color: c.muted, fontSize: 13, paddingVertical: 4 }}>{t('보드 위에서 측정하면 나옵니다. 바닥으로는 x · y · yaw 를 볼 수 없습니다.')}</Text>
        )}
      </View>
    </View>
  );
}

type UxStep = 'pre' | 'measure' | 'calib' | 'result';
const UX_STEPS: { key: UxStep; label: string }[] = [
  { key: 'pre', label: '선행 점검' }, { key: 'measure', label: '측정' }, { key: 'calib', label: '보정' }, { key: 'result', label: '결과' },
];
const UX_COLLECT_S = 12, UX_REMEASURE_S = 4;

export function DepthExtrinsicModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const ip = useRobot((s) => s.ip);
  const ux = useUserExtrinsic((s) => s.ux);
  const error = useUserExtrinsic((s) => s.error);
  const robot = useTelemetry((s) => s.robot);
  const sitting = !robot?.isStanding;
  const running = ux?.state === 'running';
  const [step, setStep] = useState<UxStep>(() => (running && ux?.action === 'board' ? 'calib' : 'pre'));
  const [measured, setMeasured] = useState(false);
  const [reverted, setReverted] = useState(false);
  const pending = useRef<UserExtrinsicAction | null>(running ? (ux?.action ?? 'measure') : null);
  const [placing, setPlacing] = useState(running && ux?.action === 'approach');
  const startedAt = useRef({ phase: '', at: Date.now() });
  const [, tick] = useState(0);
  useEffect(() => {
    if (!running) return;
    const h = setInterval(() => tick((n) => n + 1), 500);
    return () => clearInterval(h);
  }, [running]);
  useEffect(() => {
    if (running || !ux || !pending.current) return;
    const a = pending.current;
    pending.current = null;
    if (a === 'approach') {
      if (ux.approach?.result === 'ok') run('measure');
      return;
    }
    if (a === 'board') { setStep('result'); return; }
    setMeasured(true);
    setReverted(a === 'revert');
    setStep(a === 'revert' || [0, 1, 2, 3].every((id) => uxBoardsOk(ux, id)) ? 'measure' : 'pre');
  }, [running, ux]);
  function run(a: UserExtrinsicAction) {
    pending.current = a;
    if (a === 'board') setStep('calib');
    if (a === 'approach') { setPlacing(true); setMeasured(false); }
    void startUserExtrinsic(ip, a);
  }
  const phaseKey = running ? `${ux?.action}:${ux?.phase}` : '';
  if (startedAt.current.phase !== phaseKey) startedAt.current = { phase: phaseKey, at: Date.now() };
  const elapsed = (Date.now() - startedAt.current.at) / 1000;

  const stepBar = <StepBar labels={UX_STEPS.map((s) => s.label)} cur={UX_STEPS.findIndex((x) => x.key === step)} warn={t('끝날 때까지 로봇을 움직이지 마세요')} warnOn={running} />;
  const btn = (label: string, enabled: boolean, onPress: () => void, ghost = false) => (
    <Tappable key={label} onPress={enabled ? onPress : undefined}
      style={[styles.btn, styles.btnSm, { backgroundColor: ghost ? c.elev : c.accent, borderColor: ghost ? c.line : c.accent, borderRadius: radius.md, opacity: enabled ? 1 : 0.4 }]}>
      <Text style={{ color: ghost ? c.text : '#fff', fontSize: 14, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
  const banner = (tone: 'ok' | 'warn' | 'info' | 'bad', head: string, body = '') => {
    const skin = { ok: ['rgba(63,185,80,0.12)', c.greenTx], warn: ['rgba(210,153,34,0.12)', c.amberTx], info: ['rgba(77,156,245,0.12)', c.accent2], bad: ['rgba(231,51,28,0.10)', c.redTx] }[tone];
    return (
      <View style={{ backgroundColor: skin[0], borderRadius: radius.sm, paddingHorizontal: 12, paddingVertical: 7, flexDirection: 'row', gap: 10, flexWrap: 'wrap' }}>
        <Text style={{ color: skin[1], fontSize: 14, fontWeight: '700' }}>{head}</Text>
        {!!body && <Text style={{ color: c.text, fontSize: 14, flexShrink: 1 }}>{body}</Text>}
      </View>
    );
  };
  const line = (mark: string, color: string, text: string, right?: ReactNode) => (
    <View style={[styles.uxline, { backgroundColor: c.elev, borderRadius: radius.sm }]}>
      <Text style={{ color, fontWeight: '800', width: 14, textAlign: 'center' }}>{mark}</Text>
      <Text style={{ color: c.text, fontSize: 14, flex: 1 }}>{text}</Text>
      {right}
    </View>
  );
  const guard = sitting && line('!', c.amberTx, t('앉아 있음 — 세운 뒤 측정하세요'));
  const again = () => { setMeasured(false); setReverted(false); setStep('pre'); };
  const can = !running && !sitting;

  const ap = ux?.approach;
  const approaching = running && ux?.action === 'approach';
  const placeIdx = approaching ? (ux?.phase === 'walking' ? 1 : 0) : ap?.result === 'ok' ? 2 : -1;
  const placeFail = !approaching && ap && ap.result !== 'ok' && ap.result !== 'running'
    ? (ap.result === 'no_board' ? t('Bottom2 가 판 가운데 보드를 찾지 못했습니다. 판을 몸통 아래 가운데로 옮긴 뒤 다시 하세요.')
      : ap.result === 'stopped' ? t('정지했습니다. 로봇이 섰습니다.')
      : ap.result === 'reversed' ? t('캘리브레이션 보드가 뒤집혀 있습니다. 보드 위쪽이 로봇 앞쪽을 향하게 다시 놓아 주세요.')
      : t('로봇이 움직이지 않았습니다. 서 있는지 확인하고 다시 하세요.')) : '';
  const placeBox = (
    <View style={{ backgroundColor: c.elev, borderRadius: radius.sm, padding: 12, gap: 8 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, flexWrap: 'wrap' }}>
        <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>{t('보정 위치 잡기')}</Text>
        {[t('보드 찾는 중'), t('걷는 중'), t('완료')].map((x, i) => (
          <View key={x} style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
            {i > 0 && <Text style={{ color: c.muted }}>→</Text>}
            <View style={[styles.uxchip, { borderColor: i < placeIdx || placeIdx === 2 ? c.green : i === placeIdx ? c.accent2 : c.line }]}>
              <Text style={{ color: i < placeIdx || placeIdx === 2 ? c.greenTx : i === placeIdx ? c.accent2 : c.muted, fontSize: 13, fontWeight: '700' }}>{x}</Text>
            </View>
          </View>
        ))}
        {placeIdx === 2 && running && <Text style={{ color: c.accent2, fontSize: 13, fontWeight: '700' }}>{t('측정 중..')}</Text>}
      </View>
      {!!placeFail && banner('bad', t('보정 위치를 잡지 못했습니다'), placeFail)}
    </View>
  );

  let body: ReactNode = null, actions: ReactNode = null;
  if (step === 'pre') {
    const lacking = measured && ux && !running ? [0, 1, 2, 3].filter((id) => uxBoardsOk(ux, id) === false || ux.sensors.find((s) => s.id === id)?.result !== 'ok') : [];
    body = (
      <>
        <View style={{ flexDirection: 'row', gap: 16, alignItems: 'center', flexWrap: 'wrap' }}>
          <Image source={UX_EXAMPLE} style={{ width: 340, height: 216 }} resizeMode="contain" />
          <View style={{ flex: 1, minWidth: 260, gap: 8 }}>
            {[t('평평한 바닥에 로봇을 세웁니다.'), t('보드 위쪽이 로봇 앞쪽을 향하도록 캘리브레이션 보드 판을 몸통 아래, 네 다리 사이에 넣습니다.'), t('[측정 시작]을 누르면 로봇이 판 가운데로 보정 위치를 잡은 뒤 측정합니다.')].map((x, i) => (
              <View key={i} style={{ flexDirection: 'row', gap: 8 }}>
                <View style={[styles.uxdot, { borderColor: c.accent }]}><Text style={{ color: c.accent, fontSize: 12, fontWeight: '800' }}>{i + 1}</Text></View>
                <Text style={{ color: c.text, fontSize: 14, lineHeight: 22, flex: 1 }}>{x}</Text>
              </View>
            ))}
          </View>
        </View>
        {robot?.isStanding ? line('✓', c.greenTx, t('서 있음')) : guard}
        {line('!', c.amberTx, t('보정 위치를 잡는 동안 로봇이 움직입니다. 로봇 주변을 비우고 조심하세요.'))}
        {placing && placeBox}
        {lacking.length > 0 && (
          <>
            <UxCamTiles ux={ux} mode="boards" />
            {banner('warn', t('{s} 에 보드가 충분히 안 보입니다').replace('{s}', lacking.map(cameraName).join(' · ')), t('로봇을 판 가운데로 옮긴 뒤 다시 측정하세요.'))}
          </>
        )}
      </>
    );
    actions = approaching ? btn(t('정지'), true, () => { void startUserExtrinsic(ip, 'stop'); }, true)
      : btn(running ? t('측정 중..') : t('측정 시작'), can, () => run('approach'));
  } else if (step === 'measure' && ux) {
    const need = ux.sensors.some((s) => s.needs_align || s.result !== 'ok');
    body = (
      <>
        {running ? null
          : reverted ? banner('info', t('보정 전으로 되돌렸습니다'), t('되돌린 뒤 다시 잰 값입니다.'))
          : need ? banner('warn', t('위치 보정이 필요합니다'), t('결과는 점검 탭 카드에도 남습니다.'))
          : banner('ok', t('4대 모두 정상입니다'), t('보정할 필요가 없습니다. 결과는 점검 탭 카드에도 남습니다.'))}
        <UxCamTiles ux={ux} mode="verdict" />
        <UxTables ux={ux} />
        {guard}
      </>
    );
    const remeasure = btn(running ? t('측정 중..') : t('다시 측정'), can, () => run('measure'), need);
    const calib = btn(t('보정 시작'), can, () => run('board'), !need);
    actions = need ? <>{remeasure}{calib}</> : <>{calib}{remeasure}</>;
  } else if (step === 'calib') {
    const ph = ux?.phase;
    const pct = ph === 'reverting' ? 95 : ph === 'remeasuring' ? 75 + Math.min(1, elapsed / UX_REMEASURE_S) * 20 : Math.min(1, elapsed / UX_COLLECT_S) * 70;
    const head = ph === 'reverting' ? t('되돌리는 중') : ph === 'remeasuring' ? t('다시 재는 중') : t('보드를 읽는 중');
    const computed = ph === 'remeasuring' || ph === 'reverting';
    body = (
      <>
        <View style={{ backgroundColor: c.elev, borderRadius: radius.sm, padding: 14, gap: 10 }}>
          <View style={{ flexDirection: 'row', justifyContent: 'space-between' }}>
            <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>{head}</Text>
            <Text style={{ color: c.muted, fontSize: 13 }}>{t('{s}초').replace('{s}', String(Math.round(elapsed)))}</Text>
          </View>
          <View style={{ height: 6, borderRadius: 3, backgroundColor: c.elev2, overflow: 'hidden' }}>
            <View style={{ width: `${pct}%`, height: '100%', backgroundColor: c.accent2 }} />
          </View>
          <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, flexWrap: 'wrap' }}>
            {[2, 1, 3, 0].map((id, i) => (
              <View key={id} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
                {i > 0 && <Text style={{ color: c.muted }}>→</Text>}
                <View style={[styles.uxchip, { borderColor: computed ? c.green : c.line }]}>
                  <Text style={{ color: computed ? c.greenTx : c.muted, fontSize: 13, fontWeight: '700' }}>{computed ? '✓ ' : ''}{cameraName(id)}</Text>
                </View>
              </View>
            ))}
          </View>
        </View>
        <Text style={{ color: c.muted, fontSize: 14 }}>{t('기준 센서 Bottom2 에 이웃 카메라를 차례로 맞춥니다. 창을 닫아도 로봇은 계속합니다.')}</Text>
      </>
    );
  } else if (step === 'result' && ux) {
    const o = ux.outcome;
    const msg = o === 'kept' ? banner('ok', t('보정했습니다'), t('4대 모두 정상입니다'))
      : o === 'reverted' ? banner('warn', t('보정하지 않고 되돌렸습니다'), t('보정 뒤에도 정상이 아니었습니다. 판이 밀리지 않았는지 확인하고 다시 보정하세요.'))
      : o === 'skipped' ? banner('info', t('이미 정상입니다'), t('바꿀 것이 없었습니다.'))
      : banner('bad', t('보정하지 못했습니다'), t('보드가 충분히 보이지 않는 카메라가 있습니다. 로봇 위치를 확인하고 다시 보정하세요.'));
    body = (
      <>
        {msg}
        <UxCamTiles ux={ux} mode="verdict" />
        <UxTables ux={ux} before={o === 'kept' ? ux.before : undefined} />
      </>
    );
    actions = o === 'kept' ? <>{btn(running ? t('되돌리는 중') : t('보정 전으로 되돌리기'), !running, () => run('revert'), true)}{btn(t('완료'), !running, onClose)}</>
      : o === 'skipped' ? btn(t('완료'), true, onClose)
      : btn(t('다시 보정'), true, again);
  }
  return (
    <Frame title={t('하단 depth 카메라 위치 보정')} width={880} onClose={onClose} head={stepBar} actions={actions ?? <View />}>
      {body}
      {!!error && <Text style={{ color: c.amberTx, fontSize: 13 }}>{error}</Text>}
    </Frame>
  );
}

const UX_EXAMPLE = require('@/assets/images/ux-board-example.png');

const OV = { dim: 'rgba(255,255,255,0.6)', warn: '#FFB020', bad: '#FF7B72', on: '#3FB950' } as const;

const styles = StyleSheet.create({
  box: { maxWidth: '94%', borderWidth: 1, padding: 18, gap: 12 },
  video: { width: '100%', aspectRatio: 16 / 9, backgroundColor: '#000', borderWidth: 1, overflow: 'hidden' },
  tile: { position: 'absolute', borderWidth: StyleSheet.hairlineWidth, borderColor: 'rgba(255,255,255,0.25)', justifyContent: 'space-between' },
  tileHead: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 6, paddingVertical: 4, backgroundColor: 'rgba(0,0,0,0.55)' },
  namePill: { backgroundColor: 'rgba(255,255,255,0.92)', borderRadius: 4, paddingHorizontal: 6, paddingVertical: 1 },
  tileName: { color: '#0F1413', fontSize: 11.5, fontWeight: '700', letterSpacing: 0.3 },
  tileOnly: { color: OV.dim, fontSize: 11.5, fontWeight: '600' },
  seg: { flexDirection: 'row', marginLeft: 'auto', borderWidth: 1, borderColor: 'rgba(255,255,255,0.45)', borderRadius: 5, overflow: 'hidden', backgroundColor: 'rgba(0,0,0,0.35)' },
  segItem: { height: 22, minWidth: 44, paddingHorizontal: 9, alignItems: 'center', justifyContent: 'center' },
  segDivider: { borderLeftWidth: 1, borderLeftColor: 'rgba(255,255,255,0.3)' },
  toggleTx: { color: 'rgba(255,255,255,0.8)', fontSize: 11, fontWeight: '600' },
  toggleOn: { color: '#fff', fontWeight: '700' },
  tileFoot: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 6, paddingVertical: 3, backgroundColor: 'rgba(0,0,0,0.55)' },
  fpsItem: { flexDirection: 'row', alignItems: 'baseline', gap: 3 },
  fpsKind: { color: OV.dim, fontSize: 10, fontWeight: '600' },
  fpsVal: { color: '#fff', fontSize: 12.5, fontWeight: '700' },
  fpsUnit: { color: OV.dim, fontSize: 10 },
  chip: { borderWidth: 1, borderColor: 'rgba(255,255,255,0.35)', borderRadius: 4, paddingHorizontal: 5 },
  chipTx: { color: '#D5DEDB', fontSize: 10, fontWeight: '600' },
  btn: { height: 44, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  drow: { flexDirection: 'row', alignItems: 'center', paddingVertical: 5, gap: 6 },
  rowBtn: { width: 76, height: 32, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  rowBtnSlot: { width: 76 },
  banner: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1, backgroundColor: 'rgba(0,0,0,0.55)' },
  dcell: { fontSize: 12 },
  pick: { flex: 1, height: 48, borderWidth: 1, alignItems: 'center', justifyContent: 'center', gap: 2 },
  uxtile: { flex: 1, borderWidth: 1, paddingHorizontal: 12, paddingVertical: 8, gap: 2 },
  uxrow: { flexDirection: 'row', alignItems: 'center', paddingVertical: 5, gap: 6 },
  uxhead: { fontSize: 12, fontWeight: '700' },
  uxsteps: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingBottom: 12, borderBottomWidth: StyleSheet.hairlineWidth, flexWrap: 'wrap' },
  uxdotLg: { width: 28, height: 28, borderRadius: 14 },
  uxdot: { width: 24, height: 24, borderRadius: 12, borderWidth: 1.5, alignItems: 'center', justifyContent: 'center' },
  uxline: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 12, paddingVertical: 8 },
  uxbtns: { flexDirection: 'row', alignItems: 'center', gap: 8 },
  btnSm: { minWidth: 120, height: 40, paddingHorizontal: 18 },
  uxchip: { borderWidth: 1, borderRadius: 999, paddingHorizontal: 10, paddingVertical: 3 },
});
