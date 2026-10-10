import { useEffect, useRef, useState } from 'react';
import { Animated, View, Text, StyleSheet, Pressable, type LayoutChangeEvent } from 'react-native';
import Svg, { Line, Rect } from 'react-native-svg';
import { useTheme } from '@/theme';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { useDockView } from '@/store/dockView';
import { connection } from '@/lib/connection';
import { DOCK_BAND_H, DOCK_BAND_TOP, DOCK_PANES, DOCK_PANE_KINDS, DOCK_PANE_LABELS, paneIndexOf, paneU } from '@/lib/dockLayout';
import { discardHint, MARKER_CAM, type MarkerState } from '@/lib/arucoState';
import { t } from '@/lib/i18n';

const HIT = '#FF8C1A';
const AXIS = ['#FF4438', '#4ED07A', '#5AA9FF'];
const STOP = '#FFB020';
const RESUME = '#4ED07A';
const CHARGE_SHOW_MS = 3000;
const STREAM_OFF_MS = 5000;

function statusText(s: number | undefined): { title: string; sub: string; tone: 'idle' | 'run' | 'ok' | 'warn' | 'fail' } {
  if (s == null || s === 0) return { title: t('대기'), sub: t('도킹을 시작하지 않았습니다'), tone: 'idle' };
  if (s === 1) return { title: 'APPROACH 1', sub: t('충전기 옆으로 자리를 잡고 있습니다'), tone: 'run' };
  if (s === 2 || s === 3) return { title: `APPROACH ${s}`, sub: t('충전기 앞으로 접근하고 있습니다'), tone: 'run' };
  if (s === 4) return { title: 'SITTING', sub: t('충전 단자에 앉고 있습니다'), tone: 'run' };
  if (s === 5) return { title: t('도킹 완료'), sub: t('충전을 확인하고 있습니다'), tone: 'ok' };
  if (s === 6) return { title: t('도킹 완료'), sub: t('충전 중'), tone: 'ok' };
  if (s === 7) return { title: t('도킹 완료'), sub: t('결합됐으나 충전이 시작되지 않았습니다 — 충전기 전원을 확인하세요'), tone: 'ok' };
  if (s === -1) return { title: 'RETRY', sub: t('실패했습니다 — 다시 시도합니다'), tone: 'warn' };
  if (s === -2) return { title: 'FAILED', sub: t('마커를 찾지 못했습니다'), tone: 'fail' };
  if (s === -3) return { title: 'FAILED', sub: t('마커가 너무 멉니다 — 충전기 가까이로 옮기세요'), tone: 'fail' };
  if (s === -4) return { title: 'FAILED', sub: t('재시도 횟수를 모두 썼습니다'), tone: 'fail' };
  return { title: 'ABORTED', sub: t('도킹이 취소됐습니다'), tone: 'fail' };
}

export function DockView() {
  const { radius, fonts } = useTheme();
  const hide = useDockView((s) => s.hide);
  const marker = useTelemetry((s) => s.markerState);
  const dockingStatus = useRobot((s) => s.robot?.docking_status);
  const battPct = useRobot((s) => s.battPct);
  const [box, setBox] = useState({ w: 0, h: 0 });
  const onLayout = (e: LayoutChangeEvent) => {
    const { width: w, height: h } = e.nativeEvent.layout;
    setBox((p) => (Math.abs(p.w - w) < 1 && Math.abs(p.h - h) < 1 ? p : { w, h }));
  };

  const st = statusText(dockingStatus);
  const running = dockingStatus != null && dockingStatus >= 1 && dockingStatus <= 4;
  const docked = dockingStatus != null && dockingStatus >= 5 && dockingStatus <= 7;
  const retrying = dockingStatus === -1;
  const exhausted = dockingStatus === -4;
  const dockedAt = useDockView((s) => s.dockedAt);
  useEffect(() => {
    const { dockedAt: at, setDockedAt } = useDockView.getState();
    if (!docked) { if (at != null) setDockedAt(null); }
    else if (at == null) setDockedAt(Date.now());
  }, [docked]);
  const [now, setNow] = useState(() => Date.now());
  useEffect(() => {
    if (dockedAt == null) return;
    setNow(Date.now());
    const ids = [CHARGE_SHOW_MS, STREAM_OFF_MS]
      .map((ms) => dockedAt + ms - Date.now())
      .filter((ms) => ms > 0)
      .map((ms) => setTimeout(() => setNow(Date.now()), ms + 20));
    return () => ids.forEach(clearTimeout);
  }, [dockedAt]);
  const elapsed = docked && dockedAt != null ? now - dockedAt : -1;
  const phase = elapsed >= STREAM_OFF_MS ? 2 : elapsed >= CHARGE_SHOW_MS ? 1 : 0;
  useEffect(() => { useDockView.getState().setPaused(phase === 2); }, [phase]);
  const paused = phase === 2;
  const showCharge = phase >= 1 && dockingStatus === 6;
  const hitPane = marker?.detected && !paused ? paneIndexOf(marker.cam) : -1;
  const hint = marker && !marker.detected && !paused ? discardHint(marker.discard) : null;
  const tone = st.tone === 'fail' ? '#FF8579' : st.tone === 'ok' ? '#6FD48D' : st.tone === 'warn' ? STOP
    : st.tone === 'run' ? '#F1F4F6' : '#98A1AE';
  const bottomCam = useTelemetry((s) => s.sensors?.[MARKER_CAM.bottom]);
  const projOn = bottomCam?.projector ? bottomCam.projectorOn : null;

  const bandTop = box.h * DOCK_BAND_TOP;
  const bandH = box.h * DOCK_BAND_H;
  const paneW = box.w / DOCK_PANES.length;
  const ready = box.w > 0 && box.h > 0;
  const s = Math.max(0.5, Math.min(1, bandH / 239));
  const px = (n: number) => Math.round(n * s);

  return (
    <View style={StyleSheet.absoluteFill} pointerEvents="box-none" onLayout={onLayout}>
      {paused && <View style={[StyleSheet.absoluteFill, styles.blank]} pointerEvents="none" />}

      {ready && paused && (
        <View style={[styles.bandBox, styles.center, { top: bandTop, height: bandH }]} pointerEvents="none">
          <Text style={{ color: '#5F6773', fontSize: px(17) }}>{t('도킹이 끝나 카메라 화면을 껐습니다')}</Text>
        </View>
      )}

      {ready && !paused && (
        <>
          <View style={[styles.labelRow, { top: bandTop - px(30) }]} pointerEvents="none">
            {DOCK_PANE_LABELS.map((label, i) => (
              <View key={label} style={[styles.paneLabelCell, { width: paneW, paddingHorizontal: px(12), gap: px(7) }]}>
                <View style={[styles.paneDot, {
                  width: px(8), height: px(8), borderRadius: px(4),
                  backgroundColor: i === hitPane ? HIT : '#98A1AE', opacity: i === hitPane ? 1 : 0.35,
                }]} />
                <Text style={[styles.paneLabel, {
                  fontSize: px(17), letterSpacing: px(1), color: i === hitPane ? HIT : '#98A1AE',
                }]}>
                  {label.toUpperCase()}
                </Text>
                <Text style={{ color: '#5F6773', fontFamily: fonts.mono, fontSize: px(10) }}>{DOCK_PANE_KINDS[i]}</Text>
                <View style={styles.flex1} />
                {i === hitPane && marker && marker.ids.length > 0 && (
                  <Text numberOfLines={1} style={{ color: HIT, fontFamily: fonts.mono, fontSize: px(11) }}>
                    ID {[...marker.ids].sort((a, b) => a - b).join('·')}
                  </Text>
                )}
                {DOCK_PANES[i] === MARKER_CAM.bottom && <ProjectorChip on={projOn} px={px} mono={fonts.mono} />}
              </View>
            ))}
          </View>

          <View style={[styles.bandBox, { top: bandTop, height: bandH }]} pointerEvents="none">
            {DOCK_PANES.map((cam, i) => (
              <View
                key={cam}
                style={[styles.pane, {
                  left: i * paneW, width: paneW,
                  borderColor: i === hitPane ? HIT : 'rgba(255,255,255,0.18)',
                  borderWidth: i === hitPane ? 4 : 1,
                }]}
              />
            ))}
            {hitPane >= 0 && marker && (
              <MarkerDraw paneIdx={hitPane} m={marker} w={box.w} h={bandH} paneW={paneW} />
            )}
          </View>
        </>
      )}

      <View style={[styles.bottom, { top: bandTop + bandH, paddingHorizontal: px(24), gap: px(28) }]}
        pointerEvents="box-none">
        <View style={styles.status} pointerEvents="none">
          <StepTimeline status={dockingStatus} px={px} />
          {showCharge ? (
            <>
              <View style={[styles.chargeRow, { marginTop: px(12), gap: px(16) }]}>
                <Text style={{ color: tone, fontSize: px(38), fontWeight: '700' }}>{t('충전 중')}</Text>
                <Text style={{ color: '#F1F4F6', fontFamily: fonts.mono, fontSize: px(38), fontWeight: '700' }}>
                  {battPct != null ? `${Math.round(battPct)}%` : '—'}
                </Text>
              </View>
              {battPct != null && (
                <View style={[styles.battTrack, { width: px(210), height: px(7), borderRadius: px(4), marginTop: px(8) }]}>
                  <View style={[styles.battFill, { width: `${Math.max(0, Math.min(100, battPct))}%` }]} />
                </View>
              )}
            </>
          ) : (
            <>
              <Text style={{ color: tone, fontSize: px(38), fontWeight: '700', marginTop: px(12) }}>{st.title}</Text>
              <Text style={{ color: '#98A1AE', fontSize: px(17), marginTop: px(5) }}>{st.sub}</Text>
            </>
          )}
          {hint != null && (
            <Text style={[styles.hint, {
              fontSize: px(15), marginTop: px(8),
              paddingHorizontal: px(12), paddingVertical: px(5), borderRadius: radius.sm,
            }]}>
              {t(hint.title)}
              {dockingStatus === -2 && <Text style={styles.hintAction}>{'\n'}{t(hint.action)}</Text>}
            </Text>
          )}
        </View>
        <View style={[styles.btnCol, { width: px(400), gap: px(10) }]}>
          {running ? (
            <Pressable onPress={() => connection.sendMotion('stand')}
              style={[styles.btn, { height: px(124), backgroundColor: STOP, borderRadius: radius.md }]}>
              <Text style={{ color: '#20180A', fontSize: px(27), fontWeight: '700' }}>{t('도킹 중단')}</Text>
              <Text style={{ color: 'rgba(32,24,10,0.62)', fontSize: px(14) }}>{t('로봇이 그 자리에 섭니다')}</Text>
            </Pressable>
          ) : docked ? (
            <ReleaseButton label={t('도킹 해제')} onReleased={hide} closeAfterMs={paused ? 0 : CLOSE_AFTER_RELEASE_MS} px={px} />
          ) : retrying ? (
            <View style={[styles.btn, styles.btnGhost, { height: px(124), borderRadius: radius.md }]} pointerEvents="none">
              <Text style={{ color: '#E6E8EB', fontSize: px(27), fontWeight: '700' }}>{t('자동 재시도 중')}</Text>
              <Text style={{ color: '#98A1AE', fontSize: px(14) }}>{t('일어선 뒤 다시 접근합니다')}</Text>
            </View>
          ) : exhausted ? (
            <ReleaseButton label={t('일으켜 세우기')} onReleased={hide} closeAfterMs={CLOSE_AFTER_RELEASE_MS} px={px} />
          ) : (
            <Pressable onPress={() => connection.sendMotion('dock')}
              style={[styles.btn, { height: px(124), backgroundColor: RESUME, borderRadius: radius.md }]}>
              <Text style={{ color: '#062012', fontSize: px(27), fontWeight: '700' }}>{t('도킹 재개')}</Text>
              <Text style={{ color: 'rgba(6,32,18,0.62)', fontSize: px(14) }}>{t('지금 자리에서 다시 시도합니다')}</Text>
            </Pressable>
          )}
          <Pressable onPress={hide}
            style={[styles.btn, styles.btnGhost, { height: px(56), borderRadius: radius.md }]}>
            <Text style={{ color: '#E6E8EB', fontSize: px(19), fontWeight: '700' }}>{t('닫기')}</Text>
          </Pressable>
        </View>
      </View>
    </View>
  );
}

type StepKind = 'none' | 'past' | 'cur' | 'done' | 'retry' | 'fail';
function stepKind(s: number | undefined, i: number): StepKind {
  if (s == null) return 'none';
  if (s === 6) return i === 4 ? 'done' : 'past';
  if (s === 5 || s === 7) return i < 3 ? 'past' : i === 3 ? 'cur' : 'none';
  if (s >= 1 && s <= 4) return i + 1 < s ? 'past' : i + 1 === s ? 'cur' : 'none';
  if (s === -1) return i < 3 ? 'past' : i === 3 ? 'retry' : 'none';
  if (s === -4) return i < 3 ? 'past' : i === 3 ? 'fail' : 'none';
  return 'none';
}
const OK = '#6FD48D';
const FAIL = '#FF8579';
function StepTimeline({ status, px }: { status: number | undefined; px: (n: number) => number }) {
  const steps = [t('접근 1'), t('접근 2'), t('접근 3'), t('착석'), t('충전')];
  return (
    <View style={[styles.stepRow, { gap: px(8) }]}>
      {steps.map((name, i) => {
        const k = stepKind(status, i);
        const bar = k === 'cur' ? STOP : k === 'past' ? 'rgba(255,176,32,0.42)' : k === 'done' ? OK
          : k === 'fail' ? FAIL : 'rgba(255,255,255,0.12)';
        const ink = k === 'cur' || k === 'retry' ? STOP : k === 'done' ? OK : k === 'fail' ? FAIL
          : k === 'past' ? '#98A1AE' : '#5F6773';
        return (
          <View key={name} style={styles.step}>
            <View style={[styles.stepBar, { height: px(6), borderRadius: px(3), backgroundColor: bar }]}>
              {k === 'retry' && Array.from({ length: 10 }, (_, j) => (
                <View key={j} style={[styles.flex1, { backgroundColor: j % 2 ? 'rgba(255,176,32,0.25)' : STOP }]} />
              ))}
            </View>
            <Text numberOfLines={1} style={{ color: ink, fontSize: px(14), fontWeight: k === 'cur' ? '700' : '400', marginTop: px(5) }}>
              {name}
            </Text>
          </View>
        );
      })}
    </View>
  );
}

const PROJ_ON = '#38C8E8';
function ProjectorChip({ on, px, mono }: { on: boolean | null; px: (n: number) => number; mono: string }) {
  const ink = on ? '#04161B' : on === false ? '#98A1AE' : '#5F6773';
  return (
    <View style={[styles.projChip, {
      gap: px(5), paddingHorizontal: px(8), paddingVertical: px(3), borderRadius: px(8),
      backgroundColor: on ? PROJ_ON : 'rgba(0,0,0,0.45)',
      borderColor: on ? PROJ_ON : 'rgba(255,255,255,0.22)',
    }]}>
      <View style={{ width: px(7), height: px(7), borderRadius: px(4), borderWidth: 1.5, borderColor: ink,
        backgroundColor: on ? ink : 'transparent' }} />
      <Text style={{ color: ink, fontFamily: mono, fontSize: px(11), fontWeight: on ? '700' : '400' }}>
        PROJECTOR {on ? 'ON' : on === false ? 'OFF' : '—'}
      </Text>
    </View>
  );
}

const HOLD_MS = 800;
const CLOSE_AFTER_RELEASE_MS = 2000;
function ReleaseButton({ label, onReleased, closeAfterMs, px }: {
  label: string; onReleased: () => void; closeAfterMs: number; px: (n: number) => number;
}) {
  const { radius } = useTheme();
  const [holding, setHolding] = useState(false);
  const timer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const closeTimer = useRef<ReturnType<typeof setTimeout> | null>(null);
  const fill = useRef(new Animated.Value(0)).current;
  useEffect(() => () => {
    if (timer.current) clearTimeout(timer.current);
    if (closeTimer.current) clearTimeout(closeTimer.current);
  }, []);
  const start = () => {
    setHolding(true);
    Animated.timing(fill, { toValue: 1, duration: HOLD_MS, useNativeDriver: false }).start();
    timer.current = setTimeout(() => {
      setHolding(false);
      timer.current = null;
      connection.sendMotion('stand');
      closeTimer.current = setTimeout(onReleased, closeAfterMs);
    }, HOLD_MS);
  };
  const cancel = () => {
    if (timer.current) { clearTimeout(timer.current); timer.current = null; }
    fill.stopAnimation(() => fill.setValue(0));
    setHolding(false);
  };
  return (
    <Pressable onPressIn={start} onPressOut={cancel}
      style={[styles.btn, { height: px(124), backgroundColor: '#3B8EEA', borderRadius: radius.md }]}>
      <Animated.View pointerEvents="none" style={[styles.holdFill, {
        width: fill.interpolate({ inputRange: [0, 1], outputRange: ['0%', '100%'] }),
      }]} />
      <Text style={{ color: '#04121F', fontSize: px(27), fontWeight: '700' }}>{label}</Text>
      <Text style={{ color: 'rgba(4,18,31,0.62)', fontSize: px(14) }}>
        {holding ? t('계속 누르고 계세요…') : t('길게 눌러 로봇을 일으킵니다')}
      </Text>
    </Pressable>
  );
}

function MarkerDraw({ paneIdx, m, w, h, paneW }: {
  paneIdx: number; m: MarkerState; w: number; h: number; paneW: number;
}) {
  const x = (u: number) => paneU(paneIdx, u) * w;
  const y = (v: number) => v * h;
  const [ox, oy] = m.axes[0];
  return (
    <Svg width={w} height={h} style={StyleSheet.absoluteFill}>
      <Rect x={x(m.uMin)} y={y(m.vMin)}
        width={(m.uMax - m.uMin) * paneW} height={(m.vMax - m.vMin) * h}
        fill="rgba(255,140,26,0.14)" stroke={HIT} strokeWidth={2.5} />
      {m.axes.slice(1).map(([au, av], k) => (
        <Line key={k} x1={x(ox)} y1={y(oy)} x2={x(au)} y2={y(av)}
          stroke={AXIS[k]} strokeWidth={2.5} strokeLinecap="round" />
      ))}
    </Svg>
  );
}

const styles = StyleSheet.create({
  blank: { backgroundColor: '#000' },
  center: { alignItems: 'center', justifyContent: 'center' },
  chargeRow: { flexDirection: 'row', alignItems: 'baseline' },
  labelRow: { position: 'absolute', left: 0, right: 0, flexDirection: 'row' },
  paneLabelCell: { flexDirection: 'row', alignItems: 'center' },
  paneDot: {},
  flex1: { flex: 1 },
  projChip: { flexDirection: 'row', alignItems: 'center', borderWidth: 1 },
  battTrack: { backgroundColor: 'rgba(255,255,255,0.1)', overflow: 'hidden' },
  battFill: { height: '100%', backgroundColor: '#6FD48D' },
  paneLabel: { fontWeight: '700', textShadowColor: 'rgba(0,0,0,0.9)', textShadowRadius: 6 },
  bandBox: { position: 'absolute', left: 0, right: 0 },
  pane: { position: 'absolute', top: 0, bottom: 0 },
  bottom: { position: 'absolute', left: 0, right: 0, bottom: 0, flexDirection: 'row', alignItems: 'center' },
  status: { flex: 1, justifyContent: 'center' },
  stepRow: { flexDirection: 'row' },
  step: { flex: 1, minWidth: 0 },
  stepBar: { flexDirection: 'row', overflow: 'hidden' },
  hint: { alignSelf: 'flex-start', color: STOP, fontWeight: '600',
    borderWidth: 1, borderColor: 'rgba(255,176,32,0.45)', backgroundColor: 'rgba(255,176,32,0.12)' },
  hintAction: { fontWeight: '400', color: '#F1F4F6' },
  btnCol: {},
  btn: { alignItems: 'center', justifyContent: 'center', gap: 4, paddingHorizontal: 16, overflow: 'hidden' },
  btnGhost: { backgroundColor: 'rgba(0,0,0,0.55)', borderWidth: 1, borderColor: 'rgba(255,255,255,0.28)' },
  holdFill: { position: 'absolute', left: 0, top: 0, bottom: 0, backgroundColor: 'rgba(255,255,255,0.24)' },
});
