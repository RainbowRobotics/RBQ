import { useEffect, useState } from 'react';
import { useViewport } from '@/store/viewport';
import { View, Text, StyleSheet, ScrollView, useWindowDimensions } from 'react-native';
import { RecoveryPanel } from '@/components/panels/RecoveryPanel';
import { LinearGradient } from 'expo-linear-gradient';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Popover, Modal } from '@/components/ui/overlays';
import { Slider } from '@/components/ui/controls';
import { useRobot } from '@/store/robot';
import { useTelemetry } from '@/store/telemetry';
import { useAccessLevel, useSettings } from '@/store/settings';
import { robotVersionDef, useRobotKind } from '@/modules/registry';
import { useFeatureWheel } from '@/store/capability';
import { useVisionToggles } from '@/store/visionToggles';
import { connection } from '@/lib/connection';
import { walkPercentToSi, actions } from '@/lib/rest';
import { gait as gaitApi, RL_GAIT } from '@/lib/gait';
import { commissioning } from '@/lib/commissioning';
import { sendUserCommand } from '@/lib/userCommand';
import { PROGRAM, QUADWALK_CMD } from '@/lib/robotState';
import { useAvailableSources } from '@/lib/visionSources';
import { AutostartDetail } from '@/components/panels/AutostartDetail';
import { AutoStartSteps } from '@/components/panels/CommissioningPanel';
import { dockAnchor } from '@/components/control/ToolRail';
import { t } from '@/lib/i18n';
import { INIT_STATE, type MotionName } from '@/types/robot';

function PopHeader({ icon, text }: { icon?: IconName; text: string }) {
  const { c } = useTheme();
  return (
    <View style={styles.popH}>
      {icon && <Icon name={icon} size={13} color={c.accent2} />}
      <Text style={{ color: c.muted, fontSize: 11, fontWeight: '600' }}>{text}</Text>
    </View>
  );
}

type WalkKey = 'body_height' | 'max_speed' | 'foot_height';
function SliderRow({ k, sub, param }: { k: string; sub: string; param: WalkKey }) {
  const { c, fonts } = useTheme();
  const v = useSettings((s) => s.walk[param]);
  const setWalk = useSettings((s) => s.setWalk);
  const commitWalk = useSettings((s) => s.commitWalk);
  const si = walkPercentToSi(useSettings.getState().walk);
  const label = param === 'max_speed' ? `${si.max_speed.toFixed(2)} m/s`
    : param === 'body_height' ? `${si.body_height >= 0 ? '+' : ''}${si.body_height.toFixed(2)} m` : `${v}%`;
  return (
    <View style={{ marginBottom: 12, width: 208 }}>
      <View style={styles.slTop}>
        <Text style={{ fontSize: 11, color: c.text }}>{k}<Text style={{ color: c.dim, fontSize: 9 }}>  {sub}</Text></Text>
        <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 12, fontWeight: '600' }}>{label}</Text>
      </View>
      <Slider value={v} width="100%" onChange={(nv) => setWalk(param, nv)} onCommit={commitWalk} />
    </View>
  );
}

const DYN_DEFAULT = { label: 'Default', speed: 1.0, height: 0, tilt: 0 };

function siToPercent(si: { max_speed: number; body_height: number }) {
  const sp = ((si.max_speed - 0.5) / 2.0) * 100;
  const bp = ((si.body_height + 0.25) / 0.35) * 100;
  const clamp = (v: number) => Math.round(Math.min(100, Math.max(0, v)));
  return { max_speed: clamp(sp), body_height: clamp(bp) };
}

function TiltRow() {
  const { c, fonts } = useTheme();
  const bodyTilt = useSettings((s) => s.bodyTilt);
  const setBodyTilt = useSettings((s) => s.setBodyTilt);
  const commitBodyTilt = useSettings((s) => s.commitBodyTilt);
  return (
    <View style={{ marginBottom: 12, width: 208 }}>
      <View style={styles.slTop}>
        <Text style={{ fontSize: 11, color: c.text }}>Body Tilt<Text style={{ color: c.dim, fontSize: 9 }}>{t('  본체 피치')}</Text></Text>
        <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 12, fontWeight: '600' }}>
          {bodyTilt >= 0 ? '+' : ''}{bodyTilt}°
        </Text>
      </View>
      <Slider value={((bodyTilt + 25) / 50) * 100} width="100%"
        onChange={(v) => setBodyTilt(Math.round((v / 100) * 50 - 25))} onCommit={commitBodyTilt} />
    </View>
  );
}

function ObsMarginRow() {
  const { c, fonts } = useTheme();
  const margin = useSettings((s) => s.obsAvoidMargin);
  const setMargin = useSettings((s) => s.setObsAvoidMargin);
  const commitMargin = useSettings((s) => s.commitObsAvoidMargin);
  const pct = ((margin - 0.2) / 0.3) * 100;
  return (
    <View style={{ marginBottom: 12, width: 208 }}>
      <View style={styles.slTop}>
        <Text style={{ fontSize: 11, color: c.text }}>{t('회피 여유거리')}</Text>
        <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 12, fontWeight: '600' }}>
          {margin.toFixed(2)} m
        </Text>
      </View>
      <Slider value={pct} width={230}
        onChange={(v) => setMargin(0.2 + (v / 100) * 0.3)} onCommit={commitMargin} />
    </View>
  );
}

function ObsAvoidModeRow() {
  const { c } = useTheme();
  const mode = useSettings((s) => s.obsAvoidMode);
  const setMode = useSettings((s) => s.setObsAvoidMode);
  return (
    <View style={{ marginBottom: 12, width: 208 }}>
      <Text style={{ fontSize: 11, color: c.text, marginBottom: 6 }}>{t('회피모드')}</Text>
      <View style={{ flexDirection: 'row', gap: 8 }}>
        {(['avoid', 'stop'] as const).map((m) => {
          const on = mode === m;
          return (
            <Tappable key={m} onPress={() => setMode(m)}
              style={[styles.presetBtn, { flex: 1,
                backgroundColor: on ? 'rgba(77,156,245,0.12)' : c.elev,
                borderColor: on ? 'rgba(77,156,245,0.5)' : c.line }]}>
              <Text style={{ color: on ? c.accent2 : c.text, fontSize: 10, fontWeight: '600' }}>
                {m === 'avoid' ? t('회피') : t('정지')}
              </Text>
            </Tappable>
          );
        })}
      </View>
    </View>
  );
}

function ObsMapButton() {
  const { c, radius } = useTheme();
  const vpKey = useViewport((v) => v.key);
  const setVpKey = useViewport((v) => v.setKey);
  const showObsMap = useSettings((s) => s.showObsMap);
  const setShowObsMap = useSettings((s) => s.setShowObsMap);
  const sourceReady = useAvailableSources().some((v) => v.key === 'obsmap');
  const toggleMap = () => {
    const next = !showObsMap;
    setShowObsMap(next);
    if (next) setVpKey('obsmap');
    else if (vpKey === 'obsmap') setVpKey('pose3d');
  };
  return (
    <Tappable disabled={!sourceReady} onPress={toggleMap}
      style={[styles.obsMapLink, {
        borderColor: showObsMap ? 'rgba(77,156,245,0.5)' : c.line, borderRadius: radius.md,
        backgroundColor: showObsMap ? 'rgba(77,156,245,0.12)' : c.elev,
        opacity: sourceReady ? 1 : 0.45,
      }]}>
      <Icon name="route" size={13} color={c.accent2} />
      <Text style={{ color: showObsMap ? c.accent2 : c.text, fontSize: 11, fontWeight: '700' }}>
        {sourceReady ? (showObsMap ? t('장애물 지도 ✓') : t('장애물 지도')) : t('장애물 지도 소스 없음')}
      </Text>
    </Tappable>
  );
}

export function DrivePopover({ onClose }: { onClose: () => void }) {
  const { c } = useTheme();
  const { height: winH } = useWindowDimensions();
  const pos = { ...dockAnchor(winH), width: 250 };
  const setWalk = useSettings((s) => s.setWalk);
  const commitWalk = useSettings((s) => s.commitWalk);
  const setBodyTilt = useSettings((s) => s.setBodyTilt);
  const commitBodyTilt = useSettings((s) => s.commitBodyTilt);
  const visionWalk = useSettings((s) => s.visionWalkEnabled);
  const isSim = useViewport((v) => v.key) === 'sim';
  const ip = useRobot((s) => s.ip);
  const gaitId = useRobot((s) => s.robot?.gait_id);
  const toggleVisionWalk = () => {
    const next = !visionWalk;
    useSettings.getState().setVisionWalkEnabled(next);
    if (gaitId === RL_GAIT.RL_WALK_VISION || gaitId === RL_GAIT.RL_WALK) {
      gaitApi.aiWalk(ip, next ? RL_GAIT.RL_WALK_VISION : RL_GAIT.RL_WALK).catch(() => {});
    }
  };
  const accessLevel = useSettings((s) => s.accessLevel);
  const obsAvoidUi = !isSim && accessLevel >= 2;
  const obsAvoidOn = useSettings((s) => s.obsAvoidEnabled);
  const obsAvoidMargin = useSettings((s) => s.obsAvoidMargin);
  const exitObsMapIfActive = () => {
    if (useViewport.getState().key === 'obsmap') useViewport.getState().setKey('pose3d');
  };
  const toggleObsAvoid = () => {
    const next = !obsAvoidOn;
    useSettings.getState().setObsAvoidEnabled(next);
    sendUserCommand(PROGRAM.QuadWalk, QUADWALK_CMD.OBS_AVOID, [], [next ? 1 : 0], [obsAvoidMargin]);
    if (next) {
      useSettings.getState().setShowObsMap(true);
      useViewport.getState().setKey('obsmap');
    } else {
      exitObsMapIfActive();
      useSettings.getState().setShowObsMap(false);
    }
  };
  const applyPreset = (p: typeof DYN_DEFAULT) => {
    const pct = siToPercent({ max_speed: p.speed, body_height: p.height });
    setWalk('max_speed', pct.max_speed);
    setWalk('body_height', pct.body_height);
    commitWalk();
    setBodyTilt(p.tilt);
    commitBodyTilt();
  };
  return (
    <Popover onClose={onClose} style={pos}>
      <PopHeader icon="sliders" text={t('주행 파라미터')} />
      {!isSim && <View style={styles.presetRow}>
        <Tappable onPress={toggleVisionWalk}
          style={[styles.presetBtn, {
            backgroundColor: visionWalk ? 'rgba(77,156,245,0.12)' : c.elev,
            borderColor: visionWalk ? 'rgba(77,156,245,0.5)' : c.line,
          }]}>
          <Text style={{ color: visionWalk ? c.accent2 : c.text, fontSize: 10, fontWeight: '600' }}>
            {visionWalk ? 'Vision Walk ✓' : 'Vision Walk'}
          </Text>
        </Tappable>
      </View>}
      <SliderRow k="Max Speed" sub={t('최대 속도')} param="max_speed" />
      {!isSim && <SliderRow k="Body Height" sub={t('본체 높이')} param="body_height" />}
      {!isSim && <TiltRow />}
      {obsAvoidUi && (
        <View style={{ marginBottom: 10 }}>
          <Tappable onPress={toggleObsAvoid}
            style={[styles.presetBtn, { alignSelf: 'flex-start',
              backgroundColor: obsAvoidOn ? 'rgba(77,156,245,0.12)' : c.elev,
              borderColor: obsAvoidOn ? 'rgba(77,156,245,0.5)' : c.line,
            }]}>
            <Text style={{ color: obsAvoidOn ? c.accent2 : c.text, fontSize: 10, fontWeight: '600' }}>
              {obsAvoidOn ? t('장애물 회피 ✓') : t('장애물 회피')}
            </Text>
          </Tappable>
        </View>
      )}
      {obsAvoidUi && obsAvoidOn && <ObsMarginRow />}
      {obsAvoidUi && obsAvoidOn && <ObsAvoidModeRow />}
      {obsAvoidUi && obsAvoidOn && <ObsMapButton />}
      {!isSim && <Tappable onPress={() => applyPreset(DYN_DEFAULT)}
        style={[styles.defaultBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
        <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{t('기본값 복귀')}</Text>
      </Tappable>}
    </Popover>
  );
}

function SigRow({ nm, ok, onFix, okLabel, fixLabel, soft }: {
  nm: string; ok: boolean; onFix?: () => void; okLabel?: string; fixLabel?: string;
  soft?: boolean;
}) {
  const { c } = useTheme();
  const badC = soft ? c.amber : c.red;
  const badTx = soft ? c.amberTx : c.redTx;
  const body = (
    <>
      <View style={[styles.sigDot, { backgroundColor: ok ? c.green : badC }]} />
      <Text style={{ color: c.text, fontSize: 13, fontWeight: '600' }}>{nm}</Text>
      <Text style={{ marginLeft: 'auto', fontSize: 12, fontWeight: '700', color: ok ? c.greenTx : badTx }}>
        {ok ? okLabel ?? t('정상') : fixLabel ?? (onFix ? t('실행') : t('점검'))}
      </Text>
      <Text style={{ width: 11, textAlign: 'right', fontSize: 12, fontWeight: '700', color: ok ? c.greenTx : badTx }}>
        {!ok && onFix ? '▸' : ''}
      </Text>
    </>
  );
  if (!ok && onFix) {
    return (
      <Tappable onPress={onFix} style={[styles.sigRow, { borderTopColor: c.line2 }]}>{body}</Tappable>
    );
  }
  return <View style={[styles.sigRow, { borderTopColor: c.line2 }]}>{body}</View>;
}

export function SignalPopover({ onClose, onAutoStart }: { onClose: () => void; onAutoStart?: () => void }) {
  const { c, radius } = useTheme();
  const { height: winH } = useWindowDimensions();
  const r = useRobot((s) => s.robot);
  const ip = useRobot((s) => s.ip);
  const visionConn = useTelemetry((s) => s.visionConn);
  const isMine = useRobot((s) => s.isMine);
  return (
    <Popover onClose={onClose} style={{ left: 44, top: 60, width: 280, maxHeight: winH - 72 }}>
      <PopHeader text={t('시스템 종합 상태')} />
      <SigRow nm="IMU" ok={!!r?.imu} />
      <SigRow nm="CAN" ok={!!r?.can_bus} onFix={() => { commissioning.canCheck(ip).catch(() => {}); }} />
      <SigRow nm="FindPose" ok={!!r?.find_pose} onFix={() => { commissioning.findHome(ip).catch(() => {}); }} />
      <SigRow nm="Control" ok={!!r?.control_started} />
      <SigRow nm={t('비전')} ok={visionConn === 'connected'} soft={visionConn === 'connecting'}
        okLabel={t('연결됨')} fixLabel={visionConn === 'connecting' ? t('연결 중') : t('끊김')} />
      <SigRow nm={t('소유권')} ok={isMine} soft okLabel={t('보유')} fixLabel={t('가져오기')}
        onFix={isMine ? undefined : () => connection.claimOwnership()} />
      {onAutoStart && (
        <Tappable onPress={() => { onClose(); onAutoStart(); }}
          style={{ marginTop: 10, height: 34, borderRadius: radius.md, borderWidth: 1, alignItems: 'center', justifyContent: 'center',
            backgroundColor: 'rgba(63,185,80,0.10)', borderColor: 'rgba(63,185,80,0.5)' }}>
          <Text style={{ color: c.greenTx, fontSize: 12, fontWeight: '700' }}>{t('▶ 자동 기동 (전체 시퀀스)')}</Text>
        </Tappable>
      )}
    </Popover>
  );
}

export function EStopModal({ onClose }: { onClose: () => void }) {
  const { c, radius } = useTheme();
  const kind = useRobotKind();
  const confirm = () => { connection.sendMotion('estop'); onClose(); };
  return (
    <Modal onClose={onClose}>
      <LinearGradient colors={[c.modalA, c.modalB]} style={[styles.modal, { borderColor: c.line }]}>
        <View style={[styles.estopIc, { backgroundColor: 'rgba(231,51,28,0.14)', borderColor: 'rgba(231,51,28,0.55)' }]}>
          <Icon name="estop" size={30} color={c.redbright} />
        </View>
        <Text style={{ fontSize: 18, fontWeight: '700', color: c.text, marginBottom: 8 }}>{kind ? t(kind.estop.title) : t('비상 정지하시겠습니까?')}</Text>
        <Text style={{ fontSize: 12.5, color: c.muted, textAlign: 'center', lineHeight: 18, marginBottom: 22 }}>
          {kind
            ? kind.estop.body.map((b) => t(b)).join('\n')
            : <>{t('로봇이 즉시 모든 동작을 멈추고 힘을 차단합니다.')}{'\n'}{t('제어를 다시 시작하려면 재초기화가 필요합니다.')}</>}
        </Text>
        <View style={{ flexDirection: 'row', gap: 11 }}>
          <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{t('취소')}</Text>
          </Tappable>
          <Tappable onPress={confirm} style={[styles.btn, { borderColor: c.dangerLine, overflow: 'hidden' }]}>
            <LinearGradient colors={[c.dangerA, c.dangerB]} style={StyleSheet.absoluteFill} />
            <Text style={{ color: c.onAccent, fontSize: 14, fontWeight: '600' }}>{t('정지')}</Text>
          </Tappable>
        </View>
      </LinearGradient>
    </Modal>
  );
}

export function AutoStartModal({ onClose }: { onClose: () => void }) {
  const { c } = useTheme();
  const [started, setStarted] = useState(false);
  const running = useRobot((s) => !!s.robot?.autostart?.running);
  const [sawRun, setSawRun] = useState(false);
  const ctrlStep = useRobot((s) => s.robot?.autostart?.steps?.[7]);
  const succeeded = sawRun && !running && (ctrlStep === INIT_STATE.pass || ctrlStep === INIT_STATE.warn);
  const [pending, setPending] = useState(false);
  const confirm = () => {
    if (pending || running) return;
    connection.sendMotion('auto_start');
    setStarted(true);
    setPending(true);
  };
  useEffect(() => { if (running) { setPending(false); setSawRun(true); } }, [running]);
  useEffect(() => {
    if (!pending) return;
    const id = setTimeout(() => setPending(false), 8000);
    return () => clearTimeout(id);
  }, [pending]);
  const watching = started || running || sawRun;
  const showGuide = !running && !pending;
  const { width: winW, height: winH } = useWindowDimensions();
  const modalW = Math.min(620, winW - 32);
  const compact = winH < 500;
  return (
    <Modal onClose={onClose} fit>
      <LinearGradient colors={[c.modalA, c.modalB]} style={[styles.modal, styles.fill, { borderColor: c.line, width: modalW }]}>
        <Text style={{ fontSize: 18, fontWeight: '700', color: c.text, marginBottom: 8 }}>{t('자동 기동')}</Text>
        {showGuide && (
          <Text style={{ fontSize: 12.5, color: c.muted, textAlign: 'center', lineHeight: 18, marginBottom: 14 }}>
            {(() => {
              const [a1, a2] = t('로봇을 {b}에 두고').split('{b}');
              const [b1, b2] = t('{b} 확인하세요.').split('{b}');
              const bold = { color: c.text, fontWeight: '700' as const };
              return (
                <>
                  {a1}<Text style={bold}>{t('평평한 바닥')}</Text>{a2}{'\n'}
                  {b1}<Text style={bold}>{t('모든 발과 무릎이 지면에 닿아 있는지')}</Text>{b2}
                </>
              );
            })()}
          </Text>
        )}
        <View style={[styles.fill, { marginBottom: 14, width: '100%' }]}>
          <AutoStartSteps />
          <View style={[styles.fill, { marginTop: 10 }]}><AutostartDetail compact={compact} fill /></View>
        </View>
        <View style={{ flexDirection: 'row', gap: 11 }}>
          <Tappable onPress={onClose} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line }]}>
            <Text style={{ color: c.text, fontSize: 14, fontWeight: '600' }}>{watching ? t('닫기') : t('취소')}</Text>
          </Tappable>
          {!running && !pending && !succeeded && (
            <Tappable onPress={confirm} style={[styles.btn, { backgroundColor: 'rgba(63,185,80,0.14)', borderColor: 'rgba(63,185,80,0.5)' }]}>
              <Text style={{ color: c.greenTx, fontSize: 14, fontWeight: '700' }}>
                {watching ? t('다시 시작') : t('자동 기동')}
              </Text>
            </Tappable>
          )}
        </View>
      </LinearGradient>
    </Modal>
  );
}

function ActTile({ icon, label, onPress }: { icon: IconName; label: string; onPress: () => void }) {
  const { c } = useTheme();
  return (
    <Tappable onPress={onPress} style={[styles.tile, { backgroundColor: c.elev, borderColor: c.line }]}>
      <Icon name={icon} size={22} color={c.accent2} />
      <Text style={{ fontSize: 11, fontWeight: '600', color: c.text }}>{label}</Text>
    </Tappable>
  );
}

export type Tile = { icon: IconName; label: string; motion: MotionName };
const FIXED_TILES: Tile[] = [
  { icon: 'sit', label: 'Sit', motion: 'sit' },
  { icon: 'stand', label: 'Stand', motion: 'stand' },
  { icon: 'walk', label: 'Walk', motion: 'walk' },
  { icon: 'stairs', label: 'Stairs', motion: 'stairs' },
  { icon: 'dock', label: 'Docking', motion: 'dock' },
];
export const MODEL_WALKING: Tile[] = [
  { icon: 'wave', label: 'Wave', motion: 'wave' },
  { icon: 'run', label: 'Run', motion: 'run' },
];
export const AI_MOTIONS: Tile[] = [
  { icon: 'ai', label: 'AI Walk', motion: 'ai_walk' },
  { icon: 'track', label: 'AI Vision', motion: 'ai_vision' },
  { icon: 'track', label: 'AI Vision Slow', motion: 'ai_vision_slow' },
  { icon: 'ai', label: 'AI Pronk', motion: 'ai_pronk' },
  { icon: 'ai', label: 'AI Bound', motion: 'ai_bound' },
  { icon: 'ai', label: 'AI Pace', motion: 'ai_pace' },
  { icon: 'run', label: 'AI Run', motion: 'ai_run' },
  { icon: 'stand', label: 'AI 2Leg L', motion: 'ai_2leg_l' },
  { icon: 'stand', label: 'AI 2Leg R', motion: 'ai_2leg_r' },
  { icon: 'stand', label: 'AI 2Leg F', motion: 'ai_2leg_f' },
  { icon: 'stand', label: 'AI 3Leg HL', motion: 'ai_3leg_hl' },
];

const ALL_TILES: Tile[] = [...FIXED_TILES, ...MODEL_WALKING, ...AI_MOTIONS];

export function useDockButton(): boolean {
  const rv = useSettings((s) => s.robotVersion);
  const dockScan = useVisionToggles((s) => s.dockScan);
  return !robotVersionDef(rv)?.hideDock && !dockScan;
}
export function motionTile(m: MotionName): Tile | undefined {
  return ALL_TILES.find((t) => t.motion === m);
}

export const GAIT_TO_MOTION: Record<number, MotionName> = {
  0: 'sit',
  1: 'stand',
  2: 'stand',
  3: 'walk',
  7: 'walk',
  9: 'walk',
  4: 'stairs',
  5: 'wave',
  6: 'run',
  10: 'dock',
  30: 'ai_walk',
  31: 'ai_2leg_f',
  33: 'ai_2leg_l',
  34: 'ai_2leg_r',
  35: 'ai_bound',
  36: 'ai_pace',
  37: 'ai_pronk',
  39: 'ai_3leg_hl',
  42: 'ai_vision',
  45: 'ai_run',
  47: 'ai_vision_slow',
  48: 'walk',
  49: 'walk',
};

export function useActiveMotion(): MotionName | null {
  return useRobot((s) => (s.robot ? GAIT_TO_MOTION[s.robot.gait_id] ?? null : null));
}

export function MotionGridModal({ onClose, onPick, danger = false, onZmpCalib }: {
  onClose: () => void;
  onPick?: (m: MotionName) => void;
  danger?: boolean;
  onZmpCalib?: () => void;
}) {
  const { c } = useTheme();
  const active = useActiveMotion();
  const aiVisible = AI_MOTIONS;
  const accessLevel = useSettings((s) => s.accessLevel);
  const ip = useRobot((s) => s.ip);
  const featureWheel = useFeatureWheel();
  const [recOpen, setRecOpen] = useState(false);
  const { width: winW, height: winH } = useWindowDimensions();
  const gridClamp = { maxHeight: Math.min(430, winH - 24) };
  const pick = (m: MotionName) => {
    if (onPick) onPick(m);
    else connection.sendMotion(m);
    onClose();
  };
  const tiles = (list: Tile[], ai: boolean) => list.map((t) => {
    const on = t.motion === active;
    return (
      <Tappable key={t.motion} onPress={() => pick(t.motion)}
        style={[styles.tile, { backgroundColor: on ? 'rgba(77,156,245,0.12)' : c.elev, borderColor: on ? c.accent : c.line }]}>
        <Icon name={t.icon} size={22} color={on ? c.accent2 : ai ? c.cyan : c.accent2} />
        <Text style={{ fontSize: 11, fontWeight: '600', color: on ? c.accent2 : ai ? c.cyanTx : c.text }}>{t.label}</Text>
      </Tappable>
    );
  });
  if (recOpen) {
    return (
      <Modal onClose={onClose}>
        <LinearGradient colors={[c.modalA, c.modalB]}
          style={[styles.recWide, { borderColor: c.line, width: Math.min(760, winW - 24), maxHeight: winH - 24 }]}>
          <ScrollView showsVerticalScrollIndicator contentContainerStyle={{ padding: 16 }}>
            <RecoveryPanel />
          </ScrollView>
          <Tappable onPress={() => setRecOpen(false)} hitSlop={6}
            style={[styles.recClose, { backgroundColor: c.elev, borderColor: c.line }]}>
            <Icon name="x" size={13} color={c.muted} />
          </Tappable>
        </LinearGradient>
      </Modal>
    );
  }
  return (
    <Modal onClose={onClose}>
      <LinearGradient colors={[c.modalA, c.modalB]} style={[styles.grid, gridClamp, { borderColor: c.line }]}>
        <View style={[styles.gridHead, { borderBottomColor: c.line2 }]}>
          <Text style={{ fontSize: 14, fontWeight: '700', color: c.text }}>
            {t('모션 선택')} <Text style={{ fontSize: 10, color: c.dim, fontWeight: '500' }}>{t('· 탭하면 즉시 실행 후 닫힘')}</Text>
          </Text>
          <Tappable onPress={onClose} style={[styles.gridClose, { backgroundColor: c.elev, borderColor: c.line }]}>
            <Icon name="x" size={16} color={c.muted} />
          </Tappable>
        </View>
        <ScrollView contentContainerStyle={styles.tilesWrap}>
          <Text style={[styles.groupLabel, { color: c.muted }]}>{t('모델 보행')}</Text>
          <View style={styles.tiles}>
            {tiles(MODEL_WALKING, false)}
            {danger && (
              <ActTile icon="walk" label="Model Walk"
                onPress={() => { onClose(); connection.sendMotion('walk', true); }} />
            )}
          </View>
          {aiVisible.length > 0 && (
            <>
              <Text style={[styles.groupLabel, { color: c.cyanTx, marginTop: 14 }]}>{t('AI 모션')}</Text>
              <View style={styles.tiles}>{tiles(aiVisible, true)}</View>
            </>
          )}
          {danger && (
            <>
              <Text style={[styles.groupLabel, { color: c.dim, marginTop: 14 }]}>{t('정밀·보정')}</Text>
              <View style={styles.tiles}>
                <ActTile icon="wrench" label={t('무게 중심 보정')}
                  onPress={() => { onClose(); if (onZmpCalib) onZmpCalib(); else gaitApi.zmpCalibrate(ip).catch(() => {}); }} />
                {featureWheel && accessLevel >= 2 && (
                  <ActTile icon="ai" label="HIGH SPEED"
                    onPress={() => { onClose(); gaitApi.aiWalk(ip, RL_GAIT.RL_TROT_RUN).catch(() => {}); }} />
                )}
              </View>
            </>
          )}
          {danger && accessLevel >= 2 && (
            <>
              <Text style={[styles.groupLabel, { color: c.amberTx, marginTop: 14 }]}>{t('포지션 제어')}</Text>
              <View style={styles.tiles}>
                {accessLevel >= 3 && <Tappable onPress={() => setRecOpen(true)}
                  style={[styles.tile, { backgroundColor: 'rgba(240,136,62,0.08)', borderColor: 'rgba(240,136,62,0.5)' }]}>
                  <Icon name="wrench" size={22} color={c.amberTx} />
                  <Text style={{ fontSize: 11, fontWeight: '600', color: c.amberTx }}>Manual Recovery</Text>
                </Tappable>}
                {([['lock', 'Lock Joints'], ['pos_sit', 'Position Sit'], ['pos_stand', 'Position Stand']] as const).map(([m, lb]) => (
                  <Tappable key={m} onPress={() => pick(m as MotionName)}
                    style={[styles.tile, { backgroundColor: 'rgba(240,136,62,0.08)', borderColor: 'rgba(240,136,62,0.5)' }]}>
                    <Icon name="wrench" size={22} color={c.amberTx} />
                    <Text style={{ fontSize: 11, fontWeight: '600', color: c.amberTx }}>{lb}</Text>
                  </Tappable>
                ))}
              </View>
            </>
          )}
        </ScrollView>
      </LinearGradient>
    </Modal>
  );
}

const styles = StyleSheet.create({
  popH: { flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 11 },
  defaultBtn: {
    marginTop: 8, paddingVertical: 7, borderWidth: 1, borderRadius: 8, alignItems: 'center',
  },
  presetRow: { flexDirection: 'row', flexWrap: 'wrap', gap: 5, marginBottom: 11, width: 208 },
  presetBtn: { paddingHorizontal: 9, paddingVertical: 5, borderRadius: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  slTop: { flexDirection: 'row', justifyContent: 'space-between', alignItems: 'baseline', marginBottom: 2 },
  obsMapLink: { flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 7, height: 34, borderWidth: 1, marginBottom: 10, width: 208 },
  sigRow: { flexDirection: 'row', alignItems: 'center', gap: 9, paddingVertical: 6, borderTopWidth: 1 },
  sigDot: { width: 9, height: 9, borderRadius: 5 },
  modal: { width: 404, borderWidth: 1, borderRadius: 18, padding: 26, alignItems: 'center' },
  fill: { flex: 1, minHeight: 0 },
  estopIc: { width: 56, height: 56, borderRadius: 28, borderWidth: 1, alignItems: 'center', justifyContent: 'center', marginBottom: 15 },
  btn: { flex: 1, minWidth: 96, height: 46, borderRadius: 12, borderWidth: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 7 },
  grid: { width: 474, maxHeight: 430, borderWidth: 1, borderRadius: 18, overflow: 'hidden' },
  gridHead: { flexDirection: 'row', alignItems: 'center', justifyContent: 'space-between', padding: 16, borderBottomWidth: 1 },
  gridClose: { width: 30, height: 30, borderRadius: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  tilesWrap: { padding: 18 },
  groupLabel: { fontSize: 10, fontWeight: '700', letterSpacing: 0.6, marginBottom: 10 },
  tiles: { flexDirection: 'row', flexWrap: 'wrap', gap: 11 },
  tile: { width: 132, height: 74, borderRadius: 13, borderWidth: 1, alignItems: 'center', justifyContent: 'center', gap: 7 },
  recWide: { borderWidth: 1, borderRadius: 18, overflow: 'hidden' },
  recClose: { position: 'absolute', top: 8, right: 8, width: 28, height: 28, borderRadius: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
