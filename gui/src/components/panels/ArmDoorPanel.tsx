import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, Pressable, PanResponder, Modal } from 'react-native';
import { MODAL_ORIENTATIONS } from '@/components/ui/overlays';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { CameraView } from '@/components/CameraView';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { useHasArm } from '@/store/capability';
import { visionRequest } from '@/lib/vision';
import { connection } from '@/lib/connection';
import { actions, VISION_PROGRAM } from '@/lib/rest';
import { HANDEYE_STREAM_ID } from '@/lib/visionSources';
import { arm, ARM_MISSION, type DoorParams, type Pose6 } from '@/lib/arm';
import { t } from '@/lib/i18n';
import { useDeviceToggles } from '@/store/deviceToggles';
const BLUE = '#4d9cf5';
const ORANGE = '#ff9d40';

type Aim = 'none' | 'door' | 'handle';
type XhState = { cx: number; cy: number; size: number };

function Crosshair({ color, label, st, setSt, box, onConfirm }: {
  color: string; label: string; st: XhState; setSt: (f: (p: XhState) => XhState) => void;
  box: { w: number; h: number }; onConfirm: () => void;
}) {
  const stRef = useRef(st); stRef.current = st;
  const boxRef = useRef(box); boxRef.current = box;
  const start = useRef(st);
  const move = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => true,
      onPanResponderGrant: () => { start.current = stRef.current; },
      onPanResponderMove: (_e, g) => setSt((p) => ({
        ...p,
        cx: Math.min(Math.max(start.current.cx + g.dx, 0), boxRef.current.w),
        cy: Math.min(Math.max(start.current.cy + g.dy, 0), boxRef.current.h),
      })),
    }),
  );
  const resize = useRef(
    PanResponder.create({
      onStartShouldSetPanResponder: () => true,
      onPanResponderGrant: () => { start.current = stRef.current; },
      onPanResponderMove: (_e, g) => setSt((p) => ({
        ...p,
        size: Math.min(Math.max(start.current.size + g.dx, 44), Math.max(boxRef.current.h * 0.9, 60)),
      })),
    }),
  );

  const half = st.size / 2;
  return (
    <View
      {...move.current.panHandlers}
      style={{ position: 'absolute', left: st.cx - half, top: st.cy - half, width: st.size, height: st.size }}
    >
      <View style={{ position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, borderWidth: 1.5, borderColor: color, borderRadius: st.size / 2 }} />
      <View style={{ position: 'absolute', left: '50%', top: -6, bottom: -6, width: 1.5, backgroundColor: color, transform: [{ translateX: -0.75 }] }} />
      <View style={{ position: 'absolute', top: '50%', left: -6, right: -6, height: 1.5, backgroundColor: color, transform: [{ translateY: -0.75 }] }} />
      <Text style={{ position: 'absolute', left: st.size + 8, top: 0, color, fontSize: 10, fontWeight: '700', width: 90 }}>{label}</Text>
      <Pressable onPress={onConfirm} hitSlop={8}
        style={{ position: 'absolute', left: -6, bottom: -6, width: 26, height: 26, borderRadius: 13, backgroundColor: color, alignItems: 'center', justifyContent: 'center' }}>
        <Text style={{ color: '#fff', fontSize: 14, fontWeight: '800' }}>✓</Text>
      </Pressable>
      <View {...resize.current.panHandlers}
        style={{ position: 'absolute', right: -6, bottom: -6, width: 22, height: 22, borderRadius: 6, borderWidth: 1.5, borderColor: color, backgroundColor: 'rgba(0,0,0,0.45)', alignItems: 'center', justifyContent: 'center' }}>
        <Text style={{ color, fontSize: 10, fontWeight: '800' }}>⤡</Text>
      </View>
    </View>
  );
}

function Step({ n, label, sub, on, border, disabled, onPress, flex = 1 }: {
  n: string; label: string; sub?: string; on?: boolean; border?: string; disabled?: boolean;
  onPress: () => void; flex?: number;
}) {
  const { c, radius } = useTheme();
  return (
    <Tappable onPress={onPress} disabled={disabled} style={[styles.step, {
      flex, borderRadius: radius.md, opacity: disabled ? 0.4 : 1,
      borderWidth: border ? 2 : 1,
      borderColor: border ?? (on ? 'rgba(210,153,34,0.65)' : c.line),
      backgroundColor: on ? 'rgba(210,153,34,0.16)' : c.elev,
    }]}>
      <Text style={{ color: on ? c.amberTx : c.text, fontSize: 10, fontWeight: '700' }}>
        <Text style={{ color: c.dim, fontSize: 8 }}>{n} </Text>{label}
      </Text>
      {!!sub && <Text style={{ color: c.dim, fontSize: 8 }}>{sub}</Text>}
    </Tappable>
  );
}

const fmt = (p?: Pose6) => p ? p.map((v) => v.toFixed(2)).join(' ') : '—';

export function ArmDoorPanel({ onExit }: {
  onExit?: () => void;
}) {
  const { c, fonts, radius } = useTheme();
  const hasArm = useHasArm();
  const missionType = useTelemetry((s) => s.robot?.armStat?.missionType);
  const doorPose = useTelemetry((s) => s.doorPose);
  const handlePose = useTelemetry((s) => s.doorHandlePose);

  const handleType = useDeviceToggles((s) => s.doorHandleType);
  const hingeSide = useDeviceToggles((s) => s.doorHingeSide);
  const openType = useDeviceToggles((s) => s.doorOpenType);
  const params: DoorParams = { handleType, hingeSide, openType };
  const DOOR_KEY = { handleType: 'doorHandleType', hingeSide: 'doorHingeSide', openType: 'doorOpenType' } as const;
  const setParam = (key: keyof DoorParams, v: number) => useDeviceToggles.setState({ [DOOR_KEY[key]]: v });
  const [aim, setAim] = useState<Aim>('none');
  const [typeOpen, setTypeOpen] = useState(false);
  const [manualOpen, setManualOpen] = useState(false);
  const [lastStep, setLastStep] = useState('');
  const [box, setBox] = useState({ w: 1, h: 1 });
  const [xhDoor, setXhDoor] = useState<XhState>({ cx: 200, cy: 120, size: 130 });
  const [xhHandle, setXhHandle] = useState<XhState>({ cx: 320, cy: 160, size: 80 });

  const ip = useRobot((s) => s.ip);
  useEffect(() => {
    if (!hasArm) return;
    connection.sendMotion('walk', true);
    actions.visionProgram(ip, VISION_PROGRAM.Handeye, true).catch(() => {});
    return () => { actions.visionProgram(ip, VISION_PROGRAM.Handeye, false).catch(() => {}); };
  }, [hasArm, ip]);

  const typed = params.handleType !== 0 && params.hingeSide !== 0 && params.openType !== 0;
  const norm = (xh: XhState) => ({ x: xh.cx / box.w, y: xh.cy / box.h, r: (xh.size / box.w) * 1920 / 2500 });

  const confirmDoor = () => {
    const { x, y, r } = norm(xhDoor);
    visionRequest.calcDoorPose(x, y, r);
    setAim('none');
  };
  const confirmHandle = () => {
    arm.doorDetect();
    const { x, y, r } = norm(xhHandle);
    visionRequest.calcDoorHandlePose(x, y, r);
    setAim('none');
  };

  const dockBorder = missionType === 13 ? 'rgba(63,185,80,0.7)' : missionType === 12 ? 'rgba(210,153,34,0.8)' : missionType === 11 ? 'rgba(231,51,28,0.8)' : undefined;
  const apprBorder = missionType === 3 ? 'rgba(63,185,80,0.7)' : (missionType === 7 || missionType === 8 || missionType === 9) ? 'rgba(210,153,34,0.8)' : missionType === 10 ? 'rgba(231,51,28,0.8)' : undefined;

  const run = (step: string, fn: () => void) => () => { setLastStep(step); fn(); };
  const typeLabel = typed
    ? `${params.handleType === 1 ? t('레버') : t('원형')} · ${params.hingeSide === 1 ? t('좌') : t('우')} · ${params.openType === 1 ? t('밀기') : t('당기기')}`
    : t('미선택');

  return (
    <>
      <View style={styles.wrap}>
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, marginBottom: 8 }}>
          <Text style={{ color: c.text, fontSize: 13, fontWeight: '700' }}>{t('🚪 문 열기')}</Text>
          <Text style={{ color: c.dim, fontSize: 9.5, flex: 1 }}>
            {t('단계 순서대로 진행 — 도킹·접근 테두리색 = 미션 상태(초록 성공/노랑 재시도/빨강 오류)')}
          </Text>
          {onExit && (
            <Tappable onPress={() => { arm.goMotion('Folding', 3); onExit(); }}
              style={[styles.chip, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
              <Icon name="x" size={12} color={c.muted} />
              <Text style={{ color: c.text, fontSize: 10, fontWeight: '600' }}>{t('종료 (팔 접기)')}</Text>
            </Tappable>
          )}
        </View>

        <View
          style={[styles.cam, { borderColor: c.line, borderRadius: radius.md }]}
          onLayout={(e) => setBox({ w: e.nativeEvent.layout.width, h: e.nativeEvent.layout.height })}
        >
          <CameraView streamId={HANDEYE_STREAM_ID} />
          {aim === 'door' && (
            <Crosshair color={BLUE} label={t('문 중심 · ✓ 확정')} st={xhDoor} setSt={(f) => setXhDoor(f)} box={box} onConfirm={confirmDoor} />
          )}
          {aim === 'handle' && (
            <Crosshair color={ORANGE} label={t('손잡이 · ✓ 확정')} st={xhHandle} setSt={(f) => setXhHandle(f)} box={box} onConfirm={confirmHandle} />
          )}
          <Text style={[styles.pose, { left: 10, color: c.dim, fontFamily: fonts.mono }]}>doorPose {fmt(doorPose)}</Text>
          <Text style={[styles.pose, { right: 10, color: c.dim, fontFamily: fonts.mono }]}>handlePose {fmt(handlePose)}</Text>
          {missionType !== undefined && (
            <View style={[styles.mission, { borderColor: c.line, backgroundColor: 'rgba(0,0,0,0.5)' }]}>
              <Text style={{ color: c.muted, fontSize: 8.5, fontWeight: '700' }}>MISSION: {ARM_MISSION[missionType] ?? missionType}</Text>
            </View>
          )}
        </View>

        <View style={{ flexDirection: 'row', gap: 6, marginTop: 8 }}>
          <Step n="1" label={t('문 종류')} sub={typeLabel} on={typeOpen} onPress={() => setTypeOpen(true)} />
          <Step n="2" label={t('문 중심 지정')} sub={t('크로스헤어')} on={aim === 'door'} onPress={() => setAim(aim === 'door' ? 'none' : 'door')} />
          <Step n="3" label={t('자동 도킹')} sub="DOCKING 211" border={dockBorder} disabled={!doorPose || !typed}
            on={lastStep === 'dock'} onPress={run('dock', () => arm.doorAutoDocking(params, doorPose!))} />
          <View style={{ flex: 1, minWidth: 0, height: 52, borderWidth: 1, borderColor: c.line, borderRadius: radius.md, overflow: 'hidden' }}>
            <Tappable onPress={() => { arm.aimingMode(); arm.goMotion('DoorReady'); }}
              style={{ flex: 1, alignItems: 'center', justifyContent: 'center', backgroundColor: 'rgba(210,153,34,0.14)' }}>
              <Text style={{ color: c.amberTx, fontSize: 9, fontWeight: '700' }}>{t('탐색 자세')}</Text>
            </Tappable>
            <Tappable onPress={() => arm.goMotion('Folding', 2)}
              style={{ flex: 1, alignItems: 'center', justifyContent: 'center', backgroundColor: c.elev }}>
              <Text style={{ color: c.muted, fontSize: 9, fontWeight: '700' }}>{t('보행 자세')}</Text>
            </Tappable>
          </View>
          <Step n="4" label={t('손잡이 지정')} sub={t('크로스헤어')} disabled={!typed} on={aim === 'handle'}
            onPress={() => setAim(aim === 'handle' ? 'none' : 'handle')} />
          <Step n="5" label={t('접근')} sub="APPROACH 201" border={apprBorder} disabled={!handlePose || !typed}
            on={lastStep === 'appr'} onPress={run('appr', () => arm.doorApproach(params, handlePose!))} />
          <Step n="·" label={t('미세 조절')} sub={t('스텝 이동')} on={manualOpen} onPress={() => setManualOpen(true)} />
          <Step n="6" label={t('잡기')} sub="CATCH 202" disabled={!handlePose || !typed}
            on={lastStep === 'catch'} onPress={run('catch', () => arm.doorCatch(params, handlePose!))} />
          <Step n="7" label={t('열기')} sub="OPEN 203" disabled={!handlePose || !typed}
            on={lastStep === 'open'} onPress={run('open', () => arm.doorOpen(params, handlePose!))} />
          <Step n="8" label={t('완료')} sub="FINISH 204" on={lastStep === 'fin'} onPress={run('fin', () => arm.doorFinish())} />
        </View>

        <Text style={{ color: c.amberTx, fontSize: 9.5, marginTop: 6 }}>
          {t('⚠ 좌표 계산(✓)은 로봇의 Handeye(깊이 카메라)가 응답해야 pose가 갱신됩니다. 문 종류 3가지를 모두 선택해야 손잡이 단계가 열립니다.')}
        </Text>
      </View>

      <Modal supportedOrientations={MODAL_ORIENTATIONS} visible={typeOpen} transparent animationType="fade" onRequestClose={() => setTypeOpen(false)}>
        <Pressable style={styles.scrim} onPress={() => setTypeOpen(false)}>
          <Pressable style={[styles.popup, { backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.lg }]} onPress={() => {}}>
            <View style={{ flexDirection: 'row', alignItems: 'center' }}>
              <Text style={{ color: c.text, fontSize: 12, fontWeight: '700' }}>{t('문 종류 선택')}</Text>
              <Text style={{ color: c.dim, fontSize: 9, marginLeft: 8, flex: 1 }}>{t('3가지 모두 선택해야 손잡이 지정 가능')}</Text>
              <Tappable onPress={() => setTypeOpen(false)}><Icon name="x" size={14} color={c.muted} /></Tappable>
            </View>
            <View style={{ flexDirection: 'row', gap: 10, marginTop: 10 }}>
              {([
                [t('경첩 위치'), 'hingeSide', [t('왼쪽'), t('오른쪽')]],
                [t('문 형태'), 'openType', [t('밀기 (Push)'), t('당기기 (Pull)')]],
                [t('손잡이'), 'handleType', [t('레버 (수평)'), t('원형')]],
              ] as const).map(([title, key, labels]) => (
                <View key={key} style={{ flex: 1, gap: 6 }}>
                  <Text style={{ color: c.dim, fontSize: 9.5, fontWeight: '700', textAlign: 'center' }}>{title}</Text>
                  {labels.map((lb, i) => {
                    const on = params[key] === i + 1;
                    return (
                      <Tappable key={lb} onPress={() => setParam(key, i + 1)}
                        style={[styles.typeCell, {
                          borderRadius: radius.sm,
                          borderColor: on ? 'rgba(210,153,34,0.7)' : c.line,
                          backgroundColor: on ? 'rgba(210,153,34,0.16)' : c.elev,
                        }]}>
                        <Text style={{ color: on ? c.amberTx : c.muted, fontSize: 11, fontWeight: '700' }}>{lb}</Text>
                      </Tappable>
                    );
                  })}
                </View>
              ))}
            </View>
          </Pressable>
        </Pressable>
      </Modal>
      <Modal supportedOrientations={MODAL_ORIENTATIONS} visible={manualOpen} transparent animationType="fade" onRequestClose={() => setManualOpen(false)}>
        <Pressable style={styles.scrim} onPress={() => setManualOpen(false)}>
          <Pressable style={[styles.popup, { width: 560, backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.lg }]} onPress={() => {}}>
            <View style={{ flexDirection: 'row', alignItems: 'center' }}>
              <Text style={{ color: c.text, fontSize: 12, fontWeight: '700' }}>{t('미세 조절')}</Text>
              <Text style={{ color: c.dim, fontSize: 9, marginLeft: 8, flex: 1 }}>{t('탭 1회 = 1cm / 5° 스텝 이동 (문 미션 유지)')}</Text>
              <Tappable onPress={() => setManualOpen(false)}><Icon name="x" size={14} color={c.muted} /></Tappable>
            </View>
            <View style={{ flexDirection: 'row', gap: 12, marginTop: 10 }}>
              <View style={{ flex: 1, gap: 5 }}>
                <Text style={{ color: c.dim, fontSize: 9.5, fontWeight: '700', textAlign: 'center' }}>{t('위치 XYZ · 1cm')}</Text>
                {([['X', 1], ['Y', 2], ['Z', 3]] as const).map(([nm, axis]) => (
                  <View key={nm} style={{ flexDirection: 'row', gap: 5, justifyContent: 'center' }}>
                    <Tappable onPress={() => arm.deltaXyz(axis, -0.01)} style={[styles.stepBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
                      <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>{nm} −</Text>
                    </Tappable>
                    <Tappable onPress={() => arm.deltaXyz(axis, 0.01)} style={[styles.stepBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
                      <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>{nm} +</Text>
                    </Tappable>
                  </View>
                ))}
              </View>
              <View style={{ flex: 1, gap: 5 }}>
                <Text style={{ color: c.dim, fontSize: 9.5, fontWeight: '700', textAlign: 'center' }}>{t('방향 RPY · 5°')}</Text>
                {([['Roll', 1], ['Pitch', 2], ['Yaw', 3]] as const).map(([nm, axis]) => (
                  <View key={nm} style={{ flexDirection: 'row', gap: 5, justifyContent: 'center' }}>
                    <Tappable onPress={() => arm.deltaRpy(axis, -5)} style={[styles.stepBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
                      <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>{nm} −</Text>
                    </Tappable>
                    <Tappable onPress={() => arm.deltaRpy(axis, 5)} style={[styles.stepBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
                      <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>{nm} +</Text>
                    </Tappable>
                  </View>
                ))}
              </View>
              <View style={{ width: 120, gap: 5 }}>
                <Text style={{ color: c.dim, fontSize: 9.5, fontWeight: '700', textAlign: 'center' }}>{t('그리퍼')}</Text>
                <Pressable onPressIn={() => arm.gripper('open')} onPressOut={() => arm.gripper('stop')}
                  style={({ pressed }) => [styles.stepBtn, { width: undefined, borderRadius: radius.sm, borderColor: 'rgba(63,185,80,0.5)', backgroundColor: pressed ? 'rgba(63,185,80,0.3)' : 'rgba(63,185,80,0.10)' }]}>
                  <Text style={{ color: c.greenTx, fontSize: 11, fontWeight: '700' }}>{t('열기')}</Text>
                </Pressable>
                <Pressable onPressIn={() => arm.gripper('close')} onPressOut={() => arm.gripper('stop')}
                  style={({ pressed }) => [styles.stepBtn, { width: undefined, borderRadius: radius.sm, borderColor: 'rgba(210,153,34,0.5)', backgroundColor: pressed ? 'rgba(210,153,34,0.3)' : 'rgba(210,153,34,0.10)' }]}>
                  <Text style={{ color: c.amberTx, fontSize: 11, fontWeight: '700' }}>{t('닫기')}</Text>
                </Pressable>
                <Text style={{ color: c.dim, fontSize: 8.5, textAlign: 'center' }}>{t('누르는 동안 · 떼면 정지')}</Text>
              </View>
            </View>
          </Pressable>
        </Pressable>
      </Modal>
    </>
  );
}

const styles = StyleSheet.create({
  wrap: { flex: 1, padding: 12 },
  chip: { flexDirection: 'row', alignItems: 'center', gap: 4, height: 26, paddingHorizontal: 10, borderWidth: 1 },
  cam: { flex: 1, borderWidth: 1, overflow: 'hidden', backgroundColor: '#101418' },
  pose: { position: 'absolute', bottom: 8, fontSize: 8.5 },
  mission: { position: 'absolute', right: 10, top: 8, paddingHorizontal: 7, paddingVertical: 2, borderRadius: 6, borderWidth: 1 },
  step: { minWidth: 0, height: 52, alignItems: 'center', justifyContent: 'center', gap: 1 },
  scrim: { flex: 1, backgroundColor: 'rgba(0,0,0,0.55)', alignItems: 'center', justifyContent: 'flex-end', paddingBottom: 90 },
  popup: { width: 470, borderWidth: 1, paddingHorizontal: 14, paddingVertical: 12 },
  typeCell: { height: 44, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
  stepBtn: { width: 74, height: 34, alignItems: 'center', justifyContent: 'center', borderWidth: 1 },
});
