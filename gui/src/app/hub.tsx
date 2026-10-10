import { useRobotKind } from '@/modules/registry';
import { useEffect, useRef, useState } from 'react';
import { View, StyleSheet, useWindowDimensions, Platform } from 'react-native';
import { RobotModel3D } from '@/components/RobotModel3D';
import { STANDING_RAD } from '@/lib/robotPose';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { RobotSheet } from '@/components/hub/RobotSheet';
import { HubTiles } from '@/components/hub/HubTiles';
import { AutoStartCard } from '@/components/hub/AutoStartCard';
import { AutoStartModal } from '@/components/control/overlays';
import { useRobot } from '@/store/robot';
import { useNotMine } from '@/lib/spectating';
import { useTelemetry } from '@/store/telemetry';
import { useRobots, useProfiles, useRobotReady, SIM_SERIAL } from '@/store/robots';
import { useCompactH } from '@/lib/layout';
import { isDemo } from '@/lib/demoFlag';
import { probeRobotLan, probeLocalSim, ROBOT_LAN_IP } from '@/lib/robotScan';
import { getSteamOSCached } from '@/lib/platformInfo';
import { askLocationOnce } from '@/lib/askLocationOnce';
import { restBase } from '@/lib/endpoints';
import { goTop } from '@/lib/nav';

let bootJumpDone = false;

const HERO_POSE = { joints: STANDING_RAD, rpy: [0, 0, 0] as [number, number, number] };
const HERO_ORBIT = { yaw: 2.3, pitch: 0.25 };
const LIVE_ORBIT = { yaw: -0.7, pitch: 0.35 };

export default function Hub() {
  const { c } = useTheme();
  const compactH = useCompactH();
  const { width, height } = useWindowDimensions();
  const tall = !compactH && width / height < 1.7;
  const [sheet, setSheet] = useState(false);
  const [autoStart, setAutoStart] = useState(false);
  const notMine = useNotMine();
  useEffect(() => { if (notMine) setAutoStart(false); }, [notMine]);
  const conn = useRobot((s) => s.conn);
  const authEpoch = useRobot((s) => s.authEpoch);
  const robotReady = useRobotReady();
  const controlOn = useRobot((s) => !!s.robot?.control_started);
  const hasJoints = useTelemetry((s) => !!s.robot?.joints?.length);
  const serial = useRobots((s) => s.currentSerial);
  const isSim = serial === SIM_SERIAL;
  const liveNow = conn === 'connected' && controlOn && hasJoints && !isSim;
  const [liveFor, setLiveFor] = useState<string | null>(null);
  const lastPose = useRef<typeof HERO_POSE | null>(null);
  useEffect(() => {
    if (!liveNow) return;
    if (serial) setLiveFor(serial);
    const keep = (r: ReturnType<typeof useTelemetry.getState>['robot']) => {
      if (r?.joints?.length) lastPose.current = { joints: r.joints.map((j) => j.position), rpy: [r.imu?.rpy?.[0] ?? 0, r.imu?.rpy?.[1] ?? 0, r.worldRpy?.[2] ?? 0] };
    };
    keep(useTelemetry.getState().robot);
    return useTelemetry.subscribe((st) => keep(st.robot));
  }, [liveNow, serial]);
  const live = !isSim && !!serial && liveFor === serial;
  const heroPose = liveNow ? undefined : live && lastPose.current ? lastPose.current : HERO_POSE;
  const profiles = useProfiles();
  const kind = useRobotKind();
  const via = useRobots((s) => s.local.find((p) => p.serial === s.currentSerial)?.lastVia);
  useState(() => { useRobots.getState().leaveSim(); return 0; });
  const [jumpWindow, setJumpWindow] = useState(!bootJumpDone && !isDemo());
  useEffect(() => { const t = setTimeout(() => setJumpWindow(false), 8000); return () => clearTimeout(t); }, []);
  useEffect(() => {
    if (!jumpWindow || bootJumpDone) return;
    if (conn === 'connected' && via === 'lan' && !isSim) { bootJumpDone = true; goTop('/'); }
  }, [jumpWindow, conn, via, isSim]);
  useEffect(() => { if (!jumpWindow) bootJumpDone = true; }, [jumpWindow]);
  const [heroReady, setHeroReady] = useState(false);
  useEffect(() => { const t = setTimeout(() => setHeroReady(true), 600); return () => clearTimeout(t); }, []);
  useEffect(() => { if (!isDemo()) void askLocationOnce(); }, []);
  useEffect(() => {
    if (conn !== 'disconnected') return;
    let alive = true;
    (async () => {
      const found = isDemo() ? null : await probeRobotLan({ base: restBase(ROBOT_LAN_IP) });
      if (!alive || useRobot.getState().conn !== 'disconnected') return;
      if (found !== null) { void useRobots.getState().adoptLanRobot(); return; }
      if (Platform.OS === 'web' && !getSteamOSCached() && await probeLocalSim()) {
        if (!alive || useRobot.getState().conn !== 'disconnected') return;
        void useRobots.getState().addLocalSim(); return;
      }
      const target = serial ?? profiles[0]?.serial;
      if (target) useRobots.getState().select(target, 'ask');
    })();
    return () => { alive = false; };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [authEpoch]);
  return (
    <Screen>
      <View style={[StyleSheet.absoluteFill, { left: '-30%', top: '6%' }]}>
        {kind?.HubModel ? <kind.HubModel fill />
          : heroReady && <RobotModel3D pose={heroPose} controls={false} listenPresets={false} showPresetRow={false} gridVisible={false}
          backgroundColor={c.bg} idleSpin={!live} groundShadow initialOrbit={live ? LIVE_ORBIT : HERO_ORBIT} initialDist={live ? (compactH ? 2.2 : tall ? 2.3 : 2.0) : (compactH ? 1.55 : tall ? 1.8 : 1.45)} />}
      </View>
      <HubHeader onOpenSheet={() => setSheet(true)} />
      <View pointerEvents="box-none" style={[styles.body, compactH && { padding: 10, paddingTop: 0 }]}>
        <View pointerEvents="box-none" style={styles.left}>
          {!kind && robotReady && !controlOn && !isSim && !notMine && <AutoStartCard onPress={() => setAutoStart(true)} />}
        </View>
        <HubTiles compact={compactH} />
      </View>
      <RobotSheet visible={sheet} onClose={() => setSheet(false)} />
      {autoStart && <AutoStartModal onClose={() => setAutoStart(false)} />}
    </Screen>
  );
}
const styles = StyleSheet.create({
  body: { flex: 1, flexDirection: 'row', alignItems: 'flex-end', justifyContent: 'space-between', padding: 16, paddingTop: 8 },
  left: { flex: 1, justifyContent: 'flex-end', paddingBottom: 4 },
});
