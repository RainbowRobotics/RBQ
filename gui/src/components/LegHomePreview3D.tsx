import { useEffect, useMemo, useRef, useState } from 'react';
import { View, Text, StyleSheet, PanResponder, ActivityIndicator } from 'react-native';
import type { Group } from 'three';
import { Canvas, useFrame } from '@/lib/r3fCanvas';
import { Leg, MeshPart, GlBoundary, FirstFrameSignal } from '@/components/RobotModel3D';
import { useRobotMeshes } from '@/lib/robotMeshes';
import { useRobotTextures } from '@/lib/robotTextures';
import { LEGS, OFFSETS, d2r } from '@/lib/robotPose';
import { t } from '@/lib/i18n';

export type Orbit = { yaw: number; pitch: number };

export type JointAnim = {
  moves: { axis: 0 | 1 | 2; fromDeg: number; toDeg: number }[];
  cycleKey: string;
};
const MOVE_MS = 2000;
const PAUSE_MS = 1000;

const CENTER_Y = 0.0;
const CAM_DIST = 1.1;

const ROLL_COMPENSATION = 0.6;

const YAW_BACK = Math.PI / 2;
const PITCH_BASE = 0.35;
function defaultOrbit(leg: number, rollDeg: number): Orbit {
  const def = LEGS[leg];
  const pitch = Math.max(-1.2, Math.min(1.2, PITCH_BASE - d2r(rollDeg * (def?.right ?? 1)) * ROLL_COMPENSATION));
  return { yaw: YAW_BACK, pitch };
}

function Rig({ leg, jointsRad, orbit }: {
  leg: number; jointsRad: number[]; orbit: React.MutableRefObject<Orbit>;
}) {
  const meshes = useRobotMeshes();
  const tex = useRobotTextures();
  const pitchG = useRef<Group>(null);
  const yawG = useRef<Group>(null);
  useFrame(({ camera }) => {
    if (pitchG.current) pitchG.current.rotation.x = orbit.current.pitch;
    if (yawG.current) yawG.current.rotation.y = orbit.current.yaw;
    camera.position.set(0, 0, CAM_DIST);
    camera.lookAt(0, 0, 0);
  });
  const def = LEGS[leg];
  if (!meshes || !def) return null;
  return (
    <group ref={pitchG}>
      <group ref={yawG}>
        <group position={[
          -def.front * (OFFSETS.centerToLegX + OFFSETS.centerToHipX),
          -CENTER_Y,
          -def.right * OFFSETS.centerToLegY,
        ]}>
          <MeshPart obj={meshes.trunk} rot={[d2r(-90), 0, 0]} ghost />
          <Leg def={def} joints={jointsRad} meshes={meshes} tex={tex} />
        </group>
      </group>
    </group>
  );
}

function useAnimK(anim?: JointAnim): number {
  const [k, setK] = useState(1);
  const key = anim?.cycleKey ?? '';
  useEffect(() => {
    if (!key) { setK(1); return; }
    let raf = 0;
    const t0 = Date.now();
    const loop = () => {
      const el = (Date.now() - t0) % (MOVE_MS + PAUSE_MS);
      const x = el < MOVE_MS ? el / MOVE_MS : 1;
      setK(x * x * (3 - 2 * x));
      raf = requestAnimationFrame(loop);
    };
    raf = requestAnimationFrame(loop);
    return () => cancelAnimationFrame(raf);
  }, [key]);
  return k;
}

export function LegHomePreview3D({ leg, poseDeg, anim, orbitRollDeg, orbitRef, badge, height = 220 }: {
  leg: number;
  poseDeg: [number, number, number];
  anim?: JointAnim;
  orbitRollDeg?: number;
  orbitRef?: React.MutableRefObject<Orbit>;
  badge?: string;
  height?: number;
}) {
  const meshesReady = !!useRobotMeshes();
  const [drawn, setDrawn] = useState(false);
  const [glError, setGlError] = useState<string | null>(null);
  const orbitRoll = orbitRollDeg ?? poseDeg[0];
  const own = useRef<Orbit>(defaultOrbit(leg, orbitRoll));
  const orbit = orbitRef ?? own;
  const last = useRef({ x: 0, y: 0 });
  useEffect(() => {
    orbit.current = defaultOrbit(leg, orbitRoll);
  }, [leg, orbitRoll]); // eslint-disable-line react-hooks/exhaustive-deps

  const k = useAnimK(anim);

  const moveKey = anim ? anim.moves.map((m) => `${m.axis}:${m.fromDeg}>${m.toDeg}`).join('|') : '';
  const jointsRad = useMemo(() => {
    const out = new Array(12).fill(0);
    const first = leg * 3;
    for (let i = 0; i < 3; i++) out[first + i] = d2r(poseDeg[i] ?? 0);
    for (const m of anim?.moves ?? []) {
      out[first + m.axis] = d2r(m.fromDeg + (m.toDeg - m.fromDeg) * k);
    }
    return out;
  }, [leg, poseDeg[0], poseDeg[1], poseDeg[2], k, moveKey]); // eslint-disable-line react-hooks/exhaustive-deps

  const responder = useMemo(
    () =>
      PanResponder.create({
        onStartShouldSetPanResponder: () => true,
        onMoveShouldSetPanResponder: () => true,
        onPanResponderGrant: () => { last.current = { x: 0, y: 0 }; },
        onPanResponderMove: (_e, g) => {
          const ddx = g.dx - last.current.x;
          const ddy = g.dy - last.current.y;
          last.current = { x: g.dx, y: g.dy };
          orbit.current.yaw += ddx * 0.01;
          orbit.current.pitch = Math.max(-1.2, Math.min(1.2, orbit.current.pitch + ddy * 0.01));
        },
      }),
    [orbit],
  );

  return (
    <View style={[styles.wrap, { height }]}>
      {glError && (
        <View style={styles.center}>
          <Text style={styles.note}>{glError}</Text>
        </View>
      )}
      {!drawn && !glError && (
        <View style={styles.center} pointerEvents="none">
          <ActivityIndicator size="small" color="#8a94a6" />
          <Text style={[styles.note, { marginTop: 6 }]}>{t('3D 모델 로딩 중…')}</Text>
        </View>
      )}
      {meshesReady && !glError && (
        <GlBoundary onError={setGlError}>
          <Canvas camera={{ position: [0, 0, CAM_DIST], fov: 50 }}
            onCreated={({ gl }) => { gl.debug.checkShaderErrors = false; }}>
            <ambientLight intensity={0.75} />
            <directionalLight position={[5, 5, 5]} intensity={1} />
            <directionalLight position={[-5, 2, -3]} intensity={0.4} />
            <FirstFrameSignal onDrawn={() => setDrawn(true)} />
            <Rig leg={leg} jointsRad={jointsRad} orbit={orbit} />
          </Canvas>
        </GlBoundary>
      )}
      <View style={StyleSheet.absoluteFill} {...responder.panHandlers} />
      {badge && (
        <View style={styles.badge} pointerEvents="none">
          <Text style={styles.badgeTx}>{badge}</Text>
        </View>
      )}
      <Text style={styles.hint}>{t('드래그해서 돌려보기')}</Text>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { width: '100%', borderRadius: 10, overflow: 'hidden', backgroundColor: 'rgba(0,0,0,0.25)' },
  center: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, alignItems: 'center', justifyContent: 'center', padding: 16 },
  note: { color: '#8a94a6', fontSize: 11, textAlign: 'center' },
  hint: { position: 'absolute', right: 8, bottom: 6, color: 'rgba(255,255,255,0.35)', fontSize: 9 },
  badge: { position: 'absolute', left: 8, top: 8, backgroundColor: 'rgba(0,0,0,0.42)', borderRadius: 6, paddingHorizontal: 8, paddingVertical: 4 },
  badgeTx: { color: 'rgba(255,255,255,0.88)', fontSize: 9.5, fontWeight: '700' },
});
