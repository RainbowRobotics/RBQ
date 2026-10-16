import { Component, useEffect, useMemo, useRef, type ReactNode } from 'react';
import { View, Text, PanResponder } from 'react-native';
import { useTheme } from '@/theme';
import { Canvas, useFrame } from '@/lib/r3fCanvas';
import { useRobotMeshes, useAttMeshes, type AttMeshKey } from '@/lib/robotMeshes';
import { useRobotTextures } from '@/lib/robotTextures';
import { MeshPart, Leg } from '@/components/RobotModel3D';
import * as THREE from 'three';
import type { Object3D } from 'three';
import { LEGS, d2r } from '@/lib/robotPose';
import { clampAxis, type Axis, type PayloadRow } from '@/lib/payload';
import { t } from '@/lib/i18n';

function Axis({ rot, color, len = 0.55 }: { rot: [number, number, number]; color: string; len?: number }) {
  return (
    <group rotation={rot}>
      <mesh position={[0, len / 2, 0]}>
        <cylinderGeometry args={[0.006, 0.006, len, 8]} />
        <meshBasicMaterial color={color} />
      </mesh>
      <mesh position={[0, len, 0]}>
        <coneGeometry args={[0.02, 0.06, 10]} />
        <meshBasicMaterial color={color} />
      </mesh>
    </group>
  );
}

function SlotMarker({ p, color, box }: {
  p: { x: number; y: number; z: number }; color: string; box: boolean;
}) {
  return (
    <mesh position={[p.x, p.z, -p.y]}>
      {box
        ? <boxGeometry args={[0.05, 0.05, 0.05]} />
        : <cylinderGeometry args={[0.032, 0.032, 0.05, 14]} />}
      <meshBasicMaterial color={color} />
    </mesh>
  );
}

function PtzMarker({ p, obj, tint }: {
  p: { x: number; y: number; z: number }; obj: Object3D; tint: string;
}) {
  const { node, mat } = useMemo(() => {
    const material = new THREE.MeshBasicMaterial();
    const clone = obj.clone(true);
    clone.traverse((o: any) => { if (o.isMesh) o.material = material; });
    return { node: clone, mat: material };
  }, [obj]);
  useEffect(() => { mat.color.set(tint); }, [mat, tint]);
  useEffect(() => () => mat.dispose(), [mat]);
  return (
    <primitive object={node} position={[p.x, p.z, -p.y]}
      rotation={[-Math.PI / 2, 0, 0]} scale={[0.001, 0.001, 0.001]} />
  );
}

const MESH_FOR: Record<string, AttMeshKey> = {
  PTZ_CAM: 'ptz', LIDAR_LIVOX1: 'mid360', LIDAR_LIVOX2: 'mid360', LIDAR_OUSTER: 'ouster',
};
const SENSOR_ROT: Record<string, [number, number, number]> = {
  mid360: [0, 0, 0], ouster: [-Math.PI / 2, 0, 0],
};
const MOUNT_ROT: Record<string, [number, number, number]> = {
  LIDAR_LIVOX1: [0, 0, d2r(-35)],
  LIDAR_LIVOX2: [0, 0, d2r(35)],
};
const UPRIGHT: [number, number, number] = [0, 0, 0];

function SensorMarker({ p, obj, rot, mount, selected }: {
  p: { x: number; y: number; z: number }; obj: Object3D;
  rot: [number, number, number]; mount: [number, number, number]; selected: boolean;
}) {
  const { node, meshes } = useMemo(() => {
    const clone = obj.clone(true);
    const list: { mesh: any; mat: any }[] = [];
    clone.traverse((o: any) => { if (o.isMesh) list.push({ mesh: o, mat: o.material }); });
    return { node: clone, meshes: list };
  }, [obj]);
  const selMat = useMemo(() => new THREE.MeshBasicMaterial({ color: '#e8443b' }), []);
  useEffect(() => () => selMat.dispose(), [selMat]);
  useEffect(() => {
    for (const m of meshes) m.mesh.material = selected ? selMat : m.mat;
  }, [meshes, selMat, selected]);
  return (
    <group position={[p.x, p.z, -p.y]} rotation={mount}>
      <primitive object={node} rotation={rot} />
    </group>
  );
}

function TotalComMarker({ p }: { p: { x: number; y: number; z: number } }) {
  return (
    <mesh position={[p.x, p.z, -p.y]}>
      <sphereGeometry args={[0.035, 16, 12]} />
      <meshBasicMaterial color="#a855c7" transparent opacity={0.8} />
    </mesh>
  );
}

type Orbit = { yaw: number; pitch: number; dist: number };

const ROT_PER_PX = 0.01;
const WHEEL_GAIN = 0.0012;
const PITCH_RANGE: [number, number] = [-1.2, 1.2];
const DIST_RANGE: [number, number] = [0.6, 6];
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

function CameraRig({ orbit, camRef }: {
  orbit: React.MutableRefObject<Orbit>;
  camRef: React.MutableRefObject<any>;
}) {
  useFrame(({ camera }: any) => {
    const o = orbit.current;
    const r = o.dist * Math.cos(o.pitch);
    camera.position.set(r * Math.sin(o.yaw), o.dist * Math.sin(o.pitch), r * Math.cos(o.yaw));
    camera.lookAt(0, 0, 0);
    camRef.current = camera;
  });
  return null;
}

const GIZMO_ARM = 0.09;
const GIZMO_AXES: { k: Axis; rot: [number, number, number]; color: string }[] = [
  { k: 'x', rot: [0, 0, -Math.PI / 2], color: '#e8443b' },
  { k: 'z', rot: [0, 0, 0], color: '#4a9fe0' },
  { k: 'y', rot: [-Math.PI / 2, 0, 0], color: '#4caf50' },
];

function Gizmo({ p }: { p: { x: number; y: number; z: number } }) {
  return (
    <group position={[p.x, p.z, -p.y]}>
      {GIZMO_AXES.map(({ k, rot, color }) => (
        <group key={k} rotation={rot}>
          <mesh>
            <cylinderGeometry args={[0.004, 0.004, GIZMO_ARM * 2, 8]} />
            <meshBasicMaterial color={color} />
          </mesh>
          {[1, -1].map((sgn) => (
            <mesh key={sgn} position={[0, GIZMO_ARM * sgn, 0]} rotation={[sgn > 0 ? 0 : Math.PI, 0, 0]}>
              <coneGeometry args={[0.016, 0.045, 10]} />
              <meshBasicMaterial color={color} />
            </mesh>
          ))}
        </group>
      ))}
    </group>
  );
}

const axisSceneDir = (k: Axis): [number, number, number] =>
  (k === 'x' ? [1, 0, 0] : k === 'z' ? [0, 1, 0] : [0, 0, -1]);

function project(cam: any, s: [number, number, number], w: number, h: number) {
  if (!cam || w <= 0 || h <= 0) return null;
  const v = new THREE.Vector3(s[0], s[1], s[2]).project(cam);
  if (!Number.isFinite(v.x) || !Number.isFinite(v.y)) return null;
  return { x: (v.x * 0.5 + 0.5) * w, y: (1 - (v.y * 0.5 + 0.5)) * h };
}

const toScene = (p: { x: number; y: number; z: number }): [number, number, number] => [p.x, p.z, -p.y];

const SIT_RAD: number[] = [0, 90, -150, 0, 90, -150, 0, 90, -150, 0, 90, -150].map(d2r);

function RobotGhost() {
  const meshes = useRobotMeshes();
  const tex = useRobotTextures();
  if (!meshes) return null;
  return (
    <group position={[0, 0.0, 0]}>
      <MeshPart obj={meshes.trunk} rot={[d2r(-90), 0, 0]} texture={tex?.trunk} ghost />
      {LEGS.map((def) => (
        <Leg key={def.name} def={def} joints={SIT_RAD} meshes={meshes} tex={null} ghost />
      ))}
    </group>
  );
}

class GlFallback extends Component<{ children: ReactNode; fallback: ReactNode }, { failed: boolean }> {
  state = { failed: false };
  static getDerivedStateFromError() { return { failed: true }; }
  render() { return this.state.failed ? this.props.fallback : this.props.children; }
}

export function PayloadPreview3D({ rows, total, selectedId = -1, onSelect, onMove, height = 260 }: {
  rows: PayloadRow[];
  total: { mass: number; x: number; y: number; z: number } | null;
  selectedId?: number;
  onSelect?: (id: number) => void;
  onMove?: (id: number, k: Axis, v: number) => void;
  height?: number;
}) {
  const { c, radius } = useTheme();
  const att = useAttMeshes(rows.filter((r) => r.enabled && MESH_FOR[r.name])
                               .map((r) => MESH_FOR[r.name]));
  const orbit = useRef<Orbit>({ yaw: 0.7, pitch: 0.373, dist: 1.235 });
  const wrap = useRef<View>(null);
  const pinch = useRef<number | null>(null);
  const press = useRef<null | { x: number; y: number; gizmo: boolean }>(null);
  const last = useRef({ x: 0, y: 0 });
  const cam = useRef<any>(null);
  const size = useRef({ w: 0, h: 0 });
  const live = useRef({ rows, selectedId, onSelect, onMove });
  live.current = { rows, selectedId, onSelect, onMove };
  const drag = useRef<null | {
    id: number; k: Axis; start: number; unitPerPx: number; dirX: number; dirY: number;
  }>(null);

  const pickGizmo = (px: number, py: number) => {
    const { rows: rs, selectedId: sid } = live.current;
    const row = rs.find((r) => r.id === sid);
    if (!row || !row.enabled) return null;
    const { w, h } = size.current;
    const b = toScene(row);
    const base = project(cam.current, b, w, h);
    if (!base) return null;
    let best: { id: number; k: Axis; d: number } | null = null;
    for (const { k } of GIZMO_AXES) {
      const dir = axisSceneDir(k);
      for (const sgn of [1, -1]) {
        const tip = project(cam.current,
          [b[0] + dir[0] * GIZMO_ARM * sgn, b[1] + dir[1] * GIZMO_ARM * sgn, b[2] + dir[2] * GIZMO_ARM * sgn], w, h);
        if (!tip) continue;
        const armPx = Math.hypot(tip.x - base.x, tip.y - base.y);
        const radius = Math.min(26, armPx * 0.45);
        const d = Math.hypot(tip.x - px, tip.y - py);
        if (d < radius && (!best || d < best.d)) best = { id: row.id, k, d };
      }
    }
    return best;
  };

  const pickMarker = (px: number, py: number) => {
    const { w, h } = size.current;
    let bestId = -1, bestD = 44;
    for (const r of live.current.rows) {
      if (!r.enabled) continue;
      const p = project(cam.current, toScene(r), w, h);
      if (!p) continue;
      const d = Math.hypot(p.x - px, p.y - py);
      if (d < bestD) { bestD = d; bestId = r.id; }
    }
    return bestId;
  };

  const beginAxisDrag = (id: number, k: Axis) => {
    const row = live.current.rows.find((r) => r.id === id);
    if (!row) return;
    const { w, h } = size.current;
    const b = toScene(row);
    const dir = axisSceneDir(k);
    const p0 = project(cam.current, b, w, h);
    const p1 = project(cam.current, [b[0] + dir[0] * 0.1, b[1] + dir[1] * 0.1, b[2] + dir[2] * 0.1], w, h);
    if (!p0 || !p1) return;
    const lenPx = Math.hypot(p1.x - p0.x, p1.y - p0.y);
    if (lenPx < 2) return;
    drag.current = {
      id, k, start: row[k],
      unitPerPx: 0.1 / lenPx, dirX: (p1.x - p0.x) / lenPx, dirY: (p1.y - p0.y) / lenPx,
    };
  };

  const pan = useMemo(() => PanResponder.create({
    onStartShouldSetPanResponder: () => true,
    onMoveShouldSetPanResponder: () => true,
    onPanResponderGrant: (e) => {
      const { locationX: px, locationY: py } = e.nativeEvent;
      drag.current = null;
      last.current = { x: 0, y: 0 };
      pinch.current = null;
      const hit = pickGizmo(px, py);
      if (hit) beginAxisDrag(hit.id, hit.k);
      press.current = { x: px, y: py, gizmo: !!hit };
    },
    onPanResponderMove: (e, g) => {
      const dr = drag.current;
      if (dr) {
        const along = g.dx * dr.dirX + g.dy * dr.dirY;
        live.current.onMove?.(dr.id, dr.k, clampAxis(dr.start + along * dr.unitPerPx, dr.k));
        return;
      }
      const touches = e.nativeEvent.touches ?? [];
      if (touches.length >= 2) {
        const dx = touches[0].pageX - touches[1].pageX;
        const dy = touches[0].pageY - touches[1].pageY;
        const d = Math.hypot(dx, dy);
        if (pinch.current != null && d > 0) orbit.current.dist = clamp(orbit.current.dist * (pinch.current / d), ...DIST_RANGE);
        pinch.current = d;
        last.current = { x: g.dx, y: g.dy };
        return;
      }
      pinch.current = null;
      const ddx = g.dx - last.current.x;
      const ddy = g.dy - last.current.y;
      last.current = { x: g.dx, y: g.dy };
      orbit.current.yaw -= ddx * ROT_PER_PX;
      orbit.current.pitch = clamp(orbit.current.pitch + ddy * ROT_PER_PX, ...PITCH_RANGE);
    },
    onPanResponderRelease: (_e, g) => {
      const p = press.current;
      if (p && !p.gizmo && Math.abs(g.dx) < 6 && Math.abs(g.dy) < 6) {
        const id = pickMarker(p.x, p.y);
        if (id >= 0) live.current.onSelect?.(id);
      }
      press.current = null; pinch.current = null; drag.current = null;
    },
    onPanResponderTerminate: () => { press.current = null; pinch.current = null; drag.current = null; },
  }), []);

  useEffect(() => {
    const el = wrap.current as unknown as HTMLElement | null;
    if (!el || typeof el.addEventListener !== 'function') return;
    const onWheel = (e: WheelEvent) => {
      orbit.current.dist = clamp(orbit.current.dist * (1 + e.deltaY * WHEEL_GAIN), ...DIST_RANGE);
      e.preventDefault();
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  const selRow = rows.find((r) => r.id === selectedId && r.enabled) ?? null;

  return (
    <View ref={wrap}
      onLayout={(e) => { size.current = { w: e.nativeEvent.layout.width, h: e.nativeEvent.layout.height }; }}
      style={{ height, borderWidth: 1, borderColor: c.line, borderRadius: radius.md,
               overflow: 'hidden', backgroundColor: c.bg }}>
      <GlFallback fallback={<View style={{ flex: 1, alignItems: 'center', justifyContent: 'center' }}><Text style={{ color: c.dim, fontSize: 11 }}>{t('3D 미리보기 불가 (WebGL)')}</Text></View>}>
        <Canvas frameloop="always" camera={{ position: [0.8, 0.45, 0.8], fov: 42 }}>
          <ambientLight intensity={0.9} />
          <directionalLight position={[2, 3, 2]} intensity={0.8} />
          <CameraRig orbit={orbit} camRef={cam} />
          <gridHelper args={[2, 20, '#3a4654', '#2a3646']} position={[0, -0.18, 0]} />
          <RobotGhost />
          <Axis rot={[0, 0, -Math.PI / 2]} color="#e8443b" />
          <Axis rot={[-Math.PI / 2, 0, 0]} color="#4caf50" />
          <Axis rot={[0, 0, 0]} color="#4a9fe0" />
          {rows.filter((r) => r.enabled).map((r) => {
            const tint = r.id === selectedId ? '#e8443b' : r.isCustom ? '#f2c94c' : '#4a9fe0';
            const key = MESH_FOR[r.name];
            const obj = key ? att[key] : undefined;
            if (!obj) return <SlotMarker key={r.id} p={r} box={r.isCustom} color={tint} />;
            return key === 'ptz'
              ? <PtzMarker key={r.id} p={r} obj={obj} tint={tint} />
              : <SensorMarker key={r.id} p={r} obj={obj} rot={SENSOR_ROT[key]}
                              mount={MOUNT_ROT[r.name] ?? UPRIGHT}
                              selected={r.id === selectedId} />;
          })}
          {selRow && <Gizmo p={selRow} />}
          {total && <TotalComMarker p={total} />}
        </Canvas>
      </GlFallback>
      <View {...pan.panHandlers} style={{ position: 'absolute', inset: 0 as any }} />
      <View pointerEvents="none" style={{ position: 'absolute', left: 8, bottom: 6, right: 8, flexDirection: 'row', flexWrap: 'wrap', gap: 8 }}>
        <Text style={{ color: '#4a9fe0', fontSize: 9 }}>■ {t('고정')}</Text>
        <Text style={{ color: '#f2c94c', fontSize: 9 }}>■ {t('커스텀')}</Text>
        <Text style={{ color: '#a855c7', fontSize: 9 }}>● {t('무게중심')}</Text>
      </View>
    </View>
  );
}
