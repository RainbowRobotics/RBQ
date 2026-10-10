import { Canvas, useFrame, useThree } from '@/lib/r3fCanvas';
import { useSteamOS, getPlatformInfo } from '@/lib/platformInfo';
import { useSettings } from '@/store/settings';
import { Component, useCallback, useEffect, useLayoutEffect, useMemo, useRef, useState, type MutableRefObject, type ReactNode } from 'react';
import { View, Text, Pressable, StyleSheet, PanResponder, ActivityIndicator, AppState, Platform } from 'react-native';
import type { Group, Mesh, MeshStandardMaterial, Texture, InstancedMesh } from 'three';
import { Box3, BufferGeometry, BufferAttribute, CylinderGeometry, DoubleSide, Euler,
         SphereGeometry, MeshBasicMaterial, Matrix4, Quaternion, Vector3, PerspectiveCamera,
         OrthographicCamera as ThreeOrthographicCamera, Color, Object3D, InstancedBufferAttribute } from 'three';
import { Grid } from '@react-three/drei';
import { usePointCloud, type CloudFrame } from '@/lib/pointcloud';
import { useElevationMap } from '@/lib/elevationMap';
import { useHeightmapCloud, HmChannel, isHeightmapGait, HM_LAYERS_OFF, type HmFrame, type HmLayers } from '@/lib/heightmapCloud';
import { useRobotMeshes, useArmMeshes, useWheelMeshes, useRobotMeshError, type RobotMeshes, type ArmMeshes, type WheelMeshes } from '@/lib/robotMeshes';
import { useRobotTextures, type RobotTextures } from '@/lib/robotTextures';
import { OFFSETS, LEGS, d2r, STANDING_RAD, footCenters, type LegDef } from '@/lib/robotPose';
import { onOrbitPreset } from '@/lib/robotOrbit';
import { useTelemetry } from '@/store/telemetry';
import { useHasArm, useFeatureWheel } from '@/store/capability';
import { useView3d, type LidarMode } from '@/store/view3d';
import { useRobot } from '@/store/robot';
import { resolveEndpoints } from '@/lib/resolveEndpoints';
import { webrtcClient } from '@/lib/webrtcClient';
import { t } from '@/lib/i18n';

type Orbit = { yaw: number; pitch: number; local: boolean };

export const OBSMAP_3D_W = 0.7;

const CHASE_YAW_BASE = Math.PI / 2;

export function MeshPart({ obj, rot, texture, ghost }: {
  obj: Object3D; rot: [number, number, number]; texture?: Texture | null; ghost?: boolean;
}) {
  const cloned = useMemo(() => {
    const c = obj.clone(true);
    if (!ghost) c.traverse((o) => { if ((o as Mesh).isMesh) o.castShadow = true; });
    if (ghost || texture) {
      c.traverse((o) => {
        const mesh = o as Mesh;
        if (!mesh.isMesh) return;
        const src = (Array.isArray(mesh.material) ? mesh.material[0] : mesh.material) as MeshStandardMaterial;
        const mat = src.clone();
        if (ghost) {
          mat.map = null;
          mat.color.setHex(0x4d9cf5);
          mat.transparent = true;
          mat.opacity = 0.3;
          mat.depthWrite = false;
        } else if (texture) {
          mat.map = texture;
          mat.color.setRGB(1, 1, 1);
        }
        mat.needsUpdate = true;
        mat.userData.__cloned = true;
        mesh.material = mat;
      });
    }
    return c;
  }, [obj, texture, ghost]);
  useEffect(() => () => {
    cloned.traverse((o) => {
      const mesh = o as Mesh;
      if (!mesh.isMesh) return;
      const m = mesh.material as MeshStandardMaterial | MeshStandardMaterial[];
      (Array.isArray(m) ? m : [m]).forEach((mat) => { if (mat?.userData?.__cloned) mat.dispose(); });
    });
  }, [cloned]);
  return (
    <group rotation={[rot[0], rot[1], rot[2], 'YXZ']}>
      <primitive object={cloned} />
    </group>
  );
}

export function Leg({
  def,
  joints,
  meshes,
  tex,
  ghost,
  wheel,
}: {
  def: LegDef;
  joints: number[];
  meshes: RobotMeshes;
  tex: RobotTextures | null;
  ghost?: boolean;
  wheel?: WheelMeshes | null;
}) {
  const O = OFFSETS;
  const jAbd = joints[def.base] ?? 0;
  const jHip = joints[def.base + 1] ?? 0;
  const jKnee = joints[def.base + 2] ?? 0;
  const thighTex = tex ? (def.right === 1 ? tex.thighRight : tex.thighLeft) : null;
  return (
    <group position={[def.front * O.centerToLegX, 0, def.right * O.centerToLegY]}>
      <group position={[def.front * O.centerToHipX, 0, 0]} rotation={[jAbd, 0, 0, 'YXZ']}>
        <MeshPart obj={meshes[def.hip]} rot={[d2r(-90), d2r(def.hipRear ? 180 : 0), 0]} texture={tex?.hip} ghost={ghost} />
        <group position={[0, 0, def.right * O.hipToThigh]} rotation={[0, 0, -jHip, 'YXZ']}>
          <MeshPart obj={meshes.thigh} rot={[d2r(-90), 0, 0]} texture={thighTex} ghost={ghost} />
          <group position={[0, -O.thighToKnee, 0]} rotation={[0, 0, -jKnee, 'YXZ']}>
            <MeshPart obj={wheel ? (def.right === 1 ? wheel.calfRight : wheel.calfLeft) : meshes.calf}
              rot={[d2r(-90), 0, 0]} texture={tex?.calf} ghost={ghost} />
            {wheel && (
              <group position={[0, -O.kneeToWheel, def.right * O.wheelOffsetZ]}>
                <MeshPart obj={def.right === 1 ? wheel.wheelRight : wheel.wheelLeft}
                  rot={[d2r(-90), 0, 0]} texture={tex?.calf} ghost={ghost} />
              </group>
            )}
          </group>
        </group>
      </group>
    </group>
  );
}

const sane = (v?: number) => (v != null && isFinite(v) && Math.abs(v) < 100 ? v : 0);

const _feet = [[0, 0, 0], [0, 0, 0], [0, 0, 0], [0, 0, 0]];
const _p = new Vector3();
const _rot = new Euler();
function lowestY(pts: number[][], rot: Euler | null) {
  let lo = Infinity;
  for (const p of pts) {
    _p.set(p[0], p[1], p[2]);
    if (rot) _p.applyEuler(rot);
    if (_p.y < lo) lo = _p.y;
  }
  return lo;
}

const LIDAR_RAMP_MIN = -0.5;
const LIDAR_RAMP_MAX = 1.5;

function rampRGB(t: number, out: Float32Array, o: number) {
  if (t < 0.25)      { out[o] = 0;                 out[o + 1] = t * 4;                 out[o + 2] = 1; }
  else if (t < 0.5)  { out[o] = 0;                 out[o + 1] = 1;                     out[o + 2] = 1 - (t - 0.25) * 4; }
  else if (t < 0.75) { out[o] = (t - 0.5) * 4;     out[o + 1] = 1;                     out[o + 2] = 0; }
  else               { out[o] = 1;                 out[o + 1] = 1 - (t - 0.75) * 4;    out[o + 2] = 0; }
}

function LidarCloud({ cloud }: { cloud: CloudFrame }) {
  const geom = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(cloud.positions, 3));
    const colors = new Float32Array(cloud.count * 3);
    const span = LIDAR_RAMP_MAX - LIDAR_RAMP_MIN;
    for (let i = 0; i < cloud.count; i++) {
      const z = cloud.positions[i * 3 + 2];
      const t = span > 1e-6 ? Math.min(1, Math.max(0, (z - LIDAR_RAMP_MIN) / span)) : 0.5;
      rampRGB(t, colors, i * 3);
    }
    g.setAttribute('color', new BufferAttribute(colors, 3));
    return g;
  }, [cloud]);
  useEffect(() => () => geom.dispose(), [geom]);
  return (
    <points geometry={geom}>
      <pointsMaterial size={0.02} sizeAttenuation vertexColors />
    </points>
  );
}

function LidarClouds() {
  const clouds = usePointCloud((s) => s.clouds);
  const entries = Object.entries(clouds).filter(([, c]) => !!c) as [string, CloudFrame][];
  if (entries.length === 0) return null;
  return (
    <group rotation={[-Math.PI / 2, 0, 0]}>
      {entries.map(([id, c]) => <LidarCloud key={id} cloud={c} />)}
    </group>
  );
}

const ELEV_CYL_SIDES = 6;
function ElevationOverlay() {
  const frame = useElevationMap((s) => s.frame);
  const meshRef = useRef<InstancedMesh>(null);
  const dummy = useMemo(() => new Object3D(), []);
  const colorBufRef = useRef<{ arr: Float32Array; attr: InstancedBufferAttribute } | null>(null);
  const capacity = frame ? frame.rows * frame.cols : 0;

  const count = useMemo(() => {
    if (!frame) return 0;
    let n = 0;
    for (let i = 0; i < frame.valid.length; i++) if (frame.valid[i]) n++;
    return n;
  }, [frame]);


  useEffect(() => {
    const mesh = meshRef.current;
    if (!mesh || !frame) return;
    if (count === 0) { mesh.count = 0; return; }
    const { rows, cols, gs, originX, originY, height, valid, robotZ } = frame;
    const span = LIDAR_RAMP_MAX - LIDAR_RAMP_MIN;
    let baseY = Infinity;
    for (let i = 0; i < valid.length; i++) if (valid[i] && height[i] < baseY) baseY = height[i];
    const radius = Math.max(0.008, gs * 0.5);

    let buf = colorBufRef.current;
    if (!buf || buf.arr.length < capacity * 3) {
      const arr = new Float32Array(capacity * 3);
      buf = { arr, attr: new InstancedBufferAttribute(arr, 3) };
      colorBufRef.current = buf;
    }
    const colorArr = buf.arr;

    let o = 0;
    for (let cy = 0; cy < rows; cy++) {
      for (let cx = 0; cx < cols; cx++) {
        const idx = cy * cols + cx;
        if (!valid[idx]) continue;
        const worldX = originX + (cx + 0.5) * gs;
        const worldY = originY + (cy + 0.5) * gs;
        const h = height[idx];
        const top = Math.max(h, baseY + 0.01);
        const mid = (top + baseY) / 2;
        dummy.position.set(worldX, mid, -worldY);
        dummy.scale.set(radius, top - baseY, radius);
        dummy.updateMatrix();
        mesh.setMatrixAt(o, dummy.matrix);
        const t = span > 1e-6 ? Math.min(1, Math.max(0, (h - robotZ - LIDAR_RAMP_MIN) / span)) : 0.5;
        rampRGB(t, colorArr, o * 3);
        o++;
      }
    }
    mesh.count = count;
    mesh.instanceMatrix.needsUpdate = true;
    if (mesh.instanceColor !== buf.attr) mesh.instanceColor = buf.attr;
    buf.attr.needsUpdate = true;
  }, [frame, count, dummy, capacity]);

  if (capacity === 0) return null;
  return (
    <instancedMesh ref={meshRef} args={[undefined, undefined, capacity]} frustumCulled={false}>
      <cylinderGeometry args={[1, 1, 1, ELEV_CYL_SIDES]} />
      <meshStandardMaterial />
    </instancedMesh>
  );
}

const HM_STYLE: Record<HmChannel, { color: string; size: number; bar: number }> = {
  [HmChannel.Map]:        { color: '#5d8ac2', size: 0.03,  bar: 0.02 },
  [HmChannel.Edge]:       { color: '#4a4a4a', size: 0.055, bar: 0.02 },
  [HmChannel.Stair]:      { color: '#e69f00', size: 0.03,  bar: 0.025 },
  [HmChannel.StairEdge]:  { color: '#d55e00', size: 0.05,  bar: 0.012 },
  [HmChannel.FootQuery]:  { color: '#ff4d6a', size: 0.025, bar: 0.02 },
  [HmChannel.FootAnswer]: { color: '#35d97b', size: 0.025, bar: 0.02 },
};
const FOOT_SPHERE = new SphereGeometry(0.025, 12, 8);
const FOOT_MATS: Partial<Record<string, MeshBasicMaterial>> = {};
const footMat = (color: string) => (FOOT_MATS[color] ??= new MeshBasicMaterial({ color }));

const isRunFrame = (f: HmFrame) => f.count >= 2 && !!f.w && f.w[0] > 0.5;

const BAR_GEOM = (() => { const g = new CylinderGeometry(0.5, 0.5, 1, 8); g.rotateZ(Math.PI / 2); return g; })();
function HmBars({ frame, color, diameter }: { frame: HmFrame; color: string; diameter: number }) {
  const BAR_D = diameter;
  const n = Math.floor(frame.count / 2);
  const cap = Math.max(64, 1 << Math.ceil(Math.log2(Math.max(n, 1))));
  const ref = useRef<InstancedMesh | null>(null);
  const attach = useCallback((im: InstancedMesh | null) => {
    ref.current = im;
    if (im) im.count = 0;
  }, []);
  useLayoutEffect(() => {
    const im = ref.current;
    if (!im) return;
    const p = frame.positions;
    const m = new Matrix4(), q = new Quaternion(), pos = new Vector3(), scl = new Vector3();
    const dir = new Vector3();
    const xAxis = new Vector3(1, 0, 0);
    for (let i = 0; i < n; i++) {
      const a = i * 6;
      const ax = p[a], ay = p[a + 1], az = p[a + 2];
      const bx = p[a + 3], by = p[a + 4], bz = p[a + 5];
      const len = Math.hypot(bx - ax, by - ay, bz - az);
      pos.set((ax + bx) / 2, (ay + by) / 2, (az + bz) / 2);
      if (len > 1e-6) {
        dir.set((bx - ax) / len, (by - ay) / len, (bz - az) / len);
        q.setFromUnitVectors(xAxis, dir);
      } else {
        q.identity();
      }
      scl.set(len + BAR_D, BAR_D, BAR_D);
      im.setMatrixAt(i, m.compose(pos, q, scl));
    }
    im.count = n;
    im.instanceMatrix.needsUpdate = true;
  }, [frame, n]);
  return (
    <instancedMesh key={cap} ref={attach} args={[BAR_GEOM, undefined, cap]} frustumCulled={false}>
      <meshBasicMaterial color={color} />
    </instancedMesh>
  );
}

const GRID_HALF_X = 2.0;

function rangeColors(pos: Float32Array): Float32Array {
  const n = pos.length / 3;
  const out = new Float32Array(n * 3);
  for (let i = 0; i < n; i++) {
    const t = Math.min(1, Math.max(0, (pos[i * 3] + GRID_HALF_X) / (2 * GRID_HALF_X)));
    const h = 1.5 + (1 - t) * 2.5;
    const x = 1 - Math.abs((h % 2) - 1);
    let r = 0, g = 0, b = 0;
    if (h < 1)      { r = 1; g = x; }
    else if (h < 2) { r = x; g = 1; }
    else if (h < 3) { g = 1; b = x; }
    else            { g = x; b = 1; }
    out[i * 3] = r; out[i * 3 + 1] = g; out[i * 3 + 2] = b;
  }
  return out;
}

function HmPoints({ frame, color, size, rainbow }: {
  frame: HmFrame; color: string; size: number; rainbow?: boolean;
}) {
  const geom = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(frame.positions, 3));
    if (rainbow) g.setAttribute('color', new BufferAttribute(rangeColors(frame.positions), 3));
    return g;
  }, [frame, rainbow]);
  useEffect(() => () => geom.dispose(), [geom]);
  return (
    <points geometry={geom}>
      <pointsMaterial size={size} sizeAttenuation vertexColors={!!rainbow}
                      color={rainbow ? '#ffffff' : color} />
    </points>
  );
}

function HmSpheres({ frame, color }: { frame: HmFrame; color: string }) {
  const mat = footMat(color);
  const p = frame.positions;
  const els: ReactNode[] = [];
  for (let i = 0; i < frame.count; i++) {
    els.push(<mesh key={i} geometry={FOOT_SPHERE} material={mat} position={[p[i * 3], p[i * 3 + 1], p[i * 3 + 2]]} />);
  }
  return <>{els}</>;
}

function HeightmapClouds({ frames, layers }: { frames: Partial<Record<HmChannel, HmFrame>>; layers: HmLayers }) {
  const on = (ch: HmChannel) =>
    ch === HmChannel.Map || ch === HmChannel.Edge ? layers.grid
    : ch === HmChannel.Stair ? layers.stair
    : ch === HmChannel.StairEdge ? layers.edge
    : layers.foot;
  const els: ReactNode[] = [];
  for (const k of Object.keys(frames)) {
    const ch = Number(k) as HmChannel;
    const f = frames[ch];
    if (!f || f.count === 0 || !on(ch)) continue;
    const st = HM_STYLE[ch];
    els.push(ch >= HmChannel.FootQuery
      ? <HmSpheres key={k} frame={f} color={st.color} />
      : isRunFrame(f)
      ? <HmBars key={k} frame={f} color={st.color} diameter={st.bar} />
      : <HmPoints key={k} frame={f} color={st.color} size={st.size}
                  rainbow={ch === HmChannel.Map} />);
  }
  if (els.length === 0) return null;
  return <group rotation={[-Math.PI / 2, 0, 0]}>{els}</group>;
}

function ArmRig({ joints, meshes, ghost }: { joints: number[]; meshes: ArmMeshes; ghost?: boolean }) {
  const j = (i: number) => joints[12 + i] ?? 0;
  const R = (x: number, y: number, z: number) => [x, y, z, 'YXZ'] as const;
  return (
    <group position={[0.238, 0.0605, 0]}>
      <MeshPart obj={meshes.base} rot={[0, 0, 0]} ghost={ghost} />
      <group position={[0, 0.0968, 0]} rotation={R(0, j(0), 0)}>
        <MeshPart obj={meshes.link1} rot={[0, 0, 0]} ghost={ghost} />
        <group rotation={R(0, 0, -j(1))}>
          <MeshPart obj={meshes.link2} rot={[0, 0, 0]} ghost={ghost} />
          <group position={[0, 0.3324, -0.0098]} rotation={R(0, 0, -j(2))}>
            <MeshPart obj={meshes.link3} rot={[0, 0, 0]} ghost={ghost} />
            <group position={[0, 0, 0.0948]} rotation={R(0, j(3), 0)}>
              <MeshPart obj={meshes.link4} rot={[0, 0, 0]} ghost={ghost} />
              <group position={[0, 0.296, -0.085]} rotation={R(0, 0, -j(4))}>
                <MeshPart obj={meshes.link5} rot={[0, 0, 0]} ghost={ghost} />
                <group rotation={R(0, j(5), 0)}>
                  <MeshPart obj={meshes.link6} rot={[0, 0, 0]} ghost={ghost} />
                </group>
              </group>
            </group>
          </group>
        </group>
      </group>
    </group>
  );
}

function RobotRig({
  joints,
  rpy,
  worldPos,
  terrain,
  orbit,
  ghostJoints,
  armMeshes,
  wheelMeshes,
  lidarOn,
  elevationOn,
  heightmap,
  hmLayers,
  gridVisible = true,
  gridSize = 24,
  gridDivisions = 48,
  gridColor = '#4d9cf5',
  gridCenterColor = '#2a3646',
  meshGroupRef,
  miniCam,
  viewMode,
  groundShadow = false,
  fpv,
}: {
  fpv?: 'front' | 'back' | 'stairs';
  joints: number[];
  rpy: [number, number, number];
  viewMode?: number;
  meshGroupRef?: MutableRefObject<Group | null>;
  miniCam?: import('three').Camera;
  terrain?: ReactNode;
  worldPos?: [number, number, number];
  orbit: MutableRefObject<Orbit>;
  ghostJoints?: number[];
  armMeshes?: ArmMeshes | null;
  wheelMeshes?: WheelMeshes | null;
  lidarOn?: boolean;
  elevationOn?: boolean;
  heightmap?: Partial<Record<HmChannel, HmFrame>>;
  hmLayers?: HmLayers;
  gridVisible?: boolean;
  gridSize?: number;
  gridDivisions?: number;
  gridColor?: string;
  gridCenterColor?: string;
  groundShadow?: boolean;
}) {
  const meshes = useRobotMeshes();
  const tex = useRobotTextures();
  const pitchG = useRef<Group>(null);
  const yawG = useRef<Group>(null);
  const rpyG = useRef<Group>(null);
  const meshGroupInternal = useRef<Group>(null);
  const meshGroupG = meshGroupRef ?? meshGroupInternal;
  const worldG = useRef<Group>(null);
  const trunkPts = useMemo(() => {
    if (!meshes) return null;
    const b = new Box3().setFromObject(meshes.trunk);
    const pts: number[][] = [];
    for (const x of [b.min.x, b.max.x]) for (const y of [b.min.y, b.max.y]) for (const z of [b.min.z, b.max.z]) pts.push([x, z, -y]);
    return pts;
  }, [meshes]);
  useFrame(() => {
    meshGroupG.current?.traverse((obj) => obj.layers.enable(1));
    const w = !orbit.current.local || terrain != null;
    const fp = !!fpv;
    if (!fp && (viewMode === 0 || viewMode === 1)) orbit.current.yaw = CHASE_YAW_BASE - rpy[2];
    if (pitchG.current) pitchG.current.rotation.x = fp ? 0 : orbit.current.pitch;
    if (yawG.current) yawG.current.rotation.y = fp ? 0 : orbit.current.yaw;
    if (rpyG.current) {
      rpyG.current.rotation.set(w ? rpy[0] : 0, w ? rpy[2] : 0, w ? -rpy[1] : 0, 'YXZ');
    }
    if (worldG.current) {
      const x = w ? -sane(worldPos?.[0]) : 0;
      const z = w ? sane(worldPos?.[1]) : 0;
      const simZ = terrain != null ? useTelemetry.getState().robot?.worldPos?.[2] : undefined;
      const frameZ = terrain == null && elevationOn ? useElevationMap.getState().frame?.robotZ : undefined;
      let y: number;
      if (terrain != null) y = simZ != null ? -simZ : worldG.current.position.y;
      else if (frameZ != null) y = -frameZ;
      else {
        const rot = w ? _rot.set(rpy[0], rpy[2], -rpy[1], 'YXZ') : null;
        const wheeled = !!wheelMeshes;
        footCenters(joints, _feet, wheeled);
        y = lowestY(_feet, rot) - (wheeled ? OFFSETS.wheelRadius : OFFSETS.footRadius);
        if (trunkPts) y = Math.min(y, lowestY(trunkPts, rot));
      }
      worldG.current.position.set(x, isFinite(y) ? y : worldG.current.position.y, z);
    }
  });
  if (!meshes) return null;
  return (
    <group ref={pitchG}>
      <group ref={yawG}>
        {groundShadow && (
          <directionalLight castShadow position={[1.6, 3.2, 1.2]} intensity={0.9}
            shadow-mapSize-width={512} shadow-mapSize-height={512} shadow-bias={-0.0005}
            shadow-camera-left={-1.2} shadow-camera-right={1.2} shadow-camera-top={1.2} shadow-camera-bottom={-1.2}
            shadow-camera-near={0.5} shadow-camera-far={8} />
        )}
        <group ref={worldG} position={[0, -0.42, 0]}>
          {gridVisible && (
            <Grid
              args={[gridSize, gridSize]}
              cellSize={gridSize / gridDivisions}
              cellThickness={0.6}
              cellColor={gridColor}
              sectionSize={(gridSize / gridDivisions) * 8}
              sectionThickness={1}
              sectionColor={gridCenterColor}
              fadeDistance={gridSize * 1.4}
              fadeStrength={1}
              infiniteGrid={false}
              side={DoubleSide}
            />
          )}
          {terrain}
          {elevationOn && <ElevationOverlay />}
          {groundShadow && (
            <mesh rotation-x={-Math.PI / 2} position={[0, 0.001, 0]} receiveShadow>
              <planeGeometry args={[6, 6]} />
              <shadowMaterial transparent opacity={0.16} />
            </mesh>
          )}
        </group>
        <group ref={rpyG} name="robot-body">
          <group ref={meshGroupG}>
            <MeshPart obj={meshes.trunk} rot={[d2r(-90), 0, 0]} texture={tex?.trunk} />
            {armMeshes && <ArmRig joints={joints} meshes={armMeshes} />}
            {LEGS.map((def) => (
              <Leg key={def.name} def={def} joints={joints} meshes={meshes} tex={tex} wheel={wheelMeshes} />
            ))}
          </group>
          {miniCam && (
            <primitive object={miniCam} position={[0, 1.2, 0]} rotation={[-Math.PI / 2, 0, HEADING_ROT_Z]} />
          )}
          {lidarOn && <LidarClouds />}
          {heightmap && hmLayers && <HeightmapClouds frames={heightmap} layers={hmLayers} />}
          {ghostJoints && LEGS.map((def) => (
            <Leg key={`g-${def.name}`} def={def} joints={ghostJoints} meshes={meshes} tex={null} ghost wheel={wheelMeshes} />
          ))}
        </group>
      </group>
    </group>
  );
}

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

export function FirstFrameSignal({ onDrawn }: { onDrawn: () => void }) {
  const n = useRef(0);
  useFrame(() => {
    n.current += 1;
    if (n.current === 3) onDrawn();
  });
  return null;
}

function IdleSpin({ orbit, dragging, releasedAt }: {
  orbit: MutableRefObject<Orbit>; dragging: MutableRefObject<boolean>; releasedAt: MutableRefObject<number>;
}) {
  const t = useRef(0);
  useFrame((_, dt) => {
    if (dragging.current || Date.now() - releasedAt.current < 2000) return;
    const d = Math.min(dt, 0.1);
    t.current += d;
    orbit.current.yaw += d * 0.105;
    orbit.current.pitch += Math.sin(t.current * 0.35) * d * 0.02;
  });
  return null;
}

const SIM_CAMS: [string, [number, number, number], [number, number, number, number], number, 0 | 1 | -1][] = [
  ['FT0', [0.388031, -0.0375, 0.037764], [0.5, 0.5, -0.5, -0.5], 63.7, 0],
  ['RR0', [-0.388031, 0.0375, 0.037764], [-0.5, -0.5, -0.5, -0.5], 63.7, 0],
  ['BT0', [0.364, 0, -0.024919], [0, -0.1736482, 0, 0.9848078], 90.6, 1],
  ['BT1', [0.26097, 0, -0.04582], [0, 0.1218693, 0, 0.9925462], 90.6, 1],
  ['BT2', [-0.19515, 0.0065, -0.0465], [0, 0, 0, 1], 90.6, 1],
  ['BT3', [-0.352082, -0.000011, -0.018938], [0.9848078, 0, 0.1736482, 0], 90.6, -1],
];
const _sq = new Quaternion(), _sp = new Vector3(), _sd = new Vector3(), _su = new Vector3(), _sr = new Vector3();
const mj2scene = (v: Vector3) => v.set(v.x, v.z, -v.y);

function StairsComposite({ orbit, dist }: {
  orbit: MutableRefObject<{ yaw: number; pitch: number; local: boolean }>;
  dist: MutableRefObject<number>;
}) {
  const cams = useMemo(() => ({
    list: SIM_CAMS.map(([, , , fov]) => new PerspectiveCamera(fov, 1, 0.02, 200)),
    pose: new PerspectiveCamera(35, 1, 0.05, 50),
  }), []);
  useFrame(({ gl, scene, size }) => {
    const body = scene.getObjectByName('robot-body');
    if (!body) return;
    body.updateWorldMatrix(true, false);
    SIM_CAMS.forEach(([, pos, q, , rot], i) => {
      const cam = cams.list[i];
      _sq.set(q[1], q[2], q[3], q[0]);
      _sp.set(pos[0], pos[1], pos[2]); body.localToWorld(mj2scene(_sp));
      _sd.set(0, 0, -1).applyQuaternion(_sq); mj2scene(_sd).transformDirection(body.matrixWorld);
      _su.set(0, 1, 0).applyQuaternion(_sq); mj2scene(_su).transformDirection(body.matrixWorld);
      _sr.set(1, 0, 0).applyQuaternion(_sq); mj2scene(_sr).transformDirection(body.matrixWorld);
      if (rot === 1) cam.up.copy(_sr).negate(); else if (rot === -1) cam.up.copy(_sr); else cam.up.copy(_su);
      cam.position.copy(_sp); cam.lookAt(_sp.x + _sd.x, _sp.y + _sd.y, _sp.z + _sd.z);
    });
    const W = size.width, H = size.height, wm = W / 2, hm = H / 2, ws = H / 4, hs = ws * 9 / 16;
    const draw = (cam: PerspectiveCamera, x: number, y: number, w: number, h: number) => {
      if (w < 2 || h < 2) return;
      const yy = H - y - h;
      gl.setViewport(x, yy, w, h); gl.setScissor(x, yy, w, h);
      cam.aspect = w / h; cam.updateProjectionMatrix();
      gl.clear(); gl.render(scene, cam);
    };
    gl.autoClear = false; gl.setScissorTest(true);
    draw(cams.list[0], 0, 0, wm, hm);
    draw(cams.list[1], 0, hm, wm, H - hm);
    for (let k = 0; k < 4; k++) draw(cams.list[2 + k], wm, k * ws, hs, ws);
    const r = useTelemetry.getState().robot;
    const yaw = r?.worldRpy?.[2] ?? r?.imu?.rpy?.[2] ?? 0;
    const az = yaw + Math.PI / 2 + orbit.current.yaw, el = orbit.current.pitch, R = dist.current * 1.6;
    const terr = scene.getObjectByName('sim-terrain');
    if (terr) terr.visible = false;
    cams.pose.position.set(R * Math.cos(el) * Math.cos(az), -0.05 + R * Math.sin(el), -R * Math.cos(el) * Math.sin(az));
    cams.pose.lookAt(0, -0.05, 0);
    draw(cams.pose, wm + hs, 0, W - wm - hs, H);
    if (terr) terr.visible = true;
    gl.setScissorTest(false); gl.setViewport(0, 0, W, H); gl.autoClear = true;
  }, 1);
  return null;
}

function CameraRig({ dist, fpv }: { dist: MutableRefObject<number>; fpv?: 'front' | 'back' | 'stairs' }) {
  useFrame(({ camera }) => {
    if (fpv === 'front' || fpv === 'back') {
      const r = useTelemetry.getState().robot;
      const yaw = r?.worldRpy?.[2] ?? r?.imu?.rpy?.[2] ?? 0;
      const fx = Math.cos(yaw), fz = -Math.sin(yaw);
      const s = fpv === 'front' ? 1 : -1;
      const px = fx * 0.55 * s, pz = fz * 0.55 * s, py = 0.3;
      camera.position.set(px, py, pz);
      camera.lookAt(px + fx * 3 * s, py - 0.45, pz + fz * 3 * s);
      return;
    }
    camera.position.set(0, 0.06, dist.current);
    camera.lookAt(0, 0.02, 0);
  });
  return null;
}

const HEADING_ROT_Z = -Math.PI / 2;
function TopDownMinimap({
  splitX, miniCam,
}: {
  splitX: number;
  miniCam: ThreeOrthographicCamera;
}) {
  const { gl, scene, camera, size } = useThree();

  useFrame(() => {
    const dpr = gl.getPixelRatio();
    const w = Math.round(size.width * dpr), h = Math.round(size.height * dpr);
    const leftW = Math.round(w * splitX);
    const rightW = w - leftW;

    const persp = camera as unknown as { aspect?: number; updateProjectionMatrix?: () => void };
    if (typeof persp.aspect === 'number' && h > 0) {
      persp.aspect = leftW / h;
      persp.updateProjectionMatrix?.();
    }

    const HALF_EXTENT_M = 2.7;
    const rightAspect = h > 0 ? rightW / h : 1;
    if (rightAspect >= 1) {
      miniCam.top = HALF_EXTENT_M; miniCam.bottom = -HALF_EXTENT_M;
      miniCam.left = -HALF_EXTENT_M * rightAspect; miniCam.right = HALF_EXTENT_M * rightAspect;
    } else {
      miniCam.left = -HALF_EXTENT_M; miniCam.right = HALF_EXTENT_M;
      miniCam.top = HALF_EXTENT_M / rightAspect; miniCam.bottom = -HALF_EXTENT_M / rightAspect;
    }
    miniCam.updateProjectionMatrix();

    gl.setScissorTest(true);
    gl.setViewport(0, 0, leftW, h);
    gl.setScissor(0, 0, leftW, h);
    gl.render(scene, camera);

    const prevBg = scene.background;
    const prevAlpha = gl.getClearAlpha();
    const prevColor = gl.getClearColor(new Color());
    scene.background = null;
    gl.setClearColor(0x000000, 0);
    gl.setViewport(leftW, 0, rightW, h);
    gl.setScissor(leftW, 0, rightW, h);
    gl.render(scene, miniCam);
    scene.background = prevBg;
    gl.setClearColor(prevColor, prevAlpha);
    gl.setScissorTest(false);
    gl.setViewport(0, 0, w, h);
  }, 1);

  useEffect(() => () => {
    const dpr = gl.getPixelRatio();
    const w = Math.round(size.width * dpr), h = Math.round(size.height * dpr);
    if (w > 0 && h > 0) gl.setViewport(0, 0, w, h);
    gl.setScissorTest(false);
    const persp = camera as unknown as { aspect?: number; updateProjectionMatrix?: () => void };
    if (typeof persp.aspect === 'number' && size.height > 0) {
      persp.aspect = size.width / size.height;
      persp.updateProjectionMatrix?.();
    }
  }, [camera, gl, size]);
  return null;
}
const FIXED_RPY: [number, number, number] = [0, 0, 0];

export function RobotModel3D({
  active = true, controls = true, listenPresets = true, pose, ghostJoints, onTap,
  gridVisible = true, gridSize = 24, gridDivisions = 48,
  gridColor = '#4d9cf5', gridCenterColor = '#2a3646', backgroundColor,
  viewMode, lidarMode, showPresetRow, terrain, elevationOn, topDownMini, heightmap, hmLayers,
  idleSpin = false, initialOrbit, initialDist, groundShadow = false, fpv, bodyFixed = false,
}: {
  bodyFixed?: boolean;
  fpv?: 'front' | 'back' | 'stairs';
  active?: boolean;
  controls?: boolean;
  listenPresets?: boolean;
  pose?: { joints: number[]; rpy: [number, number, number] };
  terrain?: ReactNode;
  ghostJoints?: number[];
  onTap?: () => void;
  gridVisible?: boolean; gridSize?: number; gridDivisions?: number;
  gridColor?: string; gridCenterColor?: string;
  backgroundColor?: string;
  viewMode?: number;
  lidarMode?: LidarMode;
  showPresetRow?: boolean;
  elevationOn?: boolean;
  topDownMini?: boolean;
  heightmap?: Partial<Record<HmChannel, HmFrame>>;
  hmLayers?: HmLayers;
  idleSpin?: boolean;
  initialOrbit?: { yaw: number; pitch: number };
  initialDist?: number;
  groundShadow?: boolean;
} = {}) {
  const steamos = useSteamOS();
  const meshesReady = !!useRobotMeshes();
  const [drawn, setDrawn] = useState(false);
  const meshGroupRef = useRef<Group>(null);
  const miniCam = useMemo(() => {
    const cam = new ThreeOrthographicCamera(-1, 1, 1, -1, 0.01, 10);
    cam.layers.set(1);
    return cam;
  }, []);

  const [glReady, setGlReady] = useState(false);
  const [glEpoch, setGlEpoch] = useState(0);
  useEffect(() => {
    if (Platform.OS !== 'ios') return;
    let sawBackground = false;
    const sub = AppState.addEventListener('change', (st) => {
      if (st === 'background') { sawBackground = true; return; }
      if (st !== 'active' || !sawBackground) return;
      sawBackground = false;
      setGlEpoch((e) => e + 1);
    });
    return () => sub.remove();
  }, []);
  useEffect(() => {
    let alive = true;
    const delay = new Promise((r) => setTimeout(r, 150));
    Promise.all([delay, getPlatformInfo().catch(() => null)]).then(() => { if (alive) setGlReady(true); });
    return () => { alive = false; };
  }, []);

  const [glError, setGlError] = useState<string | null>(null);

  const meshError = useRobotMeshError();

  const tel = useTelemetry((s) => (pose ? undefined : s.robot));
  const joints = pose ? pose.joints : tel?.joints?.length ? tel.joints.map((j) => j.position) : STANDING_RAD;
  const rpy = pose ? pose.rpy : bodyFixed ? FIXED_RPY
    : ([tel?.imu?.rpy?.[0] ?? 0, tel?.imu?.rpy?.[1] ?? 0, tel?.worldRpy?.[2] ?? 0] as [number, number, number]);
  const worldPos = pose || bodyFixed ? undefined : (tel?.worldPos as [number, number, number] | undefined);
  const hasArm = useHasArm();
  const armMeshes = useArmMeshes(!pose && hasArm);
  const featureWheel = useFeatureWheel();
  const wheelMeshes = useWheelMeshes(featureWheel);
  const [lidarOn, setLidarOn] = useState(false);
  const hasCloud = usePointCloud((s) => !pose && !!s.lastAt);
  const lidarControlled = lidarMode !== undefined;
  const lidarActive = (lidarControlled ? lidarMode !== 'off' : lidarOn) && hasCloud;
  const liveHm = useHeightmapCloud((s) => (pose || heightmap ? undefined : s.clouds));
  const hmGrid = useView3d((s) => s.hmGrid), hmStair = useView3d((s) => s.hmStair);
  const hmEdge = useView3d((s) => s.hmEdge), hmFoot = useView3d((s) => s.hmFoot);
  const hmFrames = heightmap ?? liveHm;
  const hmGaitOn = useTelemetry((s) => (pose || heightmap ? true : isHeightmapGait(s.robot?.gaitId)));
  const hmLayersEff: HmLayers = hmLayers ?? (hmGaitOn
    ? { grid: hmGrid, stair: hmStair, edge: hmEdge, foot: hmFoot }
    : HM_LAYERS_OFF);

  const wantsCloud = !pose && !heightmap
    && ((lidarControlled ? lidarMode !== 'off' : lidarOn) || hmGrid || hmStair || hmEdge || hmFoot);
  const visionIp = useRobot((s) => resolveEndpoints(s.ip, s.visionIp).vision);
  useEffect(() => {
    if (wantsCloud && visionIp) webrtcClient.ensureConnected(visionIp);
  }, [wantsCloud, visionIp]);

  const orbit = useRef<Orbit>({ yaw: initialOrbit?.yaw ?? -0.7, pitch: initialOrbit?.pitch ?? 0.35, local: false });
  const releasedAt = useRef(0);
  const dragging = useRef(false);
  useEffect(() => {
    if (!listenPresets) return;
    return onOrbitPreset((p) => { orbit.current.yaw = p.yaw; orbit.current.pitch = p.pitch; orbit.current.local = p.local; });
  }, [listenPresets]);
  const camDist = useRef(initialDist ?? 1.5);
  useEffect(() => { if (initialDist != null) camDist.current = initialDist; }, [initialDist]);
  useEffect(() => { if (initialOrbit) { orbit.current.yaw = initialOrbit.yaw; orbit.current.pitch = initialOrbit.pitch; } }, [initialOrbit?.yaw, initialOrbit?.pitch]); // eslint-disable-line react-hooks/exhaustive-deps
  const wrapRef = useRef<View>(null);
  useEffect(() => {
    const el = wrapRef.current as unknown as HTMLElement | null;
    if (!el || typeof el.addEventListener !== 'function') return;
    const onWheel = (e: WheelEvent) => {
      camDist.current = Math.min(6, Math.max(0.6, camDist.current * (1 + e.deltaY * 0.0012)));
      e.preventDefault();
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);
  useEffect(() => {
    if (viewMode === undefined) return;
    if (viewMode === 0) { orbit.current = { yaw: CHASE_YAW_BASE, pitch: 1.15, local: false }; camDist.current = 3.2; }
    else if (viewMode === 1) { orbit.current = { yaw: CHASE_YAW_BASE, pitch: 0.05, local: false }; camDist.current = 0.7; }
    else { orbit.current = { yaw: -0.7, pitch: 0.35, local: false }; camDist.current = 1.5; }
  }, [viewMode]);
  const last = useRef({ x: 0, y: 0 });
  const pinchLast = useRef<number | null>(null);
  const onTapRef = useRef(onTap);
  onTapRef.current = onTap;
  const responder = useMemo(
    () =>
      PanResponder.create({
        onStartShouldSetPanResponder: () => true,
        onMoveShouldSetPanResponder: () => true,
        onPanResponderGrant: () => {
          last.current = { x: 0, y: 0 };
          pinchLast.current = null;
          dragging.current = true;
        },
        onPanResponderMove: (e, g) => {
          const t = e.nativeEvent.touches;
          if (t.length >= 2) {
            const d = Math.hypot(t[0].pageX - t[1].pageX, t[0].pageY - t[1].pageY);
            if (pinchLast.current != null && d > 0) {
              camDist.current = clamp(camDist.current * (pinchLast.current / d), 0.6, 6);
            }
            pinchLast.current = d;
            last.current = { x: g.dx, y: g.dy };
            return;
          }
          pinchLast.current = null;
          const ddx = g.dx - last.current.x;
          const ddy = g.dy - last.current.y;
          last.current = { x: g.dx, y: g.dy };
          orbit.current.yaw += ddx * 0.01;
          orbit.current.pitch = clamp(orbit.current.pitch + ddy * 0.01, -1.2, 1.2);
        },
        onPanResponderRelease: (e, g) => {
          pinchLast.current = null;
          dragging.current = false;
          releasedAt.current = Date.now();
          if (Math.abs(g.dx) < 6 && Math.abs(g.dy) < 6) onTapRef.current?.();
        },
      }),
    [],
  );

  return (
    <View ref={wrapRef} style={StyleSheet.absoluteFill}>
      {(glError || meshError) && (
        <View style={[StyleSheet.absoluteFill, { alignItems: 'center', justifyContent: 'center', padding: 24 }]}>
          <Text style={{ color: '#c96', fontSize: 12, textAlign: 'center', lineHeight: 18 }}>{glError ?? meshError}</Text>
        </View>
      )}
      {!drawn && !glError && !meshError && (
        <View style={[StyleSheet.absoluteFill, { alignItems: 'center', justifyContent: 'center' }]} pointerEvents="none">
          <ActivityIndicator size="small" color="#8a94a6" />
          <Text style={{ color: '#8a94a6', fontSize: 11, marginTop: 8 }}>{t('3D 모델 로딩 중…')}</Text>
        </View>
      )}
      {glReady && !glError && meshesReady && (
      <GlBoundary onError={setGlError}>
      <Canvas
        key={glEpoch}
        frameloop={active ? 'always' : 'never'}
        shadows={groundShadow ? 'soft' : false}
        gl={steamos ? { antialias: true, powerPreference: 'high-performance' } : undefined}
        camera={{ position: [0, 0.06, 1.5], fov: 50 }}
        // @ts-expect-error
        dpr={steamos ? 1 : [1, 2]}
        onCreated={({ camera, gl }) => {
          camera.lookAt(0, 0.02, 0);
          gl.debug.checkShaderErrors = false;
          (gl.domElement as HTMLCanvasElement | undefined)?.addEventListener?.('webglcontextlost',
            () => setGlError('3D 중단 — WebGL 컨텍스트 소실(GPU 드라이버/절전). 화면을 닫았다 다시 열면 복구 시도'), { once: true });
        }}
      >
        {backgroundColor && <color attach="background" args={[backgroundColor]} />}
        <ambientLight intensity={0.7} ref={(l) => l?.layers.enable(1)} />
        <directionalLight position={[5, 5, 5]} intensity={1} ref={(l) => l?.layers.enable(1)} />
        <directionalLight position={[-5, 2, -3]} intensity={0.4} ref={(l) => l?.layers.enable(1)} />
        <CameraRig dist={camDist} fpv={fpv} />
        {fpv === 'stairs' && <StairsComposite orbit={orbit} dist={camDist} />}
        {idleSpin && <IdleSpin orbit={orbit} dragging={dragging} releasedAt={releasedAt} />}
        <FirstFrameSignal onDrawn={() => setDrawn(true)} />
        <group position={[0, 0.12, 0]}>
          <RobotRig joints={joints} rpy={rpy} worldPos={worldPos} orbit={orbit} ghostJoints={ghostJoints} armMeshes={armMeshes} fpv={fpv}
            terrain={terrain}
            wheelMeshes={wheelMeshes}
            lidarOn={lidarActive}
            elevationOn={elevationOn}
            heightmap={hmFrames} hmLayers={hmLayersEff}
            gridVisible={gridVisible} gridSize={gridSize} gridDivisions={gridDivisions}
            gridColor={gridColor} gridCenterColor={gridCenterColor} groundShadow={groundShadow}
            meshGroupRef={meshGroupRef} miniCam={topDownMini ? miniCam : undefined} viewMode={viewMode} />
        </group>
        {topDownMini && <TopDownMinimap splitX={OBSMAP_3D_W} miniCam={miniCam} />}
      </Canvas>
      </GlBoundary>
      )}
      <View style={[StyleSheet.absoluteFill, { touchAction: 'none', userSelect: 'none' } as any]} {...responder.panHandlers} />
      {(showPresetRow ?? controls) && (
      <View style={presetStyles.row} pointerEvents="box-none">
        {hasCloud && !lidarControlled && (
          <Pressable onPress={() => setLidarOn((v) => !v)}
            style={[presetStyles.btn, lidarOn && { backgroundColor: 'rgba(77,156,245,0.55)' }]}>
            <Text style={{ color: '#fff', fontSize: 11, fontWeight: '600' }}>LiDAR</Text>
          </Pressable>
        )}
      </View>
      )}
    </View>
  );
}

export class GlBoundary extends Component<{ onError: (msg: string) => void; children: ReactNode }, { failed: boolean }> {
  state = { failed: false };
  static getDerivedStateFromError() { return { failed: true }; }
  componentDidCatch(e: unknown) { this.props.onError('3D 사용 불가 — WebGL 초기화 실패: ' + String((e as Error)?.message ?? e)); }
  render() { return this.state.failed ? null : this.props.children; }
}


const presetStyles = StyleSheet.create({
  row: { position: 'absolute', bottom: 6, left: 0, right: 0, flexDirection: 'row', justifyContent: 'center', gap: 5 },
  btn: { backgroundColor: 'rgba(13,17,23,0.72)', paddingHorizontal: 9, paddingVertical: 5, borderRadius: 6 },
});

