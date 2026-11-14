import { useEffect, useMemo, useRef, useState, type MutableRefObject } from 'react';
import { View, Text, StyleSheet, PanResponder, AppState, Platform } from 'react-native';
import {
  BufferAttribute, BufferGeometry, Color, Euler, Line, LineBasicMaterial, LineSegments, Plane, Raycaster, Vector2, Vector3,
  type Camera, type Group,
} from 'three';
import { Canvas, useFrame, useThree } from '@/lib/r3fCanvas';
import { useSteamOS, getPlatformInfo } from '@/lib/platformInfo';
import { GlBoundary } from '@/components/RobotModel3D';
import { useTheme } from '@/theme';
import { useDeviceToggles } from '@/store/deviceToggles';
import { useSlamnav, useSlamnavSession, type TopoNode } from '@/lib/slamnav';
import { heightColors, pickNearest, SCAN_STOPS, type CloudSet } from '@/lib/slamnavWire';
import { editReady, setSlamTool, slamTapMap, slamTapNode, tapGroundZ, useSlamEdit } from '@/lib/slamEdit';
import { SlamEditBar, useSlamEditAllowed } from '@/components/control/SlamEditBar';
import { t } from '@/lib/i18n';

type Cam = { yaw: number; pitch: number; dist: number; tx: number; ty: number; tz: number; init: boolean };

const FOV = 50;
const DIST_MIN = 2, DIST_MAX = 400;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));
const TAP_PX = 8;
const AIM_HOLD_MS = 250;
const PICK_PX = 24;

const NODE_COLOR: Record<string, string> = { ROUTE: '#7aa7d8', GOAL: '#34c759', BASE: '#ffd60a', STAIR: '#bf5af2', ZONE: '#64d2ff', OBS: '#ff453a' };

const MAP_ROT: [number, number, number] = [-Math.PI / 2, 0, 0];

function usePointsGeom(positions: Float32Array | undefined, colors: Float32Array | null | undefined) {
  const geom = useMemo(() => {
    if (!positions) return null;
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(positions, 3));
    if (colors) g.setAttribute('color', new BufferAttribute(colors, 3));
    return g;
  }, [positions, colors]);
  useEffect(() => () => geom?.dispose(), [geom]);
  return geom;
}

function Points({ positions, colors, color, size, under }: {
  positions?: Float32Array; colors?: Float32Array | null; color?: string; size: number;
  under?: boolean;
}) {
  const geom = usePointsGeom(positions, colors);
  const dpr = useThree((s) => s.viewport.dpr);
  if (!geom) return null;
  return (
    <points geometry={geom} renderOrder={under ? -1 : 0}>
      <pointsMaterial size={size * dpr} sizeAttenuation={false} vertexColors={!!colors} color={colors ? undefined : color}
        depthWrite={!under} />
    </points>
  );
}

function MapCloudLayer() {
  const map = useSlamnav((s) => s.map);
  const colors = useMemo(() => (map ? heightColors(map.positions, map.count, map.label) : null), [map]);
  return <Points positions={map?.positions} colors={colors} size={2} />;
}

function LiveLayers() {
  const live = useSlamnav((s) => s.live);
  const scan = useSlamnav((s) => s.scan);
  const obs = useSlamnav((s) => s.obs);
  const liveColors = useMemo(() => (live ? heightColors(live.positions, live.count) : null), [live]);
  const scanColors = useMemo(() => (scan ? heightColors(scan.positions, scan.count, undefined, SCAN_STOPS) : null), [scan]);
  const obsColors = useMemo(() => (obs ? obsColorsOf(obs) : null), [obs]);
  return (
    <>
      <Points positions={live?.positions} colors={liveColors} size={2} />
      <Points positions={scan?.positions} colors={scanColors} size={2.5} />
      <Points positions={obs?.positions} colors={obsColors} size={4} />
    </>
  );
}

function obsColorsOf(c: CloudSet): Float32Array {
  const out = new Float32Array(c.count * 3);
  for (let i = 0; i < c.count; i++) {
    const p = c.values ? c.values[i] / 255 : 1;
    out[i * 3] = 1; out[i * 3 + 1] = 0.45 * (1 - p); out[i * 3 + 2] = 0.1;
  }
  return out;
}

function TopoLayer() {
  const topo = useSlamnav((s) => s.topo);
  const { pos, col, links } = useMemo(() => {
    const nodes = topo ?? [];
    const byId = new Map<string, TopoNode>(nodes.map((n) => [n.id, n]));
    const p = new Float32Array(nodes.length * 3), cl = new Float32Array(nodes.length * 3);
    const tmp = new Color();
    nodes.forEach((n, i) => {
      p.set(n.pos, i * 3);
      tmp.set(NODE_COLOR[n.type] ?? '#ff9f0a');
      cl[i * 3] = tmp.r; cl[i * 3 + 1] = tmp.g; cl[i * 3 + 2] = tmp.b;
    });
    const l: number[] = [];
    for (const n of nodes) for (const id of n.linked) { const m = byId.get(id); if (m) l.push(...n.pos, ...m.pos); }
    return { pos: nodes.length ? p : undefined, col: cl, links: l.length ? new Float32Array(l) : undefined };
  }, [topo]);
  const linkGeom = usePointsGeom(links, null);
  return (
    <>
      {linkGeom && (
        <lineSegments geometry={linkGeom}>
          <lineBasicMaterial color="#7aa7d8" transparent opacity={0.6} />
        </lineSegments>
      )}
      <Points positions={pos} colors={col} size={9} />
    </>
  );
}

function HighlightLayer() {
  const topo = useSlamnav((s) => s.topo);
  const goal = useSlamnav((s) => s.state?.ctrl.goal);
  const { selected, prev, moving } = useSlamEdit();
  const { pos, col } = useMemo(() => {
    const marks: [string | undefined, string][] = [[goal, '#34c759'], [prev, '#8e8e93'], [selected, '#ff375f'], [moving, '#ff9f0a']];
    const p: number[] = [], cl: number[] = [];
    const tmp = new Color();
    for (const [id, color] of marks) {
      const n = id ? topo?.find((m) => m.id === id) : undefined;
      if (!n) continue;
      tmp.set(color);
      p.push(...n.pos); cl.push(tmp.r, tmp.g, tmp.b);
    }
    return { pos: p.length ? new Float32Array(p) : undefined, col: new Float32Array(cl) };
  }, [topo, goal, selected, prev, moving]);
  return <Points positions={pos} colors={col} size={17} under />;
}

type Aim = { on: boolean; a: [number, number]; b: [number, number]; z: number };
function AimArrow({ aim }: { aim: MutableRefObject<Aim> }) {
  const seg = useMemo(() => {
    const g = new BufferGeometry();
    g.setAttribute('position', new BufferAttribute(new Float32Array(18), 3));
    return new LineSegments(g, new LineBasicMaterial({ color: '#ffd60a' }));
  }, []);
  useEffect(() => () => { seg.geometry.dispose(); (seg.material as LineBasicMaterial).dispose(); }, [seg]);
  useFrame(() => {
    const m = aim.current;
    seg.visible = m.on;
    if (!m.on) return;
    const [ax, ay] = m.a, [bx, by] = m.b, z = m.z + 0.05;
    const yaw = Math.atan2(by - ay, bx - ax), h = 0.35;
    const l = [yaw + 2.6, yaw - 2.6].map((r) => [bx + h * Math.cos(r), by + h * Math.sin(r)]);
    const arr = seg.geometry.getAttribute('position') as BufferAttribute;
    (arr.array as Float32Array).set([ax, ay, z, bx, by, z, bx, by, z, l[0][0], l[0][1], z, bx, by, z, l[1][0], l[1][1], z]);
    arr.needsUpdate = true;
  });
  return <primitive object={seg} />;
}

function CamGrab({ out }: { out: MutableRefObject<Camera | null> }) {
  out.current = useThree((s) => s.camera);
  return null;
}

const _ray = new Raycaster(), _ndc = new Vector2(), _plane = new Plane(new Vector3(0, 1, 0), 0), _hit = new Vector3(), _pv = new Vector3();
function screenToMap(cam: Camera, w: number, h: number, sx: number, sy: number, gz: number): [number, number] | null {
  _ndc.set((sx / w) * 2 - 1, -(sy / h) * 2 + 1);
  _ray.setFromCamera(_ndc, cam);
  _plane.constant = -gz;
  return _ray.ray.intersectPlane(_plane, _hit) ? [_hit.x, -_hit.z] : null;
}
function nodesOnScreen(cam: Camera, w: number, h: number, nodes: TopoNode[]) {
  const out: { id: string; x: number; y: number }[] = [];
  for (const n of nodes) {
    _pv.set(n.pos[0], n.pos[2], -n.pos[1]).project(cam);
    if (_pv.z < 1) out.push({ id: n.id, x: ((_pv.x + 1) / 2) * w, y: ((1 - _pv.y) / 2) * h });
  }
  return out;
}

function Polyline({ points, color }: { points?: Float32Array; color: string }) {
  const geom = usePointsGeom(points, null);
  const line = useMemo(() => (geom ? new Line(geom, new LineBasicMaterial({ color })) : null), [geom, color]);
  useEffect(() => () => { (line?.material as LineBasicMaterial | undefined)?.dispose(); }, [line]);
  return line ? <primitive object={line} /> : null;
}

function PathLayers() {
  const path = useSlamnav((s) => s.path);
  const local = useSlamnav((s) => s.state?.ctrl.localPath);
  const g = useMemo(() => (path?.points.length ? new Float32Array(path.points.flat()) : undefined), [path]);
  const localKey = local?.map((q) => `${q[0].toFixed(2)},${q[1].toFixed(2)}`).join(';') ?? '';
  const l = useMemo(() => (local?.length ? new Float32Array(local.flatMap((q) => [q[0], q[1], q[2]])) : undefined),
    [localKey]); // eslint-disable-line react-hooks/exhaustive-deps
  return (
    <>
      <Polyline points={g} color="#4d9cf5" />
      <Polyline points={l} color="#ffd60a" />
    </>
  );
}

const _e = new Euler();
function RobotBox() {
  const size = useSlamnav((s) => s.state?.robot.size);
  const has = useSlamnav((s) => !!s.state);
  const ref = useRef<Group>(null);
  useFrame(() => {
    const p = useSlamnav.getState().state?.pose;
    const g = ref.current;
    if (!p || !g) return;
    g.position.set(p.x, p.y, p.z);
    g.rotation.copy(_e.set(p.roll, p.pitch, p.yaw, 'ZYX'));
  });
  if (!has || !size) return null;
  const sx = size.x[1] - size.x[0], sy = size.y[1] - size.y[0], sz = size.z[1] - size.z[0];
  const c: [number, number, number] = [(size.x[0] + size.x[1]) / 2, (size.y[0] + size.y[1]) / 2, (size.z[0] + size.z[1]) / 2];
  return (
    <group ref={ref}>
      <mesh position={c}>
        <boxGeometry args={[sx, sy, sz]} />
        <meshBasicMaterial color="#4d9cf5" transparent opacity={0.45} />
      </mesh>
      <mesh position={[size.x[1] + 0.08, c[1], c[2]]}>
        <boxGeometry args={[0.16, sy * 0.4, sz * 0.4]} />
        <meshBasicMaterial color="#ffd60a" />
      </mesh>
    </group>
  );
}

const _t = new Vector3();
function CameraRig({ cam, view, follow }: { cam: MutableRefObject<Cam>; view: '2d' | '3d'; follow: boolean }) {
  useFrame(({ camera }) => {
    const st = useSlamnav.getState();
    const c = cam.current;
    if (!c.init) {
      const m = st.map;
      if (m && m.count > 0) {
        let x0 = Infinity, x1 = -Infinity, y0 = Infinity, y1 = -Infinity;
        for (let i = 0; i < m.count; i += 97) {
          const x = m.positions[i * 3], y = m.positions[i * 3 + 1];
          x0 = Math.min(x0, x); x1 = Math.max(x1, x); y0 = Math.min(y0, y); y1 = Math.max(y1, y);
        }
        c.tx = (x0 + x1) / 2; c.ty = (y0 + y1) / 2; c.tz = 0;
        c.dist = clamp(Math.max(x1 - x0, y1 - y0) * 1.1, DIST_MIN, DIST_MAX);
        c.init = true;
      } else if (st.state) {
        c.tx = st.state.pose.x; c.ty = st.state.pose.y; c.tz = st.state.pose.z;
        c.init = st.state.flags.slam || !st.state.flags.mapLoaded;
      }
    }
    if (follow && st.state) { c.tx = st.state.pose.x; c.ty = st.state.pose.y; c.tz = st.state.pose.z; }
    _t.set(c.tx, c.tz, -c.ty);
    if (view === '2d') {
      camera.up.set(0, 0, -1);
      camera.position.set(_t.x, _t.y + c.dist, _t.z + 0.0001);
    } else {
      camera.up.set(0, 1, 0);
      const h = c.dist * Math.cos(c.pitch);
      camera.position.set(_t.x + h * Math.cos(c.yaw), _t.y + c.dist * Math.sin(c.pitch), _t.z - h * Math.sin(c.yaw));
    }
    camera.lookAt(_t);
  });
  return null;
}

function useStatusLine(): string | null {
  const link = useSlamnav((s) => s.link);
  const hasState = useSlamnav((s) => !!s.state);
  const mapLoaded = useSlamnav((s) => !!s.state?.flags.mapLoaded);
  const mapping = useSlamnav((s) => !!s.state?.flags.slam);
  const hasMap = useSlamnav((s) => !!s.map);
  if (link === 'unsupported') return t('이 로봇 소프트웨어는 원격에서 SLAM 을 지원하지 않습니다 — 로봇을 업데이트하세요');
  if (link === 'off') return t('로봇 연결 전');
  if (!hasState) return link === 'retrying' ? t('SLAMNAV 응답 없음 — 다시 연결하는 중') : t('SLAMNAV 연결 중…');
  if (mapping) return t('맵핑 중');
  if (!mapLoaded) return t('불러온 맵 없음');
  if (!hasMap) return t('맵 받는 중…');
  return null;
}

export function SlamMapView({ backgroundColor }: { backgroundColor?: string }) {
  useSlamnavSession(true, true);
  const { c } = useTheme();
  const steamos = useSteamOS();
  const view = useDeviceToggles((s) => s.slamView);
  const follow = useDeviceToggles((s) => s.slamFollow);
  const status = useStatusLine();
  const cam = useRef<Cam>({ yaw: -Math.PI / 2 - 0.6, pitch: 0.9, dist: 20, tx: 0, ty: 0, tz: 0, init: false });
  const box = useRef({ w: 1, h: 1 });

  const [glReady, setGlReady] = useState(false);
  const [glError, setGlError] = useState<string | null>(null);
  useEffect(() => {
    let alive = true;
    const delay = new Promise((r) => setTimeout(r, 150));
    Promise.all([delay, getPlatformInfo().catch(() => null)]).then(() => { if (alive) setGlReady(true); });
    return () => { alive = false; };
  }, []);
  const [glEpoch, setGlEpoch] = useState(0);
  useEffect(() => {
    if (Platform.OS !== 'ios') return;
    let bg = false;
    const sub = AppState.addEventListener('change', (s) => {
      if (s === 'background') { bg = true; return; }
      if (s === 'active' && bg) { bg = false; setGlEpoch((e) => e + 1); }
    });
    return () => sub.remove();
  }, []);

  const wrapRef = useRef<View>(null);
  useEffect(() => {
    const el = wrapRef.current as unknown as HTMLElement | null;
    if (!el || typeof el.addEventListener !== 'function') return;
    const onWheel = (e: WheelEvent) => {
      cam.current.dist = clamp(cam.current.dist * (1 + e.deltaY * 0.0012), DIST_MIN, DIST_MAX);
      e.preventDefault();
    };
    el.addEventListener('wheel', onWheel, { passive: false });
    return () => el.removeEventListener('wheel', onWheel);
  }, []);

  const editAllowed = useSlamEditAllowed();
  useEffect(() => () => setSlamTool('view'), []);
  const glCam = useRef<Camera | null>(null);
  const aim = useRef<Aim>({ on: false, a: [0, 0], b: [0, 0], z: 0 });
  const press = useRef({ x: 0, y: 0, at: 0, kind: 'tap' as 'tap' | 'pan' | 'aim' | 'none' });
  const toMap = (sx: number, sy: number, gz: number) =>
    glCam.current ? screenToMap(glCam.current, box.current.w, box.current.h, sx, sy, gz) : null;
  const onTap = (sx: number, sy: number) => {
    const ed = useSlamEdit.getState();
    if (!glCam.current || (ed.tool === 'view' && !ed.moving)) return;
    if (!ed.moving) {
      const pts = nodesOnScreen(glCam.current, box.current.w, box.current.h, useSlamnav.getState().topo ?? []);
      const id = pickNearest(pts, sx, sy, PICK_PX);
      if (id) { slamTapNode(id); return; }
    }
    const gz = tapGroundZ(), p = toMap(sx, sy, gz);
    if (p) slamTapMap(p[0], p[1], gz, null);
  };
  const aimAllowed = () => { const ed = useSlamEdit.getState(); return !!ed.moving || (ed.tool === 'edit' && editReady()); };

  const mode = useRef({ view, follow, editAllowed });
  mode.current = { view, follow, editAllowed };
  const last = useRef({ x: 0, y: 0 });
  const pinch = useRef<number | null>(null);
  const responder = useMemo(() => PanResponder.create({
    onStartShouldSetPanResponder: () => true,
    onMoveShouldSetPanResponder: () => true,
    onPanResponderGrant: (e) => {
      last.current = { x: 0, y: 0 }; pinch.current = null;
      press.current = { x: e.nativeEvent.locationX, y: e.nativeEvent.locationY, at: Date.now(), kind: mode.current.editAllowed ? 'tap' : 'none' };
    },
    onPanResponderMove: (e, g) => {
      const tt = e.nativeEvent.touches;
      const cm = cam.current;
      const pr = press.current;
      if (tt.length >= 2) {
        if (pr.kind === 'aim') aim.current.on = false;
        pr.kind = 'pan';
        const d = Math.hypot(tt[0].pageX - tt[1].pageX, tt[0].pageY - tt[1].pageY);
        if (pinch.current != null && d > 0) cm.dist = clamp(cm.dist * (pinch.current / d), DIST_MIN, DIST_MAX);
        pinch.current = d;
        last.current = { x: g.dx, y: g.dy };
        return;
      }
      pinch.current = null;
      if (pr.kind === 'tap' && Math.hypot(g.dx, g.dy) > TAP_PX) {
        pr.kind = Date.now() - pr.at >= AIM_HOLD_MS && aimAllowed() ? 'aim' : 'pan';
        if (pr.kind === 'aim') {
          const gz = tapGroundZ(), a = toMap(pr.x, pr.y, gz);
          if (a) aim.current = { on: true, a, b: a, z: gz }; else pr.kind = 'pan';
        }
      }
      if (pr.kind === 'aim') {
        const b = toMap(pr.x + g.dx, pr.y + g.dy, aim.current.z);
        if (b) aim.current.b = b;
        return;
      }
      if (pr.kind === 'tap') return;
      const dx = g.dx - last.current.x, dy = g.dy - last.current.y;
      last.current = { x: g.dx, y: g.dy };
      if (mode.current.view === '2d') {
        if (mode.current.follow) return;
        const k = (2 * cm.dist * Math.tan((FOV * Math.PI) / 360)) / Math.max(1, box.current.h);
        cm.tx -= dx * k; cm.ty += dy * k;
      } else {
        cm.yaw -= dx * 0.01;
        cm.pitch = clamp(cm.pitch + dy * 0.01, 0.08, 1.5);
      }
    },
    onPanResponderRelease: () => {
      pinch.current = null;
      const pr = press.current;
      if (pr.kind === 'tap') onTap(pr.x, pr.y);
      else if (pr.kind === 'aim') {
        const { a, b, z } = aim.current;
        aim.current.on = false;
        const yaw = Math.hypot(b[0] - a[0], b[1] - a[1]) >= 0.15 ? Math.atan2(b[1] - a[1], b[0] - a[0]) : null;
        slamTapMap(a[0], a[1], z, yaw);
      }
      pr.kind = 'none';
    },
    onPanResponderTerminate: () => { aim.current.on = false; press.current.kind = 'none'; },
  }), []); // eslint-disable-line react-hooks/exhaustive-deps

  return (
    <View ref={wrapRef} style={StyleSheet.absoluteFill}
      onLayout={(e) => { box.current = { w: e.nativeEvent.layout.width, h: e.nativeEvent.layout.height }; }}>
      {glError && (
        <View style={[StyleSheet.absoluteFill, styles.center]}>
          <Text style={{ color: c.amber, fontSize: 12, textAlign: 'center' }}>{glError}</Text>
        </View>
      )}
      {glReady && !glError && (
        <GlBoundary onError={setGlError}>
          <Canvas
            key={glEpoch}
            frameloop="always"
            gl={steamos ? { antialias: true, powerPreference: 'high-performance' } : undefined}
            camera={{ position: [0, 20, 0.01], fov: FOV, near: 0.1, far: 2000 }}
            // @ts-expect-error
            dpr={steamos ? 1 : [1, 2]}
            onCreated={({ gl }) => { gl.debug.checkShaderErrors = false; }}>
            {backgroundColor && <color attach="background" args={[backgroundColor]} />}
            <CameraRig cam={cam} view={view} follow={follow} />
            <CamGrab out={glCam} />
            <group rotation={MAP_ROT}>
              <MapCloudLayer />
              <LiveLayers />
              <PathLayers />
              <HighlightLayer />
              <TopoLayer />
              <RobotBox />
              <AimArrow aim={aim} />
            </group>
          </Canvas>
        </GlBoundary>
      )}
      <View style={[StyleSheet.absoluteFill, { touchAction: 'none', userSelect: 'none' } as any]} {...responder.panHandlers} />
      <View style={styles.statusBar} pointerEvents="box-none">
        {status && (
          <View style={[styles.statusPill, { backgroundColor: c.glass, borderColor: c.glassLine }]} pointerEvents="none">
            <Text style={{ color: c.muted, fontSize: 11 }}>{status}</Text>
          </View>
        )}
        <SlamEditBar />
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  center: { alignItems: 'center', justifyContent: 'center', padding: 24 },
  statusBar: { position: 'absolute', top: 76, left: 0, right: 0, alignItems: 'center', gap: 6 },
  statusPill: { paddingHorizontal: 12, paddingVertical: 4, borderRadius: 999, borderWidth: 1 },
});
