import { useEffect, useMemo, useRef, useState, type MutableRefObject } from 'react';
import { View, Text, StyleSheet, PanResponder, AppState, Platform } from 'react-native';
import { BufferAttribute, BufferGeometry, Color, Euler, Line, LineBasicMaterial, Vector3, type Group } from 'three';
import { Canvas, useFrame, useThree } from '@/lib/r3fCanvas';
import { useSteamOS, getPlatformInfo } from '@/lib/platformInfo';
import { GlBoundary } from '@/components/RobotModel3D';
import { useTheme } from '@/theme';
import { useDeviceToggles } from '@/store/deviceToggles';
import { useSlamnav, useSlamnavSession, type TopoNode } from '@/lib/slamnav';
import { intensityColors, type CloudSet } from '@/lib/slamnavWire';
import { t } from '@/lib/i18n';

type Cam = { yaw: number; pitch: number; dist: number; tx: number; ty: number; tz: number; init: boolean };

const FOV = 50;
const DIST_MIN = 2, DIST_MAX = 400;
const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

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

function Points({ positions, colors, color, size }: { positions?: Float32Array; colors?: Float32Array | null; color?: string; size: number }) {
  const geom = usePointsGeom(positions, colors);
  const dpr = useThree((s) => s.viewport.dpr);
  if (!geom) return null;
  return (
    <points geometry={geom}>
      <pointsMaterial size={size * dpr} sizeAttenuation={false} vertexColors={!!colors} color={colors ? undefined : color} />
    </points>
  );
}

function MapCloudLayer() {
  const map = useSlamnav((s) => s.map);
  const colors = useMemo(() => (map ? intensityColors(map.intensity, map.count, map.label) : null), [map]);
  return <Points positions={map?.positions} colors={colors} size={2} />;
}

function LiveLayers() {
  const live = useSlamnav((s) => s.live);
  const scan = useSlamnav((s) => s.scan);
  const obs = useSlamnav((s) => s.obs);
  const liveColors = useMemo(() => (live?.values ? intensityColors(live.values, live.count) : null), [live]);
  const obsColors = useMemo(() => (obs ? obsColorsOf(obs) : null), [obs]);
  return (
    <>
      <Points positions={live?.positions} colors={liveColors} size={2} />
      <Points positions={scan?.positions} color="#e8f4ff" size={2.5} />
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
  if (link === 'unsupported') return t('원격 연결에서는 SLAM 을 쓸 수 없습니다');
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

  const mode = useRef({ view, follow });
  mode.current = { view, follow };
  const last = useRef({ x: 0, y: 0 });
  const pinch = useRef<number | null>(null);
  const responder = useMemo(() => PanResponder.create({
    onStartShouldSetPanResponder: () => true,
    onMoveShouldSetPanResponder: () => true,
    onPanResponderGrant: () => { last.current = { x: 0, y: 0 }; pinch.current = null; },
    onPanResponderMove: (e, g) => {
      const tt = e.nativeEvent.touches;
      const cm = cam.current;
      if (tt.length >= 2) {
        const d = Math.hypot(tt[0].pageX - tt[1].pageX, tt[0].pageY - tt[1].pageY);
        if (pinch.current != null && d > 0) cm.dist = clamp(cm.dist * (pinch.current / d), DIST_MIN, DIST_MAX);
        pinch.current = d;
        last.current = { x: g.dx, y: g.dy };
        return;
      }
      pinch.current = null;
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
    onPanResponderRelease: () => { pinch.current = null; },
  }), []);

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
            <group rotation={MAP_ROT}>
              <MapCloudLayer />
              <LiveLayers />
              <PathLayers />
              <TopoLayer />
              <RobotBox />
            </group>
          </Canvas>
        </GlBoundary>
      )}
      <View style={[StyleSheet.absoluteFill, { touchAction: 'none', userSelect: 'none' } as any]} {...responder.panHandlers} />
      {status && (
        <View style={styles.statusBar} pointerEvents="none">
          <View style={[styles.statusPill, { backgroundColor: c.glass, borderColor: c.glassLine }]}>
            <Text style={{ color: c.muted, fontSize: 11 }}>{status}</Text>
          </View>
        </View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  center: { alignItems: 'center', justifyContent: 'center', padding: 24 },
  statusBar: { position: 'absolute', top: 76, left: 0, right: 0, alignItems: 'center' },
  statusPill: { paddingHorizontal: 12, paddingVertical: 4, borderRadius: 999, borderWidth: 1 },
});
