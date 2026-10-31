import { Fragment, useState, type ReactNode } from 'react';
import { View, StyleSheet, type LayoutChangeEvent } from 'react-native';
import Svg, { Polygon, Polyline } from 'react-native-svg';
import { useCameraCalib, type CamCalib } from '@/lib/cameraCalib';
import { useTelemetry } from '@/store/telemetry';
import { useFeatureWheel } from '@/store/capability';
import { driveGuide, edgePoint, shoulderHalfWidth, type GuideSample } from '@/lib/driveGuide';

const ZC_MIN = 0.12;
const R2_MAX = 4;

const TONE = '#FFD60A';
const FILL = 'rgba(255,214,10,0.16)';
const SHADE = 'rgba(0,0,0,0.55)';

type Pt = [number, number];

function makeProjector(cal: CamCalib, groundPos: number[], groundRpy: number[], viewW: number, viewH: number) {
  const { fx, fy, cx, cy, coeffs, tf } = cal;
  const [k1, k2, p1, p2, k3, k4 = 0, k5 = 0, k6 = 0] = coeffs;
  const [rx, ry, rz] = groundRpy;
  const h = groundPos[2];
  const ca = Math.cos(rx), sa = Math.sin(rx), cb = Math.cos(ry), sb = Math.sin(ry), cg = Math.cos(rz), sg = Math.sin(rz);
  const g = [cg * cb, cg * sb * sa - sg * ca, cg * sb * ca + sg * sa,
             sg * cb, sg * sb * sa + cg * ca, sg * sb * ca - cg * sa,
             -sb, cb * sa, cb * ca];
  const sx = viewW / cal.width, sy = viewH / cal.height;
  return ([x, y]: Pt): Pt | null => {
    const gx = Math.cos(rz) * x - Math.sin(rz) * y, gy = Math.sin(rz) * x + Math.cos(rz) * y, gz = -h;
    const bx = g[0] * gx + g[3] * gy + g[6] * gz;
    const by = g[1] * gx + g[4] * gy + g[7] * gz;
    const bz = g[2] * gx + g[5] * gy + g[8] * gz;
    const dx = bx - tf[3], dy = by - tf[7], dz = bz - tf[11];
    const xc = tf[0] * dx + tf[4] * dy + tf[8] * dz;
    const yc = tf[1] * dx + tf[5] * dy + tf[9] * dz;
    const zc = tf[2] * dx + tf[6] * dy + tf[10] * dz;
    if (zc <= ZC_MIN) return null;
    const xn = xc / zc, yn = yc / zc, r2 = xn * xn + yn * yn;
    if (r2 > R2_MAX) return null;
    const den = 1 + k4 * r2 + k5 * r2 * r2 + k6 * r2 * r2 * r2;
    if (den <= 0) return null;
    const radial = (1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2) / den;
    const xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
    const yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;
    return [(fx * xd + cx) * sx, (fy * yd + cy) * sy];
  };
}

const ptsStr = (pts: Pt[]) => pts.map(([u, v]) => `${u.toFixed(1)},${v.toFixed(1)}`).join(' ');

export function DriveGuideOverlay({ streamId }: { streamId: number }) {
  const [box, setBox] = useState({ w: 0, h: 0 });
  const cal = useCameraCalib((s) => (streamId === 1 ? s.front : streamId === 2 ? s.rear : undefined));
  const robot = useTelemetry((s) => s.robot);
  const wheel = useFeatureWheel();
  const onLayout = (e: LayoutChangeEvent) => setBox({ w: e.nativeEvent.layout.width, h: e.nativeEvent.layout.height });

  let shapes: ReactNode = null;
  if (cal && cal.fx > 0 && cal.width > 0 && robot && robot.isStanding !== false && box.w > 0) {
    const proj = makeProjector(cal, robot.groundPos, robot.groundRpy, box.w, box.h);
    const g = driveGuide(robot.cmdVel ?? [0, 0, 0], shoulderHalfWidth(wheel));
    const L = g.samples.map((s) => proj(edgePoint(s, g.halfWidth)));
    const R = g.samples.map((s) => proj(edgePoint(s, -g.halfWidth)));
    const runs: { l: Pt[]; r: Pt[] }[] = [];
    let open = false;
    L.forEach((l, i) => {
      const r = R[i];
      if (!l || !r) { open = false; return; }
      if (!open) runs.push({ l: [], r: [] });
      open = true;
      runs[runs.length - 1].l.push(l);
      runs[runs.length - 1].r.push(r);
    });
    const tick = (s: GuideSample) => {
      const a = proj(edgePoint(s, g.halfWidth)), b = proj(edgePoint(s, -g.halfWidth));
      return a && b ? [a, b] : null;
    };
    const ticks = g.ticks.map(tick).filter((t): t is Pt[] => !!t);
    shapes = (
      <>
        {runs.filter((r) => r.l.length > 1).map((r, i) => (
          <Polygon key={`f${i}`} points={ptsStr([...r.l, ...[...r.r].reverse()])} fill={FILL} />
        ))}
        {[SHADE, TONE].map((stroke, k) => (
          <Fragment key={stroke}>
            {runs.flatMap((r, i) => [r.l, r.r].map((e, j) => (
              <Polyline key={`e${k}${i}${j}`} points={ptsStr(e)} fill="none" stroke={stroke}
                strokeWidth={k ? 2.5 : 5} strokeLinejoin="round" strokeLinecap="round" />
            )))}
            {ticks.map((t, i) => (
              <Polyline key={`t${k}${i}`} points={ptsStr(t)} fill="none" stroke={stroke}
                strokeWidth={k ? 1.5 : 3.5} strokeLinecap="round" />
            ))}
          </Fragment>
        ))}
      </>
    );
  }
  return (
    <View style={StyleSheet.absoluteFill} pointerEvents="none" onLayout={onLayout}>
      {box.w > 0 && <Svg width={box.w} height={box.h}>{shapes}</Svg>}
    </View>
  );
}
