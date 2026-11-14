import { createElement, useEffect, useRef } from 'react';
import { View, StyleSheet } from 'react-native';
import { useCameraCalib, type CamCalib } from '@/lib/cameraCalib';
import { useWallMap, type WallMapFrame } from '@/lib/wallMap';
import { useTelemetry } from '@/store/telemetry';
import { useRobot } from '@/store/robot';
import { useWebrtcStream, webrtcClient } from '@/lib/webrtcClient';
import { holdCameraView, releaseCameraView } from '@/lib/cameraViews';
import { STAIRS_STREAM_ID } from '@/lib/visionSources';

const COMPUTE_RES = 160;

type Hit = { su: number; sv: number; fade: number };

export function GroundCamOverlay({ size, active = true }: { size: number; active?: boolean }) {
  const front = useCameraCalib((s) => s.front);
  const rear = useCameraCalib((s) => s.rear);
  const bottom0 = useCameraCalib((s) => s.bottom0);
  const bottom1 = useCameraCalib((s) => s.bottom1);
  const bottom2 = useCameraCalib((s) => s.bottom2);
  const bottom3 = useCameraCalib((s) => s.bottom3);
  const grid = useWallMap((s) => s.frame);
  const frontRef = useRef<CamCalib | undefined>(front);
  const rearRef = useRef<CamCalib | undefined>(rear);
  const bottomRefs = useRef<(CamCalib | undefined)[]>([bottom0, bottom1, bottom2, bottom3]);
  const gridRef = useRef<WallMapFrame | undefined>(grid);
  frontRef.current = front;
  rearRef.current = rear;
  bottomRefs.current = [bottom0, bottom1, bottom2, bottom3];
  gridRef.current = grid;

  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const videoElRef = useRef<HTMLVideoElement | null>(null);
  const { stream } = useWebrtcStream();

  const ip = useRobot((s) => s.ip);
  useEffect(() => {
    webrtcClient.connect(ip);
  }, [ip]);

  useEffect(() => {
    if (!active) return;
    holdCameraView();
    webrtcClient.setSource(STAIRS_STREAM_ID);
    return () => releaseCameraView();
  }, [active]);

  useEffect(() => {
    let raf = 0;
    let stopped = false;
    const srcCanvas = document.createElement('canvas');
    const srcCtx = srcCanvas.getContext('2d', { willReadFrequently: true }) as CanvasRenderingContext2D | null;

    const distortPoint = (c: CamCalib, bx: number, by: number, bz: number): { xd: number; yd: number; fade: number } | null => {
      const { fx, coeffs, tf, width: cw, height: ch } = c;
      if (cw === 0 || ch === 0 || fx === 0) return null;
      const k1 = coeffs[0], k2 = coeffs[1], p1 = coeffs[2], p2 = coeffs[3], k3 = coeffs[4];
      const r00 = tf[0], r01 = tf[1], r02 = tf[2], tx = tf[3];
      const r10 = tf[4], r11 = tf[5], r12 = tf[6], ty = tf[7];
      const r20 = tf[8], r21 = tf[9], r22 = tf[10], tz = tf[11];
      const dx = bx - tx, dy = by - ty, dz = bz - tz;
      const xc = r00 * dx + r10 * dy + r20 * dz;
      const yc = r01 * dx + r11 * dy + r21 * dz;
      const zc = r02 * dx + r12 * dy + r22 * dz;
      const ZC_MIN = 0.12, ZC_FADE = 0.3;
      if (zc <= ZC_MIN) return null;
      const xn = xc / zc, yn = yc / zc;
      const r2 = xn * xn + yn * yn;
      const radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
      const xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
      const yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;
      const fade = Math.min(1, (zc - ZC_MIN) / (ZC_FADE - ZC_MIN));
      return { xd, yd, fade };
    };

    const project = (c: CamCalib, oy: number, wm: number, hm: number, bx: number, by: number, bz: number): Hit | null => {
      const d = distortPoint(c, bx, by, bz);
      if (!d) return null;
      const sx = wm / c.width, sy = hm / c.height;
      const su = (c.fx * d.xd + c.cx) * sx, sv = oy + (c.fy * d.yd + c.cy) * sy;
      if (su < 0 || su >= wm || sv < oy || sv >= oy + hm) return null;
      return { su, sv, fade: d.fade };
    };

    const projectBottom = (c: CamCalib, k: number, wm: number, ws: number, hs: number, rotCW: boolean, bx: number, by: number, bz: number): Hit | null => {
      const d = distortPoint(c, bx, by, bz);
      if (!d) return null;
      const sx = ws / c.width, sy = hs / c.height;
      const suPre = (c.fx * d.xd + c.cx) * sx, svPre = (c.fy * d.yd + c.cy) * sy;
      if (suPre < 0 || suPre >= ws || svPre < 0 || svPre >= hs) return null;
      const suPost = rotCW ? hs - 1 - svPre : svPre;
      const svPost = rotCW ? suPre : ws - 1 - suPre;
      return { su: wm + suPost, sv: ws * k + svPost, fade: d.fade };
    };

    const draw = () => {
      if (stopped) return;
      raf = requestAnimationFrame(draw);
      const canvas = canvasRef.current;
      const video = videoElRef.current;
      const cf = frontRef.current;
      const cr = rearRef.current;
      const [cb0, cb1, cb2, cb3] = bottomRefs.current;
      const g = gridRef.current;
      const ctx = canvas?.getContext('2d');
      if (!canvas || !ctx || !srcCtx || !video || !g) return;
      if (!cf && !cr && !cb0 && !cb1 && !cb2 && !cb3) return;
      if (video.readyState < 2 || video.videoWidth === 0) return;

      if (srcCanvas.width !== video.videoWidth || srcCanvas.height !== video.videoHeight) {
        srcCanvas.width = video.videoWidth;
        srcCanvas.height = video.videoHeight;
      }
      srcCtx.drawImage(video, 0, 0);
      let srcData: ImageData;
      try {
        srcData = srcCtx.getImageData(0, 0, srcCanvas.width, srcCanvas.height);
      } catch {
        return;
      }
      const sw = srcData.width, sh = srcData.height, sPix = srcData.data;
      const wm = sw / 2, hm = sh / 2;
      const ws = Math.floor(sh / 4), hs = Math.floor((ws * 9) / 16);

      if (canvas.width !== COMPUTE_RES) canvas.width = COMPUTE_RES;
      if (canvas.height !== COMPUTE_RES) canvas.height = COMPUTE_RES;
      const dst = ctx.createImageData(COMPUTE_RES, COMPUTE_RES);
      const dPix = dst.data;

      const robot = useTelemetry.getState().robot;
      const [gtx, gty, gtz] = robot?.groundPos ?? [0, 0, 0.42];
      const [grx, gry, grz] = robot?.groundRpy ?? [0, 0, 0];
      const cgx = Math.cos(grx), sgx = Math.sin(grx);
      const cgy = Math.cos(gry), sgy = Math.sin(gry);
      const cgz = Math.cos(grz), sgz = Math.sin(grz);
      const g00 = cgz * cgy, g01 = cgz * sgy * sgx - sgz * cgx, g02 = cgz * sgy * cgx + sgz * sgx;
      const g10 = sgz * cgy, g11 = sgz * sgy * sgx + cgz * cgx, g12 = sgz * sgy * cgx - cgz * sgx;
      const g20 = -sgy,      g21 = cgy * sgx,                   g22 = cgy * cgx;

      const { rows, cols, gs } = g;
      const gcx = cols / 2, gcy = rows / 2;
      const MAX_GROUND_DIST = 2.5, GROUND_FADE_DIST = 2.0;
      for (let py = 0; py < COMPUTE_RES; py++) {
        const v = (py / COMPUTE_RES) * rows;
        for (let px = 0; px < COMPUTE_RES; px++) {
          const u = (px / COMPUTE_RES) * cols;
          const o = (py * COMPUTE_RES + px) * 4;
          const X = (gcy - v) * gs;
          const Y = (gcx - u) * gs;
          const groundDist = Math.sqrt(X * X + Y * Y);
          if (groundDist >= MAX_GROUND_DIST) { dPix[o + 3] = 0; continue; }
          const distFade = groundDist <= GROUND_FADE_DIST ? 1
            : 1 - (groundDist - GROUND_FADE_DIST) / (MAX_GROUND_DIST - GROUND_FADE_DIST);
          const ggx = X - gtx, ggy = Y - gty, ggz = 0 - gtz;
          const bx = g00 * ggx + g10 * ggy + g20 * ggz;
          const by = g01 * ggx + g11 * ggy + g21 * ggz;
          const bz = g02 * ggx + g12 * ggy + g22 * ggz;

          const hit: Hit | null = (cf && project(cf, 0, wm, hm, bx, by, bz))
            || (cr && project(cr, hm, wm, hm, bx, by, bz))
            || (cb0 && projectBottom(cb0, 0, wm, ws, hs, true, bx, by, bz))
            || (cb1 && projectBottom(cb1, 1, wm, ws, hs, true, bx, by, bz))
            || (cb2 && projectBottom(cb2, 2, wm, ws, hs, true, bx, by, bz))
            || (cb3 && projectBottom(cb3, 3, wm, ws, hs, false, bx, by, bz))
            || null;
          if (!hit) { dPix[o + 3] = 0; continue; }
          const si = (((hit.sv | 0) * sw) + (hit.su | 0)) * 4;
          dPix[o] = sPix[si]; dPix[o + 1] = sPix[si + 1]; dPix[o + 2] = sPix[si + 2];
          dPix[o + 3] = 210 * hit.fade * distFade;
        }
      }
      ctx.putImageData(dst, 0, 0);
    };
    raf = requestAnimationFrame(draw);
    return () => { stopped = true; cancelAnimationFrame(raf); };
  }, []);

  return (
    <View style={[StyleSheet.absoluteFill, { alignItems: 'center', justifyContent: 'center' }]} pointerEvents="none">
      {createElement('video', {
        ref: (el: HTMLVideoElement | null) => {
          videoElRef.current = el;
          if (el && stream) { el.srcObject = stream as MediaStream; el.play().catch(() => {}); }
        },
        autoPlay: true,
        muted: true,
        playsInline: true,
        style: { display: 'none' },
      })}
      {createElement('canvas', {
        ref: (el: HTMLCanvasElement | null) => { canvasRef.current = el; },
        style: { width: size, height: size, imageRendering: 'pixelated' },
      })}
    </View>
  );
}
