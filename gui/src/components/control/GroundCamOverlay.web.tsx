import { createElement, useEffect, useRef } from 'react';
import { View, StyleSheet } from 'react-native';
import { useCameraCalib, type CamCalib } from '@/lib/cameraCalib';
import { useWallMap, type WallMapFrame } from '@/lib/wallMap';
import { useTelemetry } from '@/store/telemetry';
import { desktopVideo } from '@/lib/desktopVideo';
import { webrtcClient } from '@/lib/webrtcClient';

const FRONT_STREAM_ID = 1;
const COMPUTE_RES = 160;

export function GroundCamOverlay({ size }: { size: number }) {
  const calib = useCameraCalib((s) => s.front);
  const grid = useWallMap((s) => s.frame);
  const calibRef = useRef<CamCalib | undefined>(calib);
  const gridRef = useRef<WallMapFrame | undefined>(grid);
  calibRef.current = calib;
  gridRef.current = grid;

  const canvasRef = useRef<HTMLCanvasElement | null>(null);
  const imgElRef = useRef<HTMLImageElement | null>(null);

  useEffect(() => {
    webrtcClient.setSource(FRONT_STREAM_ID);
  }, []);

  useEffect(() => {
    let raf = 0;
    let stopped = false;
    const srcCanvas = document.createElement('canvas');
    const srcCtx = srcCanvas.getContext('2d', { willReadFrequently: true }) as CanvasRenderingContext2D | null;

    const draw = () => {
      if (stopped) return;
      raf = requestAnimationFrame(draw);
      const canvas = canvasRef.current;
      const img = imgElRef.current;
      const c = calibRef.current;
      const g = gridRef.current;
      const ctx = canvas?.getContext('2d');
      if (!canvas || !ctx || !srcCtx || !img || !c || !g) return;
      if (!img.complete || img.naturalWidth === 0 || c.width === 0 || c.fx === 0) return;

      if (srcCanvas.width !== img.naturalWidth || srcCanvas.height !== img.naturalHeight) {
        srcCanvas.width = img.naturalWidth;
        srcCanvas.height = img.naturalHeight;
      }
      srcCtx.drawImage(img, 0, 0);
      let srcData: ImageData;
      try {
        srcData = srcCtx.getImageData(0, 0, srcCanvas.width, srcCanvas.height);
      } catch {
        return;
      }
      const sw = srcData.width, sh = srcData.height, sPix = srcData.data;

      if (canvas.width !== COMPUTE_RES) canvas.width = COMPUTE_RES;
      if (canvas.height !== COMPUTE_RES) canvas.height = COMPUTE_RES;
      const dst = ctx.createImageData(COMPUTE_RES, COMPUTE_RES);
      const dPix = dst.data;

      const { fx, fy, cx: ppx, cy: ppy, coeffs, tf } = c;
      const k1 = coeffs[0], k2 = coeffs[1], p1 = coeffs[2], p2 = coeffs[3], k3 = coeffs[4];
      const r00 = tf[0], r01 = tf[1], r02 = tf[2], tx = tf[3];
      const r10 = tf[4], r11 = tf[5], r12 = tf[6], ty = tf[7];
      const r20 = tf[8], r21 = tf[9], r22 = tf[10], tz = tf[11];

      const { rows, cols, gs } = g;
      const gcx = cols / 2, gcy = rows / 2;
      const robot = useTelemetry.getState().robot;
      const [gtx, gty, gtz] = robot?.groundPos ?? [0, 0, 0.42];
      const [grx, gry, grz] = robot?.groundRpy ?? [0, 0, 0];
      const cgx = Math.cos(grx), sgx = Math.sin(grx);
      const cgy = Math.cos(gry), sgy = Math.sin(gry);
      const cgz = Math.cos(grz), sgz = Math.sin(grz);
      const g00 = cgz * cgy, g01 = cgz * sgy * sgx - sgz * cgx, g02 = cgz * sgy * cgx + sgz * sgx;
      const g10 = sgz * cgy, g11 = sgz * sgy * sgx + cgz * cgx, g12 = sgz * sgy * cgx - cgz * sgx;
      const g20 = -sgy,      g21 = cgy * sgx,                   g22 = cgy * cgx;

      const MAX_GROUND_DIST = 1.0, GROUND_FADE_DIST = 0.7;
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
          const dx = bx - tx, dy = by - ty, dz = bz - tz;
          const xc = r00 * dx + r10 * dy + r20 * dz;
          const yc = r01 * dx + r11 * dy + r21 * dz;
          const zc = r02 * dx + r12 * dy + r22 * dz;
          const ZC_MIN = 0.12, ZC_FADE = 0.3;
          if (zc <= ZC_MIN) { dPix[o + 3] = 0; continue; }
          const xn = xc / zc, yn = yc / zc;
          const r2 = xn * xn + yn * yn;
          const radial = 1 + k1 * r2 + k2 * r2 * r2 + k3 * r2 * r2 * r2;
          const xd = xn * radial + 2 * p1 * xn * yn + p2 * (r2 + 2 * xn * xn);
          const yd = yn * radial + p1 * (r2 + 2 * yn * yn) + 2 * p2 * xn * yn;
          const su = fx * xd + ppx, sv = fy * yd + ppy;
          if (su < 0 || su >= sw || sv < 0 || sv >= sh) { dPix[o + 3] = 0; continue; }
          const si = (((sv | 0) * sw) + (su | 0)) * 4;
          const fade = Math.min(1, (zc - ZC_MIN) / (ZC_FADE - ZC_MIN));
          dPix[o] = sPix[si]; dPix[o + 1] = sPix[si + 1]; dPix[o + 2] = sPix[si + 2]; dPix[o + 3] = 210 * fade * distFade;
        }
      }
      ctx.putImageData(dst, 0, 0);
    };
    raf = requestAnimationFrame(draw);
    return () => { stopped = true; cancelAnimationFrame(raf); };
  }, []);

  return (
    <View style={[StyleSheet.absoluteFill, { alignItems: 'center', justifyContent: 'center' }]} pointerEvents="none">
      {createElement('img', {
        ref: (el: HTMLImageElement | null) => {
          imgElRef.current = el;
          desktopVideo.setGroundImgEl(el);
        },
        style: { display: 'none' },
      })}
      {createElement('canvas', {
        ref: (el: HTMLCanvasElement | null) => { canvasRef.current = el; },
        style: { width: size, height: size, imageRendering: 'pixelated' },
      })}
    </View>
  );
}
