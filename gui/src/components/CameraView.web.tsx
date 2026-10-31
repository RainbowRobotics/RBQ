import { createElement, useCallback, useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useRobot } from '@/store/robot';
import { useTheme } from '@/theme';
import { webrtcClient, useWebrtcStream } from '@/lib/webrtcClient';
import { holdCameraView, releaseCameraView } from '@/lib/cameraViews';
import { t } from '@/lib/i18n';

type CropRegion = 'frontRear' | 'front' | 'rear';
const CROP_RECTS: Record<CropRegion, [number, number, number, number]> = {
  frontRear: [0, 0.5, 0, 1],
  front: [0, 0.5, 0, 0.5],
  rear: [0, 0.5, 0.5, 1],
};
export function CameraView({ streamId, active = true, replay = false, cropRegion, fit = "cover" }: { streamId: number; active?: boolean; replay?: boolean; cropRegion?: CropRegion; fit?: "cover" | "contain" }) {
  const ip = useRobot((s) => s.ip);
  const { c, fonts } = useTheme();
  const { stream, status, retryS } = useWebrtcStream();

  useEffect(() => {
    if (!replay) webrtcClient.connect(ip);
  }, [ip, replay]);
  useEffect(() => { if (!active || replay) return; holdCameraView(); return () => releaseCameraView(); }, [active, replay]);
  useEffect(() => {
    if (active && !replay) webrtcClient.setSource(streamId);
  }, [streamId, ip, active, replay]);

  const [box, setBox] = useState({ w: 0, h: 0 });
  const [media, setMedia] = useState({ w: 0, h: 0 });
  const noteMediaSize = (w: number, h: number) => {
    if (w > 0 && h > 0) setMedia((m) => (m.w === w && m.h === h ? m : { w, h }));
  };
  const roRef = useRef<ResizeObserver | null>(null);
  const wrapRefCb = useCallback((el: HTMLDivElement | null) => {
    roRef.current?.disconnect();
    roRef.current = null;
    if (!el) return;
    const ro = new ResizeObserver((entries) => {
      const { width, height } = entries[0].contentRect;
      setBox((b) => (b.w === width && b.h === height ? b : { w: width, h: height }));
    });
    ro.observe(el);
    roRef.current = ro;
  }, []);
  const cropWrap = (el: ReturnType<typeof createElement>) =>
    cropRegion
      ? createElement('div', { ref: wrapRefCb, style: { position: 'absolute', inset: 0, overflow: 'hidden' } }, el)
      : el;
  const mediaStyle = (): Record<string, string | number> => {
    if (!cropRegion) return { position: 'absolute', inset: 0, width: '100%', height: '100%', objectFit: fit };
    const [x0, x1, y0, y1] = CROP_RECTS[cropRegion];
    if (!box.w || !box.h || !media.w || !media.h) {
      const sx = 1 / (x1 - x0), sy = 1 / (y1 - y0);
      return { position: 'absolute', left: `${-x0 * sx * 100}%`, top: `${-y0 * sy * 100}%`, width: `${sx * 100}%`, height: `${sy * 100}%` };
    }
    const cropWpx = (x1 - x0) * media.w, cropHpx = (y1 - y0) * media.h;
    const scale = Math.max(box.w / cropWpx, box.h / cropHpx);
    const left = box.w / 2 - ((x0 + x1) / 2) * media.w * scale;
    const top = box.h / 2 - ((y0 + y1) / 2) * media.h * scale;
    return { position: 'absolute', left, top, width: media.w * scale, height: media.h * scale };
  };

  const videoRef = useCallback(
    (el: HTMLVideoElement | null) => {
      if (el && stream) {
        el.srcObject = stream as MediaStream;
        el.play().catch(() => {});
        el.onloadedmetadata = () => noteMediaSize(el.videoWidth, el.videoHeight);
      }
    },
    [stream],
  );

  if (stream)
    return cropWrap(createElement('video', {
      ref: videoRef,
      autoPlay: true,
      muted: true,
      playsInline: true,
      style: mediaStyle(),
    }));
  return (
    <View style={[StyleSheet.absoluteFill, styles.center]}>
      <Text style={{ color: c.muted, fontFamily: fonts.mono, fontSize: 11 }}>{retryS == null ? t(status)
        : t('{r} — {n}s 후 재시도').replace('{r}', t(status)).replace('{n}', String(retryS))}</Text>
    </View>
  );
}

const styles = StyleSheet.create({ center: { alignItems: 'center', justifyContent: 'center' } });
