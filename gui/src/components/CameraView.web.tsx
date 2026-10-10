import { createElement, useCallback, useEffect } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useRobot } from '@/store/robot';
import { useTheme } from '@/theme';
import { webrtcClient, useWebrtcStream } from '@/lib/webrtcClient';
import { holdCameraView, releaseCameraView } from '@/lib/cameraViews';
import { desktopVideo } from '@/lib/desktopVideo';
import { t } from '@/lib/i18n';

export function CameraView({ streamId, active = true }: { streamId: number; active?: boolean }) {
  const ip = useRobot((s) => s.ip);
  const { c, fonts } = useTheme();
  const { stream, imgMode, status, retryS } = useWebrtcStream();

  useEffect(() => {
    webrtcClient.connect(ip);
  }, [ip]);
  useEffect(() => { if (!active) return; holdCameraView(); return () => releaseCameraView(); }, [active]);
  useEffect(() => {
    if (active) webrtcClient.setSource(streamId);
  }, [streamId, ip, active]);

  if (imgMode)
    return createElement('img', {
      ref: (el: HTMLImageElement | null) => desktopVideo.setImgEl(el),
      style: { position: 'absolute', inset: 0, width: '100%', height: '100%', objectFit: 'cover' },
    });

  const videoRef = useCallback(
    (el: HTMLVideoElement | null) => {
      if (el && stream) {
        el.srcObject = stream as MediaStream;
        el.play().catch(() => {});
      }
    },
    [stream],
  );

  if (stream)
    return createElement('video', {
      ref: videoRef,
      autoPlay: true,
      muted: true,
      playsInline: true,
      style: { position: 'absolute', inset: 0, width: '100%', height: '100%', objectFit: 'cover' },
    });
  return (
    <View style={[StyleSheet.absoluteFill, styles.center]}>
      <Text style={{ color: c.muted, fontFamily: fonts.mono, fontSize: 11 }}>{retryS == null ? t(status)
        : t('{r} — {n}s 후 재시도').replace('{r}', t(status)).replace('{n}', String(retryS))}</Text>
    </View>
  );
}

const styles = StyleSheet.create({ center: { alignItems: 'center', justifyContent: 'center' } });
