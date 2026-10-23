import { createElement, useCallback, useEffect } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useRobot } from '@/store/robot';
import { useTheme } from '@/theme';
import { webrtcClient, useWebrtcStream } from '@/lib/webrtcClient';
import { holdCameraView, releaseCameraView } from '@/lib/cameraViews';
import { t } from '@/lib/i18n';

export function CameraView({ streamId, active = true, replay = false }: { streamId: number; active?: boolean; replay?: boolean }) {
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
