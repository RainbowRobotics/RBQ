import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet, AppState, type AppStateStatus } from 'react-native';
// @ts-ignore
import { RTCView } from 'react-native-webrtc';
import { useRobot } from '@/store/robot';
import { useTheme } from '@/theme';
import { webrtcClient, useWebrtcStream } from '@/lib/webrtcClient';
import { holdCameraView, releaseCameraView } from '@/lib/cameraViews';
import { t } from '@/lib/i18n';

export function CameraView({ streamId, active = true }: { streamId: number; active?: boolean }) {
  const ip = useRobot((s) => s.ip);
  const { c, fonts } = useTheme();
  const { url, status, retryS } = useWebrtcStream();

  useEffect(() => {
    webrtcClient.connect(ip);
  }, [ip]);
  useEffect(() => { if (!active) return; holdCameraView(); return () => releaseCameraView(); }, [active]);
  useEffect(() => {
    if (active) webrtcClient.setSource(streamId);
  }, [streamId, ip, active]);

  const [viewEpoch, setViewEpoch] = useState(0);
  const sawBackground = useRef(false);
  useEffect(() => {
    const sub = AppState.addEventListener('change', (next: AppStateStatus) => {
      if (next === 'background') { sawBackground.current = true; return; }
      if (next !== 'active' || !sawBackground.current) return;
      sawBackground.current = false;
      webrtcClient.resumeAfterBackground();
      setViewEpoch((e) => e + 1);
    });
    return () => sub.remove();
  }, []);

  if (url) return <RTCView key={viewEpoch} streamURL={url} objectFit="cover" style={StyleSheet.absoluteFill} />;
  return (
    <View style={[StyleSheet.absoluteFill, styles.center]}>
      <Text style={{ color: c.muted, fontFamily: fonts.mono, fontSize: 11 }}>{retryS == null ? t(status)
        : t('{r} — {n}s 후 재시도').replace('{r}', t(status)).replace('{n}', String(retryS))}</Text>
    </View>
  );
}

const styles = StyleSheet.create({ center: { alignItems: 'center', justifyContent: 'center' } });
