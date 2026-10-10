import { View, Text, StyleSheet } from 'react-native';
import { useVideoPlayer, VideoView } from 'expo-video';
import { robotAuth } from '@/lib/auth';
import { rest, type MediaKind } from '@/lib/rest';
import { useRobot } from '@/store/robot';
import { t } from '@/lib/i18n';

export function MediaVideo({ kind, path, size: _size }: { kind: MediaKind; path: string; size: number }) {
  const ip = useRobot((s) => s.ip);
  const url = rest.mediaFileUrl(ip, kind, path);
  const player = useVideoPlayer(url ? { uri: url, headers: robotAuth() } : null,
    (p) => { p.loop = false; p.play(); });
  if (!url) {
    return <View style={styles.box}><Text style={styles.msg}>{t('원격(중계) 연결에서는 영상을 볼 수 없습니다')}</Text></View>;
  }
  return <VideoView player={player} nativeControls contentFit="contain" style={StyleSheet.absoluteFill} />;
}

const styles = StyleSheet.create({
  box: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, alignItems: 'center', justifyContent: 'center' },
  msg: { color: '#fff', fontSize: 12 },
});
