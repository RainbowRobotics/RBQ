import { View, Text, StyleSheet } from 'react-native';
import { useVideoPlayer, VideoView } from 'expo-video';
import { rest, MEDIA_ROOT, type MediaKind } from '@/lib/rest';
import { useRobotFile } from '@/lib/robotFile';
import { useRobot } from '@/store/robot';
import { t } from '@/lib/i18n';

export function MediaVideo({ kind, path, size: _size }: { kind: MediaKind; path: string; size: number }) {
  const ip = useRobot((s) => s.ip);
  const url = rest.mediaFileUrl(ip, kind, path);
  const { src, loading } = useRobotFile(ip, url, `${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`);
  const player = useVideoPlayer(src, (p) => { p.loop = false; p.play(); });
  if (!src) {
    return <View style={styles.box}><Text style={styles.msg}>{loading ? t('로봇에서 받는 중…') : t('영상을 가져오지 못했습니다')}</Text></View>;
  }
  return <VideoView player={player} nativeControls contentFit="contain" style={StyleSheet.absoluteFill} />;
}

const styles = StyleSheet.create({
  box: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, alignItems: 'center', justifyContent: 'center' },
  msg: { color: '#fff', fontSize: 12 },
});
