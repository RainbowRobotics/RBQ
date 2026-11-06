import { useEffect, useState } from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';
import { useVideoPlayer, VideoView } from 'expo-video';
import { robotAuth } from '@/lib/auth';
import { ptzQcApi } from '@/lib/ptzQcApi';
import { t } from '@/lib/i18n';

export function QcVideo({ path }: { path: string; bytes?: number }) {
  const url = ptzQcApi.mediaUrl(path);
  const [go, setGo] = useState(false);
  useEffect(() => { setGo(false); }, [path]);
  if (!url) return <View style={styles.box}><Text style={styles.msg}>{t('원격(중계) 연결이나 데모에서는 녹화를 볼 수 없습니다 — 같은 망에서 열어 주세요')}</Text></View>;
  if (!go) {
    return (
      <Pressable style={styles.box} onPress={() => setGo(true)} accessibilityRole="button" accessibilityLabel={t('영상 재생')}>
        <Text style={styles.play}>▶</Text>
        <Text style={styles.msg}>{t('눌러서 영상 받기')}</Text>
      </Pressable>
    );
  }
  return <Player url={url} />;
}

function Player({ url }: { url: string }) {
  const player = useVideoPlayer({ uri: url, headers: robotAuth() }, (p) => { p.loop = false; p.play(); });
  return <VideoView player={player} nativeControls contentFit="contain" style={StyleSheet.absoluteFill} />;
}

const styles = StyleSheet.create({
  box: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, alignItems: 'center', justifyContent: 'center', padding: 12 },
  msg: { color: '#fff', fontSize: 12, textAlign: 'center' },
  play: { color: '#fff', fontSize: 28 },
});
