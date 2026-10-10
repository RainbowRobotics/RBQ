import { useEffect, useMemo, useRef } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useVideoPlayer, VideoView } from 'expo-video';
import { File, Paths } from 'expo-file-system';
import { robotAuth } from '@/lib/auth';
import { useRobots } from '@/store/robots';
import { rest } from '@/lib/rest';
import type { BlackboxSession, BbVideoClip } from '@/lib/blackbox';

const SYNC_TOL_S = 0.25;
const MAX_PLAY_RATE = 4;
const CHECK_MS = 200;

type Src = { uri: string; headers?: Record<string, string> } | null;

function Cam({ clip, cam, url, targetMs, playing, speed }: {
  clip?: BbVideoClip; cam: 'front' | 'rear'; url: string | null;
  targetMs: number; playing: boolean; speed: number;
}) {
  const buf = clip?.buf;
  const fileUri = useMemo(() => {
    if (!buf || buf.byteLength === 0) return null;
    try {
      const f = new File(Paths.cache, `bb-${cam}-${buf.byteLength}.mp4`);
      f.write(new Uint8Array(buf));
      return f.uri;
    } catch {
      return null;
    }
  }, [buf, cam]);

  const passwords = useRobots((s) => s.passwords);
  const source: Src = useMemo(
    () => (fileUri ? { uri: fileUri }
         : clip && url ? { uri: url, headers: robotAuth() }
         : null),
    [fileUri, clip, url, passwords],
  );

  const player = useVideoPlayer(source, (p) => { p.muted = true; p.loop = false; });

  const lastCheckAt = useRef(0);
  useEffect(() => {
    if (!source) return;
    const now = Date.now();
    if (playing && now - lastCheckAt.current < CHECK_MS) return;
    lastCheckAt.current = now;
    const t = Math.max(0, targetMs / 1000);
    try {
      if (Math.abs(player.currentTime - t) > SYNC_TOL_S) player.currentTime = t;
    } catch { }
  }, [targetMs, playing, player, source]);

  useEffect(() => {
    if (!source) return;
    try {
      if (playing && speed <= MAX_PLAY_RATE) { player.playbackRate = speed; player.play(); }
      else player.pause();
    } catch { }
  }, [playing, speed, player, source]);

  return (
    <View style={styles.cell}>
      {source
        ? <VideoView player={player} nativeControls={false} contentFit="contain"
                     style={StyleSheet.absoluteFill} />
        : <Text style={styles.sub}>
            {clip?.pending ? '영상 인코딩 중…' : clip?.failed ? '영상을 가져오지 못했습니다' : '(no video)'}
          </Text>}
      <Text style={styles.tag}>{cam === 'front' ? 'Front' : 'Rear'}</Text>
    </View>
  );
}

export function BlackBoxVideo({ sess, frame, playing, speed, ip, date, session }: {
  sess: BlackboxSession | null; frame: number; playing: boolean; speed: number;
  ip: string; date: string; session: string;
  onRetry?: () => void;
}) {
  const dataMs = sess ? frame * sess.tickMs : 0;
  return (
    <>
      {(['front', 'rear'] as const).map((cam) => (
        <Cam key={cam} cam={cam} clip={sess?.video[cam]}
             url={date && session ? rest.blackboxFileUrl(ip, `${date}/${session}/${cam}.mp4`) : null}
             targetMs={dataMs + (sess?.video[cam]?.skewMs ?? 0)} playing={playing} speed={speed} />
      ))}
    </>
  );
}

const styles = StyleSheet.create({
  cell: { flex: 1, alignItems: 'center', justifyContent: 'center', overflow: 'hidden',
          borderRightWidth: StyleSheet.hairlineWidth, borderColor: 'rgba(255,255,255,0.08)' },
  sub: { color: 'rgba(255,255,255,0.25)', fontSize: 9 },
  tag: { position: 'absolute', left: 6, top: 4, color: 'rgba(255,255,255,0.55)',
         fontSize: 9, fontWeight: '700', letterSpacing: 0.4 },
});
