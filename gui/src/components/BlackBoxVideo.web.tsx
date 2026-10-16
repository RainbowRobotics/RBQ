import { createElement, useEffect, useMemo, useRef } from 'react';
import { View, Text, Pressable, StyleSheet } from 'react-native';
import type { BlackboxSession, BbVideoClip } from '@/lib/blackbox';

function clipStatus(clip?: BbVideoClip): string {
  if (!clip) return '(no video)';
  if (clip.pending) return '영상 인코딩 중…';
  if (clip.failed) return '영상을 가져오지 못했습니다';
  return '(no video)';
}

const SYNC_TOL_S = 0.25;
const MAX_PLAY_RATE = 4;

function Cam({ clip, label, targetMs, playing, speed, onRetry }: {
  clip?: BbVideoClip; label: string; targetMs: number; playing: boolean; speed: number;
  onRetry?: () => void;
}) {
  const el = useRef<HTMLVideoElement | null>(null);
  const lastSeekAt = useRef(0);
  const buf = clip?.buf;
  const url = useMemo(
    () => (buf && buf.byteLength > 0 ? URL.createObjectURL(new Blob([buf], { type: 'video/mp4' })) : ''),
    [buf],
  );
  useEffect(() => () => { if (url) URL.revokeObjectURL(url); }, [url]);

  const targetS = Math.max(0, targetMs / 1000);
  useEffect(() => {
    const v = el.current;
    if (!v || !url) return;
    if (Math.abs(v.currentTime - targetS) <= SYNC_TOL_S) return;
    const now = Date.now();
    if (playing && now - lastSeekAt.current < 200) return;
    lastSeekAt.current = now;
    try { v.currentTime = targetS; } catch { }
  }, [targetS, playing, url]);

  useEffect(() => {
    const v = el.current;
    if (!v || !url) return;
    if (playing && speed <= MAX_PLAY_RATE) {
      try { v.playbackRate = speed; } catch { }
      v.play().catch(() => { });
    } else {
      v.pause();
    }
  }, [playing, speed, url]);

  return (
    <View style={styles.cell}>
      {url
        ? createElement('video', {
            ref: (n: HTMLVideoElement | null) => { el.current = n; },
            src: url,
            muted: true,
            playsInline: true,
            preload: 'auto',
            style: { width: '100%', height: '100%', objectFit: 'contain', background: '#000' },
          })
        : clip?.failed && onRetry
        ? <Pressable onPress={onRetry}><Text style={[styles.sub, styles.retry]}>{clipStatus(clip)} · 다시 시도</Text></Pressable>
        : <Text style={styles.sub}>{clipStatus(clip)}</Text>}
      <Text style={styles.tag}>{label}</Text>
    </View>
  );
}

export function BlackBoxVideo({ sess, frame, playing, speed, onRetry }: {
  sess: BlackboxSession | null; frame: number; playing: boolean; speed: number;
  ip: string; date: string; session: string;
  onRetry?: () => void;
}) {
  const dataMs = sess ? frame * sess.tickMs : 0;
  return (
    <>
      {(['front', 'rear'] as const).map((cam) => (
        <Cam key={cam} clip={sess?.video[cam]} label={cam === 'front' ? 'Front' : 'Rear'} onRetry={onRetry}
             targetMs={dataMs + (sess?.video[cam]?.skewMs ?? 0)} playing={playing} speed={speed} />
      ))}
    </>
  );
}

const styles = StyleSheet.create({
  cell: { flex: 1, alignItems: 'center', justifyContent: 'center', overflow: 'hidden',
          borderRightWidth: StyleSheet.hairlineWidth, borderColor: 'rgba(255,255,255,0.08)' },
  sub: { color: 'rgba(255,255,255,0.25)', fontSize: 9 },
  retry: { color: 'rgba(255,255,255,0.55)', textDecorationLine: 'underline' },
  tag: { position: 'absolute', left: 6, top: 4, color: 'rgba(255,255,255,0.55)',
         fontSize: 9, fontWeight: '700', letterSpacing: 0.4 },
});
