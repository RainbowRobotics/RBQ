import { createElement, useEffect, useState } from 'react';
import { View, Text, ActivityIndicator, Pressable, StyleSheet } from 'react-native';
import { robotAuth } from '@/lib/auth';
import { ptzQcApi } from '@/lib/ptzQcApi';
import { humanSize } from '@/lib/media';
import { t } from '@/lib/i18n';

export function QcVideo({ path, bytes = 0 }: { path: string; bytes?: number }) {
  const [url, setUrl] = useState<string | null>(null);
  const [got, setGot] = useState(0);
  const [err, setErr] = useState<string | null>(null);
  const [go, setGo] = useState(false);
  const src = ptzQcApi.mediaUrl(path);
  useEffect(() => { setGo(false); }, [path]);
  useEffect(() => {
    if (!src || !go) return;
    let alive = true, made: string | null = null;
    const ctl = new AbortController();
    setUrl(null); setGot(0); setErr(null);
    (async () => {
      try {
        const res = await fetch(src, { headers: robotAuth(), signal: ctl.signal });
        if (!res.ok || !res.body) throw new Error(`HTTP ${res.status}`);
        const reader = res.body.getReader();
        const parts: Uint8Array[] = [];
        let n = 0;
        for (;;) {
          const { done, value } = await reader.read();
          if (done) break;
          parts.push(value); n += value.byteLength;
          if (alive) setGot(n);
        }
        if (!alive) return;
        made = URL.createObjectURL(new Blob(parts as BlobPart[], { type: path.endsWith('.mkv') ? 'video/x-matroska' : 'video/mp4' }));
        setUrl(made);
      } catch (e) {
        if (alive && (e as Error)?.name !== 'AbortError') setErr((e as Error)?.message ?? 'error');
      }
    })();
    return () => { alive = false; ctl.abort(); if (made) URL.revokeObjectURL(made); };
  }, [src, path, go]);
  if (!src) return <View style={styles.box}><Text style={styles.msg}>{t('원격(중계) 연결이나 데모에서는 녹화를 볼 수 없습니다 — 같은 망에서 열어 주세요')}</Text></View>;
  if (!go) {
    return (
      <Pressable style={styles.box} onPress={() => setGo(true)} accessibilityRole="button" accessibilityLabel={t('영상 재생')}>
        <Text style={styles.play}>▶</Text>
        <Text style={styles.msg}>{t('눌러서 영상 받기')}{bytes ? ` · ${humanSize(bytes)}` : ''}</Text>
      </Pressable>
    );
  }
  if (err) return <View style={styles.box}><Text style={styles.msg}>{t('영상을 가져오지 못했습니다')} — {err}</Text></View>;
  if (!url) {
    return (
      <View style={styles.box}>
        <ActivityIndicator color="#fff" />
        <Text style={styles.msg}>{t('영상 받는 중…')} {humanSize(got)}{bytes ? ` / ${humanSize(bytes)}` : ''}</Text>
      </View>
    );
  }
  return createElement('video', { src: url, controls: true, autoPlay: true, playsInline: true, style: { width: '100%', height: '100%', objectFit: 'contain', background: '#000' } });
}

const styles = StyleSheet.create({
  box: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, alignItems: 'center', justifyContent: 'center', gap: 8, padding: 12 },
  msg: { color: '#fff', fontSize: 12, textAlign: 'center' },
  play: { color: '#fff', fontSize: 28 },
});
