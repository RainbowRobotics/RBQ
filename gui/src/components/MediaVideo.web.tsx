import { createElement, useEffect, useState } from 'react';
import { View, Text, ActivityIndicator, StyleSheet } from 'react-native';
import { robotAuth } from '@/lib/auth';
import { restBase, MEDIA_ROOT, type MediaKind } from '@/lib/rest';
import { useRobot } from '@/store/robot';
import { humanSize } from '@/lib/media';
import { t } from '@/lib/i18n';

export function MediaVideo({ kind, path, size }: { kind: MediaKind; path: string; size: number }) {
  const ip = useRobot((s) => s.ip);
  const [url, setUrl] = useState<string | null>(null);
  const [got, setGot] = useState(0);
  const [err, setErr] = useState<string | null>(null);
  useEffect(() => {
    let alive = true;
    let made: string | null = null;
    const ctl = new AbortController();
    setUrl(null); setGot(0); setErr(null);
    (async () => {
      try {
        const res = await fetch(`${restBase(ip)}${MEDIA_ROOT[kind]}/file?path=${encodeURIComponent(path)}`,
          { headers: robotAuth(), signal: ctl.signal });
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
        made = URL.createObjectURL(new Blob(parts as BlobPart[], { type: 'video/mp4' }));
        setUrl(made);
      } catch (e) {
        if (alive && (e as Error)?.name !== 'AbortError') setErr((e as Error)?.message ?? 'error');
      }
    })();
    return () => { alive = false; ctl.abort(); if (made) URL.revokeObjectURL(made); };
  }, [ip, kind, path]);

  if (err) return <View style={styles.box}><Text style={styles.msg}>{t('영상을 가져오지 못했습니다')} — {err}</Text></View>;
  if (!url) {
    const pct = size > 0 ? Math.min(100, Math.round((got / size) * 100)) : 0;
    return (
      <View style={styles.box}>
        <ActivityIndicator color="#fff" />
        <Text style={styles.msg}>{t('영상 받는 중…')} {humanSize(got)} / {humanSize(size)} ({pct}%)</Text>
      </View>
    );
  }
  return createElement('video', {
    src: url, controls: true, autoPlay: true, playsInline: true,
    style: { width: '100%', height: '100%', objectFit: 'contain', background: '#000' },
  });
}

const styles = StyleSheet.create({
  box: { position: 'absolute', top: 0, left: 0, right: 0, bottom: 0, alignItems: 'center', justifyContent: 'center', gap: 8 },
  msg: { color: '#fff', fontSize: 12 },
});
