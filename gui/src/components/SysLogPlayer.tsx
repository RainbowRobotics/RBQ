import { useEffect, useMemo, useRef, useState } from 'react';
import { View, Text, StyleSheet, FlatList, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Slider } from '@/components/ui/controls';
import type { LogLine, LogLevel } from '@/types/robot';
import { t } from '@/lib/i18n';

const SPEEDS = [0.25, 0.5, 1, 2, 4, 8, 16, 60, 300];
const LINE_H = 21;
const MAX_MARKS = 400;

const LEVEL_HEX: Record<LogLevel, string> = {
  TRACE: '#888888', DEBUG: '#90A4AE', INFO: '#4FC3F7', SUCCESS: '#81C784',
  WARNING: '#FFB74D', ERROR: '#E57373', FATAL: '#B71C1C',
};

function msOfDay(ts: string): number {
  const m = /^(\d{2}):(\d{2}):(\d{2})(?:\.(\d{1,3}))?/.exec(ts);
  if (!m) return -1;
  return +m[1] * 3600000 + +m[2] * 60000 + +m[3] * 1000 + +(m[4] ?? '0').padEnd(3, '0');
}

function fmtClock(dayMs: number) {
  const t = Math.max(0, Math.round(dayMs)) % 86400000;
  const p = (n: number, w = 2) => String(n).padStart(w, '0');
  return `${p(Math.floor(t / 3600000))}:${p(Math.floor(t / 60000) % 60)}:${p(Math.floor(t / 1000) % 60)}.${p(t % 1000, 3)}`;
}

type Entry = { ln: LogLine; ms: number };

export function SysLogPlayer({ date, logs, onClose }: {
  date: string; logs: LogLine[]; onClose: () => void;
}) {
  const { c, fonts, radius } = useTheme();
  const [nowMs, setNowMs] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const playheadMs = useRef(0);
  const listRef = useRef<FlatList>(null);

  const { entries, baseMs, durationMs } = useMemo(() => {
    const raw: Entry[] = [];
    let last = 0;
    for (const ln of logs) {
      const t = msOfDay(ln.ts);
      const ms = t >= 0 ? t : last;
      last = ms;
      raw.push({ ln, ms });
    }
    raw.sort((a, b) => a.ms - b.ms);
    const base = raw.length ? raw[0].ms : 0;
    for (const e of raw) e.ms -= base;
    const dur = (raw.length ? raw[raw.length - 1].ms : 0) + 1000;
    return { entries: raw, baseMs: base, durationMs: dur };
  }, [logs]);

  const marks = useMemo(() => {
    const all = entries
      .filter((e) => e.ln.level === 'WARNING' || e.ln.level === 'ERROR' || e.ln.level === 'FATAL')
      .map((e) => ({ pct: (e.ms / durationMs) * 100, err: e.ln.level !== 'WARNING' }));
    if (all.length <= MAX_MARKS) return all;
    const step = all.length / MAX_MARKS;
    return Array.from({ length: MAX_MARKS }, (_, i) => all[Math.floor(i * step)]);
  }, [entries, durationMs]);

  useEffect(() => {
    if (!playing) return;
    let raf = 0;
    let last = Date.now();
    const tick = () => {
      const now = Date.now();
      playheadMs.current += (now - last) * speed;
      last = now;
      if (playheadMs.current >= durationMs) {
        playheadMs.current = durationMs;
        setNowMs(durationMs);
        setPlaying(false);
        return;
      }
      setNowMs(playheadMs.current);
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [playing, speed, durationMs]);

  const seek = (ms: number) => {
    const v = Math.max(0, Math.min(durationMs, ms));
    playheadMs.current = v;
    setNowMs(v);
  };
  const togglePlay = () => {
    if (!playing && nowMs >= durationMs) seek(0);
    setPlaying((p) => !p);
  };
  const stepNext = () => {
    setPlaying(false);
    const e = entries.find((x) => x.ms > playheadMs.current);
    if (e) seek(e.ms);
  };
  const stepPrev = () => {
    setPlaying(false);
    for (let i = entries.length - 1; i >= 0; i--) {
      if (entries[i].ms < playheadMs.current) { seek(entries[i].ms); return; }
    }
  };

  const hotIdx = useMemo(() => {
    let lo = 0, hi = entries.length - 1, ans = -1;
    while (lo <= hi) {
      const mid = (lo + hi) >> 1;
      if (entries[mid].ms <= nowMs) { ans = mid; lo = mid + 1; }
      else hi = mid - 1;
    }
    return ans;
  }, [entries, nowMs]);

  useEffect(() => {
    if (hotIdx >= 0) listRef.current?.scrollToOffset({ offset: Math.max(0, hotIdx * LINE_H - 120), animated: false });
  }, [hotIdx]);

  const durS = ((durationMs - 1000) / 1000).toFixed(0);

  return (
    <View style={{ flex: 1 }}>
      <View style={styles.head}>
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '700' }}>{t('System Log 재생')}</Text>
        <Text style={{ color: c.dim, fontSize: 10 }}>
          {date} · {logs.length.toLocaleString()}{t('줄')} · {fmtClock(baseMs).slice(0, 8)}~{fmtClock(baseMs + durationMs - 1000).slice(0, 8)} ({durS}s)
        </Text>
        <View style={{ flex: 1 }} />
        <Tappable onPress={onClose} style={[styles.closeBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Icon name="x" size={13} color={c.muted} />
          <Text style={{ color: c.muted, fontSize: 11 }}>{t('닫기')}</Text>
        </Tappable>
      </View>

      <View style={[styles.logPanel, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
        <View style={[styles.logHead, { borderBottomColor: c.line2 }]}>
          <Text style={{ color: c.dim, fontSize: 9, fontWeight: '700', letterSpacing: 0.5 }}>
            SYSTEM LOG · {t('재생 위치 동기')} <Text style={{ fontWeight: '400' }}>— {t('탭하면 그 시점으로 이동')}</Text>
          </Text>
        </View>
        {entries.length === 0 ? (
          <View style={styles.center}><Text style={{ color: c.dim, fontSize: 11 }}>{t('재생할 로그가 없습니다.')}</Text></View>
        ) : (
          <FlatList
            ref={listRef}
            data={entries}
            keyExtractor={(_, i) => String(i)}
            contentContainerStyle={{ padding: 8 }}
            getItemLayout={(_, i) => ({ length: LINE_H, offset: LINE_H * i, index: i })}
            initialNumToRender={30}
            windowSize={9}
            removeClippedSubviews
            extraData={hotIdx}
            renderItem={({ item: e, index: i }: { item: Entry; index: number }) => (
              <Pressable onPress={() => { setPlaying(false); seek(e.ms); }}
                style={[styles.ln, i === hotIdx && { backgroundColor: 'rgba(77,156,245,0.12)', borderLeftWidth: 2, borderLeftColor: c.accent2 }]}>
                <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 10 }}>{e.ln.ts}</Text>
                <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 10, width: 78 }} numberOfLines={1}>[{e.ln.process}]</Text>
                <Text style={{ color: LEVEL_HEX[e.ln.level] ?? c.muted, fontFamily: fonts.mono, fontSize: 10, width: 56 }}>{e.ln.level}</Text>
                <Text style={{ color: c.text, fontFamily: fonts.mono, fontSize: 10, flex: 1 }} numberOfLines={1}>{e.ln.msg}</Text>
              </Pressable>
            )}
          />
        )}
      </View>

      <View style={[styles.playbar, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
        <Tappable onPress={stepPrev} style={[styles.stepBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Icon name="prev" size={13} color={c.muted} />
        </Tappable>
        <Tappable onPress={togglePlay}
          style={[styles.playBtn, { backgroundColor: 'rgba(77,156,245,0.14)', borderColor: 'rgba(77,156,245,0.5)' }]}>
          <Icon name={playing ? 'pause' : 'play2'} size={15} color={c.accent2} />
        </Tappable>
        <Tappable onPress={stepNext} style={[styles.stepBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Icon name="next" size={13} color={c.muted} />
        </Tappable>
        <View style={{ flex: 1 }}>
          <View style={styles.markBar}>
            {marks.map((m, i) => (
              <View key={i} style={[styles.mark, { left: `${m.pct}%`, backgroundColor: m.err ? '#E57373' : '#FFB74D' }]} />
            ))}
          </View>
          <Slider value={(nowMs / durationMs) * 100} width="100%"
            onChange={(pct) => { setPlaying(false); seek((pct / 100) * durationMs); }} />
        </View>
        <Text style={{ color: c.text, fontFamily: fonts.mono, fontSize: 10.5 }}>{fmtClock(baseMs + nowMs)}</Text>
        <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 9 }}>{hotIdx + 1}/{entries.length}{t('줄')}</Text>
        <Tappable onPress={() => setSpeed(SPEEDS[(SPEEDS.indexOf(speed) + 1) % SPEEDS.length])}
          style={[styles.speedBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Text style={{ color: c.accent2, fontSize: 10.5, fontWeight: '700' }}>{speed}x</Text>
        </Tappable>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  center: { flex: 1, alignItems: 'center', justifyContent: 'center', padding: 20 },
  head: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 16, paddingVertical: 9 },
  closeBtn: { flexDirection: 'row', alignItems: 'center', gap: 5, height: 26, paddingHorizontal: 10, borderRadius: 8, borderWidth: 1 },
  logPanel: { flex: 1, marginHorizontal: 14, borderWidth: 1, overflow: 'hidden' },
  logHead: { paddingHorizontal: 10, paddingVertical: 7, borderBottomWidth: 1 },
  ln: { flexDirection: 'row', gap: 7, height: LINE_H, alignItems: 'center', paddingLeft: 5 },
  playbar: {
    flexDirection: 'row', alignItems: 'center', gap: 10, height: 52,
    marginHorizontal: 14, marginVertical: 10, paddingHorizontal: 12, borderWidth: 1,
  },
  markBar: { height: 6, marginBottom: 1 },
  mark: { position: 'absolute', top: 0, width: 2, height: 6, borderRadius: 1 },
  stepBtn: { width: 27, height: 27, borderRadius: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  playBtn: { width: 33, height: 33, borderRadius: 9, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  speedBtn: { height: 25, paddingHorizontal: 9, borderRadius: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
