import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { View, Text, StyleSheet, Pressable, ActivityIndicator, useWindowDimensions, type GestureResponderEvent } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Segmented, Select } from '@/components/ui/controls';
import { CameraView } from '@/components/CameraView';
import { rest, type ArchiveCmd, type ArchiveIndex, type ArchiveState } from '@/lib/rest';
import { streamerReady } from '@/lib/media';
import {
  ARCHIVE_SPEEDS, coverage, timeAt, firstStart, clock, livePos, fmtDay, camLabel, archiveErrText,
  viewWindow, needsRecenter, ticks, spans, durLabel, type ArchiveZoom,
} from '@/lib/archive';
import { t } from '@/lib/i18n';

const POLL_MS = 1000;
const JUMPS: { ms: number; label: string }[] = [
  { ms: -300_000, label: '-5m' }, { ms: -30_000, label: '-30s' }, { ms: 30_000, label: '+30s' }, { ms: 300_000, label: '+5m' },
];

export function ArchiveReplay({ ip }: { ip: string }) {
  const { c, fonts, radius } = useTheme();
  const [idx, setIdx] = useState<ArchiveIndex | null>(null);
  const [date, setDate] = useState('');
  const [cam, setCam] = useState('front');
  const [loading, setLoading] = useState(false);
  const [err, setErr] = useState('');
  const [st, setSt] = useState<ArchiveState | null>(null);
  const [stAt, setStAt] = useState(0);
  const [now, setNow] = useState(Date.now());
  const [barW, setBarW] = useState(0);
  const [zoom, setZoom] = useState<ArchiveZoom>('fit');
  const [anchor, setAnchor] = useState<number | null>(null);
  const playing = st?.mode === 'archive';
  const playingRef = useRef(false);
  playingRef.current = playing;
  const triedRef = useRef(false);
  const mountedRef = useRef(true);

  const loadIndex = useCallback(async (d?: string) => {
    setLoading(true);
    setErr('');
    try {
      await streamerReady(ip);
      const r = await rest.archiveIndex(ip, d);
      setIdx(r ?? {});
      setDate(r?.date ?? '');
      const cams = r?.cams ?? [];
      setCam((cur) => (cams.length === 0 || cams.includes(cur) ? cur : cams[0]));
    } catch (e) {
      setErr(archiveErrText(e));
    } finally {
      setLoading(false);
    }
  }, [ip]);

  useEffect(() => { void loadIndex(); }, [loadIndex]);

  const send = useCallback(async (cmd: ArchiveCmd): Promise<boolean> => {
    if (cmd.action === 'play' || cmd.action === 'seek') triedRef.current = true;
    setErr('');
    try {
      await streamerReady(ip);
      const r = await rest.archivePlayback(ip, cmd);
      if (!mountedRef.current) {
        if (r?.mode === 'archive') void rest.archivePlayback(ip, { action: 'live' }).catch(() => {});
        return true;
      }
      if (r?.mode !== 'archive') triedRef.current = false;
      setSt(r?.mode === 'archive' ? r : null);
      setStAt(Date.now());
      return true;
    } catch (e) {
      if (mountedRef.current) setErr(archiveErrText(e));
      return false;
    }
  }, [ip]);

  useEffect(() => {
    if (!playing) return;
    const poll = setInterval(() => { void send({ action: 'state' }); }, POLL_MS);
    const tick = setInterval(() => setNow(Date.now()), 250);
    return () => { clearInterval(poll); clearInterval(tick); };
  }, [playing, send]);

  useEffect(() => {
    mountedRef.current = true;
    return () => {
      mountedRef.current = false;
      if (playingRef.current || triedRef.current) void rest.archivePlayback(ip, { action: 'live' }).catch(() => {});
    };
  }, [ip]);

  const segs = useMemo(() => idx?.segments ?? [], [idx]);
  const dayStart = idx?.dayStartMs ?? 0;
  const cams = idx?.cams ?? [];
  const running = playing && !st?.paused && !st?.ended;
  const pos = playing ? livePos(st?.posMs ?? 0, stAt, now, st?.speed ?? 1, running && !st?.waiting) : null;
  const win = useMemo(() => viewWindow(zoom, segs, cam, dayStart, anchor), [zoom, segs, cam, dayStart, anchor]);
  useEffect(() => {
    if (zoom === 'fit' || zoom === 'day' || pos == null) return;
    if (anchor == null || needsRecenter(win, pos)) setAnchor(pos);
  }, [zoom, pos, win, anchor]);
  const bands = useMemo(() => coverage(segs, cam, win), [segs, cam, win]);
  const spanList = useMemo(() => spans(segs, cam), [segs, cam]);
  const curSpan = pos != null ? spanList.findIndex((x) => pos >= x.start - 1000 && pos <= x.end + 1000) : -1;
  const tickList = useMemo(() => (dayStart ? ticks(win, dayStart) : []), [win, dayStart]);
  const posFrac = pos != null ? (pos - win.from) / (win.to - win.from) : null;

  const playAt = (atMs: number) => void send({ action: 'play', cam, atMs });
  const onBar = (e: GestureResponderEvent) => {
    const ne = e.nativeEvent as GestureResponderEvent['nativeEvent'] & { offsetX?: number };
    const x = ne.locationX ?? ne.offsetX ?? 0;
    if (barW > 0 && dayStart) playAt(timeAt(x / barW, win));
  };
  const onCam = (k: string) => {
    setCam(k);
    if (playing && pos != null) void send({ action: 'play', cam: k, atMs: pos });
  };
  const onPlayPause = () => {
    if (!playing) {
      const at = firstStart(segs, cam);
      if (at != null) playAt(at);
      return;
    }
    void send({ action: st?.paused || st?.ended ? 'resume' : 'pause' });
  };
  const onLive = () => { void send({ action: 'live' }); };

  const days = idx?.days ?? [];
  const empty = idx != null && segs.length === 0;

  const winH = useWindowDimensions().height;
  const stageRef = useRef<View>(null);
  const [videoTop, setVideoTop] = useState(0);
  const [belowH, setBelowH] = useState(0);
  const videoMaxW = Math.min(960, Math.max(320, (winH - videoTop - belowH - 56) * 16 / 9));

  return (
    <View style={S.root}>
      <View style={S.head}>
        <View style={{ minWidth: 150 }}>
          <Select label={t('날짜')} value={date} onChange={(d) => { void loadIndex(d); }}
            options={days.map((d) => ({ key: d, label: fmtDay(d) }))} />
        </View>
        {cams.length > 1 && (
          <View style={{ minWidth: 140 }}>
            <Segmented value={cam} onChange={onCam} options={cams.map((k) => ({ key: k, label: camLabel(k) }))} />
          </View>
        )}
        <Tappable onPress={() => { void loadIndex(date || undefined); }}
          style={[S.chip, { borderColor: c.line, backgroundColor: c.elev }]}>
          <Icon name="recover" size={11} color={c.accent2} />
          <Text style={{ color: c.accent2, fontSize: 9 }}>{t('새로고침')}</Text>
        </Tappable>
        <View style={{ flex: 1 }} />
        {idx?.enabled === false && (
          <Text style={{ color: c.amberTx, fontSize: 10 }}>{t('로봇의 상시 녹화가 꺼져 있습니다 — 새 녹화가 쌓이지 않습니다')}</Text>
        )}
      </View>

      <View ref={stageRef} style={S.stage}
        onLayout={() => stageRef.current?.measureInWindow((_x, y) => { if (y > 0) setVideoTop(y); })}>
        <View style={[S.video, { maxWidth: videoMaxW, borderRadius: radius.md, borderColor: c.line }]}>
          {playing ? <CameraView streamId={0} replay /> : (
            <View style={S.center}>
              {loading ? <ActivityIndicator color={c.accent2} /> : (
                <Text style={{ color: 'rgba(255,255,255,0.72)', fontSize: 12, textAlign: 'center', paddingHorizontal: 24 }}>
                  {empty
                    ? t('이 날은 상시 녹화 파일이 없습니다. 로봇의 상시 녹화가 켜져 있어야 쌓입니다.')
                    : t('아래 시간 막대에서 볼 시각을 누르면 로봇이 그 시각부터 재생합니다.')}
                </Text>
              )}
            </View>
          )}
          {playing && st?.waiting && !st?.ended && (
            <View style={[S.badge, { backgroundColor: 'rgba(0,0,0,0.55)', borderColor: c.line }]}>
              <Text style={{ color: c.text, fontSize: 11 }}>{t('최신 녹화를 기다리는 중 — 로봇이 저장하는 대로 이어서 재생합니다')}</Text>
            </View>
          )}
          {playing && st?.ended && (
            <View style={[S.badge, { backgroundColor: 'rgba(0,0,0,0.55)', borderColor: c.line }]}>
              <Text style={{ color: c.text, fontSize: 11 }}>
                {st.error ? t('재생할 녹화가 없습니다')
                  : st.nextMs && dayStart ? t('이 녹화 구간이 끝났습니다 — 재생을 누르면 다음 구간({t})부터 봅니다').replace('{t}', clock(st.nextMs, dayStart))
                  : t('녹화 끝 — 이후 녹화가 없습니다')}
              </Text>
            </View>
          )}
        </View>
      </View>

      <View onLayout={(e) => setBelowH(e.nativeEvent.layout.height)}>
        {err !== '' && <Text style={{ color: c.redTx, fontSize: 11, paddingTop: 6 }}>{err}</Text>}

        {spanList.length > 0 && (
          <View style={S.spanRow}>
            <Text style={{ color: c.dim, fontSize: 10 }}>{t('녹화 구간')}</Text>
            {spanList.length > 6 && <Text style={{ color: c.dim, fontSize: 10 }}>{`+${spanList.length - 6}`}</Text>}
            {spanList.slice(-6).map((x) => {
              const on = spanList.indexOf(x) === curSpan;
              return (
                <Tappable key={x.start} onPress={() => playAt(x.start)}
                  style={[S.chip, { borderColor: on ? c.accent2 : c.line, backgroundColor: on ? 'rgba(77,156,245,0.14)' : c.elev }]}>
                  <Text style={{ color: on ? c.accent2 : c.muted, fontSize: 10, fontFamily: fonts.mono }}>
                    {`${clock(x.start, dayStart)} – ${clock(x.end, dayStart)}  (${durLabel(x.end - x.start)})`}
                  </Text>
                </Tappable>
              );
            })}
          </View>
        )}

        <View style={S.zoomRow}>
          <View style={{ minWidth: 260 }}>
            <Segmented value={zoom} onChange={(z) => { setZoom(z); setAnchor(pos); }}
              options={[{ key: 'fit', label: t('맞춤') }, { key: 'day', label: t('24시간') }, { key: 'hour', label: t('1시간') }, { key: 'min10', label: t('10분') }]} />
          </View>
          <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono }}>
            {dayStart ? `${clock(win.from, dayStart).slice(0, 5)} – ${clock(win.to, dayStart).slice(0, 5)}` : ''}
          </Text>
        </View>
        <View style={S.timeline}>
          <Pressable onPress={onBar} onLayout={(e) => setBarW(e.nativeEvent.layout.width)}
            style={[S.bar, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
            {bands.map((b, i) => (
              <View key={i} pointerEvents="none"
                style={[S.band, { left: `${b.from * 100}%`, width: `${Math.max(0.15, (b.to - b.from) * 100)}%`, backgroundColor: c.accent2 }]} />
            ))}
            {posFrac != null && posFrac >= 0 && posFrac <= 1 && (
              <View pointerEvents="none" style={[S.head2, { left: `${posFrac * 100}%`, backgroundColor: c.redbright }]} />
            )}
          </Pressable>
          <View style={S.ticks} pointerEvents="none">
            {tickList.map((k) => (
              <Text key={k.label + k.frac} style={[S.tick, { left: `${k.frac * 100}%`, color: c.dim, fontFamily: fonts.mono }]}>{k.label}</Text>
            ))}
          </View>
        </View>

        <View style={S.ctrls}>
          <Tappable onPress={onPlayPause} disabled={!playing && (segs.length === 0)}
            style={[S.chip, running
              ? { borderColor: 'rgba(210,153,34,0.5)', backgroundColor: 'rgba(210,153,34,0.12)' }
              : { borderColor: 'rgba(63,185,80,0.5)', backgroundColor: 'rgba(63,185,80,0.12)' },
            { opacity: !playing && segs.length === 0 ? 0.4 : 1 }]}>
            <Icon name={running ? 'pause' : 'play2'} size={12} color={running ? c.amber : c.green} />
            <Text style={{ color: running ? c.amberTx : c.greenTx, fontSize: 10, fontWeight: '700' }}>{running ? t('일시정지') : t('재생')}</Text>
          </Tappable>
          {JUMPS.map((j) => (
            <Tappable key={j.label} onPress={() => pos != null && playAt(pos + j.ms)} disabled={!playing}
              style={[S.chip, { borderColor: c.line, backgroundColor: c.elev, opacity: playing ? 1 : 0.4 }]}>
              <Text style={{ color: c.muted, fontSize: 10, fontFamily: fonts.mono }}>{j.label}</Text>
            </Tappable>
          ))}
          <View style={[S.divider, { backgroundColor: c.line2 }]} />
          <View style={S.speedGroup}>
            {SPEED_BUTTONS.map((v) => {
              const on = playing && (st?.speed ?? 1) === v;
              return (
                <Tappable key={v} onPress={() => { void send({ action: 'speed', speed: v }); }} disabled={!playing}
                  style={[S.chip, { borderColor: on ? 'rgba(77,156,245,0.5)' : c.line, backgroundColor: on ? 'rgba(77,156,245,0.14)' : c.elev, opacity: playing ? 1 : 0.4 }]}>
                  <Text style={{ color: on ? c.accent2 : c.muted, fontSize: 10, fontFamily: fonts.mono, fontWeight: on ? '700' : '400' }}>{`${v}x`}</Text>
                </Tappable>
              );
            })}
          </View>
          <View style={[S.divider, { backgroundColor: c.line2 }]} />
          <Text style={{ color: playing ? c.text : c.dim, fontSize: 13, fontFamily: fonts.mono, marginLeft: 6 }}>
            {pos != null && dayStart ? clock(pos, dayStart) : '--:--:--'}
          </Text>
          <View style={{ flex: 1 }} />
          <Tappable onPress={onLive} disabled={!playing}
            style={[S.chip, { borderColor: 'rgba(231,51,28,0.5)', backgroundColor: 'rgba(231,51,28,0.10)', opacity: playing ? 1 : 0.4 }]}>
            <View style={[S.liveDot, { backgroundColor: c.redbright }]} />
            <Text style={{ color: c.redbright, fontSize: 10, fontWeight: '700' }}>{t('재생 끝내기')}</Text>
          </Tappable>
        </View>
      </View>
    </View>
  );
}

const SPEED_BUTTONS = [...ARCHIVE_SPEEDS].sort((a, b) => a - b);

const S = StyleSheet.create({
  root: {},
  head: { flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap', gap: 8, paddingBottom: 8 },
  stage: { alignItems: 'center' },
  video: { width: '100%', aspectRatio: 16 / 9, overflow: 'hidden', backgroundColor: '#000', borderWidth: 1 },
  center: { flex: 1, alignItems: 'center', justifyContent: 'center' },
  badge: { position: 'absolute', left: 10, bottom: 10, paddingHorizontal: 10, paddingVertical: 5, borderRadius: 6, borderWidth: 1 },
  spanRow: { flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap', gap: 6, paddingTop: 10 },
  zoomRow: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingTop: 10 },
  timeline: { paddingTop: 6 },
  bar: { height: 26, borderWidth: 1, overflow: 'hidden' },
  band: { position: 'absolute', top: 3, bottom: 3, opacity: 0.75 },
  head2: { position: 'absolute', top: 0, bottom: 0, width: 2, marginLeft: -1 },
  ticks: { height: 14, marginTop: 2 },
  tick: { position: 'absolute', fontSize: 9, marginLeft: -13 },
  ctrls: { flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap', gap: 6, paddingTop: 10 },
  chip: { flexDirection: 'row', alignItems: 'center', gap: 5, borderWidth: 1, borderRadius: 6, paddingHorizontal: 9, paddingVertical: 5 },
  liveDot: { width: 7, height: 7, borderRadius: 4 },
  speedGroup: { flexDirection: 'row', alignItems: 'center', gap: 4 },
  divider: { width: 1, height: 18, marginHorizontal: 4 },
});
