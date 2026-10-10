import { useEffect, useMemo, useRef, useState } from 'react';
import { View, Text, StyleSheet, FlatList, ScrollView, Pressable, ActivityIndicator } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Slider } from '@/components/ui/controls';
import { RobotModel3D } from '@/components/RobotModel3D';
import { loadBlackboxSession, fetchBlackboxVideo, chan, type BlackboxSession } from '@/lib/blackbox';
import { pickPcdAt, type HmLayers } from '@/lib/heightmapCloud';
import { BlackBoxDataPanel } from '@/components/BlackBoxDataPanel';
import { BlackBoxVideo } from '@/components/BlackBoxVideo';
import { t } from '@/lib/i18n';
import { useCompactH } from '@/lib/layout';

const SPEEDS = [0.25, 0.5, 1, 2, 4, 8, 16];

const HM_MENU: ReadonlyArray<readonly [keyof HmLayers, string, string]> = [
  ['grid',  'Grid',     '#4a9fd6'],
  ['stair', 'Stair',    '#e69f00'],
  ['edge',  'Edge',     '#d55e00'],
  ['foot',  'Foot Q/A', '#ff4d6a'],
];
const LINE_H = 21;
const LOG_ROW_MIN_W = 760;

const LEVEL_HEX: Record<string, string> = {
  TRACE: '#888888', DEBUG: '#90A4AE', INFO: '#4FC3F7', SUCCESS: '#81C784',
  WARNING: '#FFB74D', ERROR: '#E57373', FATAL: '#B71C1C',
};

function gaitName(id: number) {
  return id === -1 ? 'OFF' : id === 0 ? 'SIT' : id === 1 ? 'STAND' : id === 3 ? 'TROT' : `#${id}`;
}

function fmtClock(epochMs: number) {
  const d = new Date(epochMs);
  const p = (n: number, w = 2) => String(n).padStart(w, '0');
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}.${p(d.getMilliseconds(), 3)}`;
}

function ChipBox({ k, v, color }: { k: string; v: string; color?: string }) {
  const { c } = useTheme();
  return (
    <View style={[styles.chip, { backgroundColor: c.elev, borderColor: c.line }]}>
      <Text style={{ fontSize: 7.5, fontWeight: '700', letterSpacing: 0.5, color: c.dim }}>{k}</Text>
      <Text style={{ fontFamily: 'monospace', fontSize: 11, fontWeight: '600', color: color ?? c.text }}>{v}</Text>
    </View>
  );
}

type Pane = '3d' | 'data' | 'video';

function PaneToggle({ pane, onChange }: { pane: Pane; onChange: (p: Pane) => void }) {
  const { c } = useTheme();
  return (
    <View style={styles.paneToggle}>
      {(['3d', 'data', 'video'] as const).map((k) => (
        <Pressable key={k} onPress={() => onChange(k)}
          style={[styles.paneBtn, {
            borderColor: pane === k ? 'rgba(77,156,245,0.6)' : c.line,
            backgroundColor: pane === k ? 'rgba(77,156,245,0.14)' : c.elev,
          }]}>
          <Text style={{ color: pane === k ? c.accent2 : c.dim, fontSize: 9, fontWeight: '700' }}>
            {k === '3d' ? '3D' : k === 'data' ? t('데이터') : t('영상')}
          </Text>
        </Pressable>
      ))}
    </View>
  );
}

export function BlackBoxPlayer({ ip, date, session, preloaded }: {
  ip: string; date: string; session: string;
  preloaded?: BlackboxSession;
}) {
  const { c, fonts, radius } = useTheme();
  const [sess, setSess] = useState<BlackboxSession | null>(null);
  const [err, setErr] = useState('');
  const [frame, setFrame] = useState(0);
  const [playing, setPlaying] = useState(false);
  const [speed, setSpeed] = useState(1);
  const playheadMs = useRef(0);
  const startRef = useRef(0);
  const logScrollRef = useRef<FlatList>(null);

  useEffect(() => {
    let dead = false;
    if (preloaded) { setSess(preloaded); return; }
    loadBlackboxSession(ip, date, session)
      .then((s) => { if (!dead) setSess(s); })
      .catch((e: any) => { if (!dead) setErr(t('세션을 불러오지 못했습니다 (') + (e?.message || t('오류')) + ')'); });
    return () => { dead = true; };
  }, [ip, date, session, preloaded]);

  const [reloading, setReloading] = useState(false);
  const reloadSession = () => {
    if (reloading || preloaded) return;
    setReloading(true);
    loadBlackboxSession(ip, date, session)
      .then((s) => setSess(s))
      .catch(() => { })
      .finally(() => setReloading(false));
  };

  startRef.current = sess?.startEpochMs ?? 0;
  const clipEndMs = sess ? sess.startEpochMs + sess.frameCount * sess.tickMs : 0;
  const maybeEncoding = !!sess && !sess.video.front && !sess.video.rear
                        && Date.now() - clipEndMs < 120_000;
  const hmPendKey = !sess ? ''
    : maybeEncoding ? 'meta'
    : (['front', 'rear'] as const).filter((cam) => sess.video[cam]?.pending).join(',');
  useEffect(() => {
    if (preloaded || hmPendKey === '') return;
    let dead = false, tries = 0;
    let timer: ReturnType<typeof setTimeout>;
    const tick = async () => {
      if (dead) return;
      tries += 1;
      const got = await fetchBlackboxVideo(ip, date, session, startRef.current)
        .catch(() => ({} as BlackboxSession['video']));
      if (dead) return;
      if (got.front?.buf || got.rear?.buf || got.front || got.rear) {
        setSess((prev) => (prev ? { ...prev, video: got } : prev));
        return;
      }
      if (tries < 6) { timer = setTimeout(tick, 3000); return; }
      setSess((prev) => {
        if (!prev) return prev;
        const video = { ...prev.video };
        for (const cam of ['front', 'rear'] as const) {
          video[cam] = { ...(video[cam] ?? { skewMs: 0, durationMs: 0 }), pending: false, failed: true };
        }
        return { ...prev, video };
      });
    };
    timer = setTimeout(tick, 1500);
    return () => { dead = true; clearTimeout(timer); };
  }, [hmPendKey, ip, date, session, preloaded]);

  const retryClips = () => setSess((prev) => {
    if (!prev) return prev;
    const video = { ...prev.video };
    let changed = false;
    for (const cam of ['front', 'rear'] as const) {
      if (video[cam]?.failed) { video[cam] = { ...video[cam]!, failed: false, pending: true }; changed = true; }
    }
    return changed ? { ...prev, video } : prev;
  });

  useEffect(() => {
    if (!playing || !sess) return;
    let raf = 0;
    let last = Date.now();
    const tick = () => {
      const now = Date.now();
      playheadMs.current += (now - last) * speed;
      last = now;
      const total = (sess.frameCount - 1) * sess.tickMs;
      if (playheadMs.current >= total) { playheadMs.current = total; setFrame(sess.frameCount - 1); setPlaying(false); return; }
      setFrame(Math.floor(playheadMs.current / sess.tickMs));
      raf = requestAnimationFrame(tick);
    };
    raf = requestAnimationFrame(tick);
    return () => cancelAnimationFrame(raf);
  }, [playing, speed, sess]);

  const seekFrame = (f: number) => {
    if (!sess) return;
    const nf = Math.max(0, Math.min(sess.frameCount - 1, Math.round(f)));
    playheadMs.current = nf * sess.tickMs;
    setFrame(nf);
  };
  const togglePlay = () => {
    if (!sess) return;
    if (!playing && frame >= sess.frameCount - 1) seekFrame(0);
    setPlaying((p) => !p);
  };

  const pose = useMemo(() => {
    if (!sess) return undefined;
    const joints: number[] = [];
    for (let i = 0; i < 12; i++) joints.push(chan(sess, frame, `joint.pos[${i}]`));
    const rpy: [number, number, number] = [
      chan(sess, frame, 'imu.rpy.r'), chan(sess, frame, 'imu.rpy.p'), chan(sess, frame, 'imu.rpy.y'),
    ];
    return { joints, rpy };
  }, [sess, frame]);

  const [showRef, setShowRef] = useState(false);
  const [hmSel, setHmSel] = useState<HmLayers>({ grid: true, stair: false, edge: false, foot: true });
  const [hmOpen, setHmOpen] = useState(false);
  const hmAny = hmSel.grid || hmSel.stair || hmSel.edge || hmSel.foot;
  const [pane, setPane] = useState<Pane>('3d');
  const [view, setView] = useState<'split' | 'quad' | '3d' | 'data' | 'video' | 'log'>('split');
  const compact = useCompactH();
  const vw = compact ? 'split' : view;
  const [cPane, setCPane] = useState<'3d' | 'data'>('3d');
  const [logW, setLogW] = useState(0);
  const [cLog, setCLog] = useState(false);
  const refJoints = useMemo(() => {
    if (!sess || !showRef) return undefined;
    const j: number[] = [];
    for (let i = 0; i < 12; i++) j.push(chan(sess, frame, `ref.joint.pos[${i}]`));
    return j;
  }, [sess, frame, showRef]);
  const hmFrames = useMemo(
    () => (sess?.pcd && hmAny ? pickPcdAt(sess.pcd, frame * sess.tickMs) : undefined),
    [sess, frame, hmAny],
  );

  const [levelOff, setLevelOff] = useState<Set<string>>(new Set());
  const shownLogs = useMemo(
    () => (sess ? (levelOff.size === 0 ? sess.logs : sess.logs.filter((l) => !levelOff.has(l.level))) : []),
    [sess, levelOff]);
  const absMs = sess ? sess.startEpochMs + frame * sess.tickMs : 0;
  const hotLog = useMemo(() => {
    let lo = 0, hi = shownLogs.length - 1, ans = -1;
    while (lo <= hi) { const mid = (lo + hi) >> 1; if (shownLogs[mid].epochMs <= absMs) { ans = mid; lo = mid + 1; } else hi = mid - 1; }
    return ans;
  }, [shownLogs, absMs]);

  useEffect(() => {
    if (hotLog >= 0) logScrollRef.current?.scrollToOffset({ offset: Math.max(0, hotLog * LINE_H - 120), animated: false });
  }, [hotLog]);

  const r2d = (v: number) => (v * 180) / Math.PI;

  if (err || !sess) {
    return (
      <View style={[styles.center, { backgroundColor: c.bg }]}>
        {err ? (
          <>
            <Icon name="x" size={22} color={c.redTx} />
            <Text style={{ color: c.muted, fontSize: 12, marginTop: 8, textAlign: 'center' }}>{err}</Text>
          </>
        ) : (
          <>
            <ActivityIndicator color={c.accent2} />
            <Text style={{ color: c.dim, fontSize: 11, marginTop: 8 }}>{t('세션 데이터 불러오는 중… (data.log)')}</Text>
          </>
        )}
      </View>
    );
  }

  const durS = ((sess.frameCount - 1) * sess.tickMs / 1000).toFixed(1);
  const isFall = chan(sess, frame, 'status.is_fall') !== 0;

  return (
    <View style={{ flex: 1 }}>
      {!compact && (
      <View style={styles.head}>
        <Text style={{ color: c.text, fontSize: 12, fontWeight: '700' }}>{t('Black Box 재생')}</Text>
        <Text numberOfLines={1} style={{ color: c.dim, fontSize: 10, flexShrink: 0 }}>{durS}s · {sess.frameCount}F · {sess.tickMs}ms tick</Text>
        <Text numberOfLines={1} style={{ color: sess.robot ? c.muted : c.dim, fontSize: 10, flexShrink: 1 }}>
          {sess.robot
            ? `${sess.robot.serial} (${sess.robot.via === 'meta' ? 'meta.json' : sess.robot.via === 'filename' ? t('파일명') : t('연결 중인 로봇')})`
            : t('로봇 미상')}
        </Text>
        <View style={{ flexDirection: 'row', gap: 4, marginLeft: 10, flexShrink: 0 }}>
          {([['split', t('분할')], ['quad', t('4분할')], ['3d', '3D'], ['data', t('데이터')],
             ['video', t('영상')], ['log', t('로그')]] as const).map(([k, label]) => (
            <Pressable key={k} onPress={() => setView(k)}
              style={[styles.viewBtn, {
                borderColor: view === k ? 'rgba(77,156,245,0.6)' : c.line,
                backgroundColor: view === k ? 'rgba(77,156,245,0.14)' : c.elev,
              }]}>
              <Text style={{ color: view === k ? c.accent2 : c.dim, fontSize: 9, fontWeight: '700' }}>{label}</Text>
            </Pressable>
          ))}
        </View>
        {sess.sync.warn && (
          <Pressable disabled={!sess.sync.retry || !!preloaded || reloading} onPress={reloadSession}
            style={[styles.syncWarn, { borderColor: 'rgba(251,186,22,0.55)', backgroundColor: 'rgba(251,186,22,0.12)' }]}>
            <Text style={{ color: c.amberTx, fontSize: 9, fontWeight: '700' }}>
              ⚠ {t('시간동기 불확실')} · {reloading ? t('다시 받는 중…') : sess.sync.reason}
              {sess.sync.retry && !preloaded && !reloading ? ` · ${t('눌러서 다시')}` : ''}
            </Text>
          </Pressable>
        )}
      </View>
      )}

      {(() => {
        const poseEl = (
          <View style={[styles.pose, { backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.md }]}>
            <RobotModel3D controls={false} listenPresets={false} pose={pose} ghostJoints={refJoints}
              heightmap={hmFrames} hmLayers={hmSel} />
            <Pressable onPress={() => setShowRef((v) => !v)}
              style={[styles.refBtn, {
                borderColor: showRef ? 'rgba(77,156,245,0.6)' : c.line,
                backgroundColor: showRef ? 'rgba(77,156,245,0.14)' : c.elev,
              }]}>
              <Text style={{ color: showRef ? c.accent2 : c.dim, fontSize: 9, fontWeight: '700' }}>Ref</Text>
            </Pressable>
            {!!sess?.pcd && (
              <>
                <Pressable onPress={() => setHmOpen((v) => !v)}
                  style={[styles.pcdBtn, {
                    borderColor: hmAny ? 'rgba(77,156,245,0.6)' : c.line,
                    backgroundColor: hmAny ? 'rgba(77,156,245,0.14)' : c.elev,
                  }]}>
                  <Text style={{ color: hmAny ? c.accent2 : c.dim, fontSize: 9, fontWeight: '700' }}>
                    Heightmap ▾
                  </Text>
                </Pressable>
                {hmOpen && (
                  <View style={[styles.hmMenu, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
                    {HM_MENU.map(([key, label, dot]) => (
                      <Pressable key={key} onPress={() => setHmSel((p) => ({ ...p, [key]: !p[key] }))}
                        style={styles.hmMenuRow}>
                        <View style={[styles.hmMenuDot, { backgroundColor: dot, opacity: hmSel[key] ? 1 : 0.3 }]} />
                        <Text style={{ color: hmSel[key] ? c.text : c.dim, fontSize: 10, flex: 1 }}>{label}</Text>
                        <Text style={{ color: hmSel[key] ? c.accent2 : c.line, fontSize: 10, fontWeight: '700' }}>
                          {hmSel[key] ? '✓' : ''}
                        </Text>
                      </Pressable>
                    ))}
                  </View>
                )}
              </>
            )}
            {vw === 'split' && !compact && <PaneToggle pane={pane} onChange={setPane} />}
          </View>
        );
        const dataEl = (
          <View style={{ flex: 1 }}>
            <BlackBoxDataPanel sess={sess} frame={frame} topPad={!compact} />
            {vw === 'split' && !compact && <PaneToggle pane={pane} onChange={setPane} />}
          </View>
        );
        const videoEl = (
          <View style={[styles.videoPane, { borderColor: c.line, borderRadius: radius.md }]}>
            <BlackBoxVideo sess={sess} frame={frame} playing={playing} speed={speed}
                           ip={ip} date={date} session={session} onRetry={retryClips} />
          </View>
        );
        const logEl = (
          <View style={[styles.logPanel, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
          <View style={[styles.logHead, { borderBottomColor: c.line2 }]}>
            {vw !== 'quad' && (
            <Text style={{ color: c.dim, fontSize: 9, fontWeight: '700', letterSpacing: 0.5 }} numberOfLines={1}>
              {compact ? 'SYSTEM LOG' : <>{t('SYSTEM LOG · 재생 위치 동기 ')}<Text style={{ fontWeight: '400' }}>{t('— 탭하면 그 시점으로 이동')}</Text></>}
            </Text>
            )}
            <View style={{ flexDirection: 'row', gap: 4, marginLeft: 'auto' }}>
              {Object.keys(LEVEL_HEX).map((lv) => {
                const off = levelOff.has(lv);
                return (
                  <Pressable key={lv} onPress={() => setLevelOff((prev) => {
                    const next = new Set(prev); if (off) next.delete(lv); else next.add(lv); return next;
                  })}
                    style={[styles.lvChip, { borderColor: off ? c.line : LEVEL_HEX[lv], opacity: off ? 0.4 : 1 }]}>
                    <Text style={{ color: off ? c.dim : LEVEL_HEX[lv], fontSize: 8, fontWeight: '700' }}>{lv.slice(0, 3)}</Text>
                  </Pressable>
                );
              })}
            </View>
          </View>
          {shownLogs.length === 0 ? (
            <View style={styles.center}><Text style={{ color: c.dim, fontSize: 11 }}>{sess.logs.length === 0 ? t('이 세션에 로그가 없습니다.') : t('필터에 걸린 로그가 없습니다.')}</Text></View>
          ) : (
            <View style={{ flex: 1 }} onLayout={(e) => setLogW(e.nativeEvent.layout.width)}>
            <ScrollView horizontal showsHorizontalScrollIndicator persistentScrollbar
                        contentContainerStyle={{ width: Math.max(logW, LOG_ROW_MIN_W) }}>
            <FlatList
              style={{ width: Math.max(logW, LOG_ROW_MIN_W) }}
              ref={logScrollRef}
              data={shownLogs}
              keyExtractor={(_, i) => String(i)}
              contentContainerStyle={{ padding: 8 }}
              getItemLayout={(_, i) => ({ length: LINE_H, offset: LINE_H * i, index: i })}
              initialNumToRender={30}
              windowSize={9}
              removeClippedSubviews
              extraData={hotLog}
              renderItem={({ item: ln, index: i }) => (
                <Pressable onPress={() => seekFrame((ln.epochMs - sess.startEpochMs) / sess.tickMs)}
                  style={[styles.ln, i === hotLog && { backgroundColor: 'rgba(77,156,245,0.12)', borderLeftWidth: 2, borderLeftColor: c.accent2 }]}>
                  <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 10 }}>{ln.ts}</Text>
                  <Text style={{ color: c.accent2, fontFamily: fonts.mono, fontSize: 10, width: 78 }} numberOfLines={1}>[{ln.process}]</Text>
                  <Text style={{ color: LEVEL_HEX[ln.level] ?? c.muted, fontFamily: fonts.mono, fontSize: 10, width: 56 }}>{ln.level}</Text>
                  <Text style={{ color: c.text, fontFamily: fonts.mono, fontSize: 10, flex: 1 }} numberOfLines={1}>{ln.msg}</Text>
                </Pressable>
              )}
            />
            </ScrollView>
            </View>
          )}
        </View>
        );
        const chipsEl = (
          <View style={styles.chipRow}>
            <ChipBox k="ROLL" v={`${r2d(pose!.rpy[0]).toFixed(1)}°`} />
            <ChipBox k="PITCH" v={`${r2d(pose!.rpy[1]).toFixed(1)}°`} />
            <ChipBox k="YAW" v={`${r2d(pose!.rpy[2]).toFixed(1)}°`} />
            <ChipBox k="GAIT" v={gaitName(chan(sess, frame, 'status.gait_id'))} color={c.greenTx} />
            <ChipBox k="FALL" v={isFall ? 'YES' : 'no'} color={isFall ? c.redTx : undefined} />
            <ChipBox k="BAT L" v={`${chan(sess, frame, 'pdu.bat.left.voltage').toFixed(1)}V`} />
            <ChipBox k="BAT R" v={`${chan(sess, frame, 'pdu.bat.right.voltage').toFixed(1)}V`} />
            <ChipBox k="CMD VX" v={chan(sess, frame, 'cmd.vel_x').toFixed(2)} />
            <ChipBox k="JOY L" v={`${chan(sess, frame, 'joy.l_rl').toFixed(2)},${chan(sess, frame, 'joy.l_ud').toFixed(2)}`} />
          </View>
        );
        if (compact) {
          return (
            <View style={{ flex: 1, flexDirection: 'row', gap: 6, paddingHorizontal: 8 }}>
              <View style={{ flex: cLog ? 44 : 100 }}>
                {cPane === '3d' ? poseEl : dataEl}
                {cPane === '3d' && <View style={styles.chipOverlay} pointerEvents="none">{chipsEl}</View>}
                {sess.sync.warn && (
                  <View pointerEvents="none" style={[styles.syncWarnC, { borderColor: 'rgba(251,186,22,0.55)', backgroundColor: 'rgba(251,186,22,0.85)' }]}>
                    <Text style={{ color: '#222', fontSize: 9, fontWeight: '700' }}>⚠ {sess.sync.reason}</Text>
                  </View>
                )}
              </View>
              {cLog && <View style={{ flex: 56 }}>{logEl}</View>}
            </View>
          );
        }
        if (vw === 'quad') {
          return (
            <View style={{ flex: 1, flexDirection: 'row', gap: 8, paddingHorizontal: 14 }}>
              <View style={{ flex: 1, gap: 8 }}>
                <View style={{ flex: 1 }}>{dataEl}</View>
                <View style={{ flex: 1 }}>{logEl}</View>
              </View>
              <View style={{ flex: 1, gap: 8 }}>
                <View style={{ flex: 1 }}>{videoEl}</View>
                <View style={{ flex: 1 }}>{poseEl}</View>
              </View>
            </View>
          );
        }
        const solo: Pane = vw === 'split' ? pane : (vw === 'log' ? '3d' : vw);
        return (
          <View style={{ flex: 1, flexDirection: 'row', gap: 8, paddingHorizontal: 14 }}>
            {vw !== 'log' && (
              <View style={{ width: vw === 'split' ? '44%' : '100%', gap: 6 }}>
                {solo === '3d' ? poseEl : solo === 'video' ? videoEl : dataEl}
                {chipsEl}
              </View>
            )}
            {(vw === 'split' || vw === 'log') && logEl}
          </View>
        );
      })()}

      <View style={[styles.playbar, compact && styles.playbarC, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }]}>
        <Tappable onPress={() => { setPlaying(false); seekFrame(frame - 1); }}
          style={[styles.stepBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Icon name="prev" size={13} color={c.muted} />
        </Tappable>
        <Tappable onPress={togglePlay}
          style={[styles.playBtn, { backgroundColor: 'rgba(77,156,245,0.14)', borderColor: 'rgba(77,156,245,0.5)' }]}>
          <Icon name={playing ? 'pause' : 'play2'} size={15} color={c.accent2} />
        </Tappable>
        <Tappable onPress={() => { setPlaying(false); seekFrame(frame + 1); }}
          style={[styles.stepBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Icon name="next" size={13} color={c.muted} />
        </Tappable>
        <View style={{ flex: 1 }}>
          <Slider value={(frame / Math.max(1, sess.frameCount - 1)) * 100} width="100%"
            onChange={(pct) => { setPlaying(false); seekFrame((pct / 100) * (sess.frameCount - 1)); }} />
        </View>
        <Text style={{ color: c.text, fontFamily: fonts.mono, fontSize: 10.5 }}>{fmtClock(absMs)}</Text>
        <Text style={{ color: c.dim, fontFamily: fonts.mono, fontSize: 9 }}>F {frame + 1}/{sess.frameCount}</Text>
        <Tappable onPress={() => setSpeed(SPEEDS[(SPEEDS.indexOf(speed) + 1) % SPEEDS.length])}
          style={[styles.speedBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Text style={{ color: c.accent2, fontSize: 10.5, fontWeight: '700' }}>{speed}x</Text>
        </Tappable>
        {compact && (
          <>
            <Tappable onPress={() => setCPane((p) => (p === '3d' ? 'data' : '3d'))}
              style={[styles.speedBtn, { backgroundColor: c.elev, borderColor: c.line }]}>
              <Text style={{ color: c.text, fontSize: 10.5, fontWeight: '600' }}>{cPane === '3d' ? t('데이터') : '3D'}</Text>
            </Tappable>
            <Tappable onPress={() => setCLog((v) => !v)}
              style={[styles.speedBtn, { backgroundColor: cLog ? 'rgba(77,156,245,0.14)' : c.elev, borderColor: cLog ? 'rgba(77,156,245,0.5)' : c.line }]}>
              <Text style={{ color: cLog ? c.accent2 : c.text, fontSize: 10.5, fontWeight: '600' }}>{t('로그')}</Text>
            </Tappable>
          </>
        )}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  center: { flex: 1, alignItems: 'center', justifyContent: 'center', padding: 20 },
  head: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingHorizontal: 16, paddingVertical: 9 },
  pose: { flex: 1, borderWidth: 1, overflow: 'hidden' },
  refBtn: { position: 'absolute', top: 6, right: 6, paddingHorizontal: 8, paddingVertical: 3, borderRadius: 7, borderWidth: 1 },
  pcdBtn: { position: 'absolute', top: 6, right: 46, paddingHorizontal: 8, paddingVertical: 3, borderRadius: 7, borderWidth: 1 },
  hmMenu: { position: 'absolute', top: 30, right: 46, minWidth: 116, borderWidth: 1, paddingVertical: 3, zIndex: 10 },
  hmMenuRow: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 8, paddingVertical: 5 },
  hmMenuDot: { width: 8, height: 8, borderRadius: 4 },
  paneToggle: { position: 'absolute', top: 6, left: 6, flexDirection: 'row', gap: 4 },
  paneBtn: { paddingHorizontal: 8, paddingVertical: 3, borderRadius: 7, borderWidth: 1 },
  chipRow: { flexDirection: 'row', flexWrap: 'wrap', gap: 5 },
  chip: { paddingHorizontal: 9, paddingVertical: 5, borderRadius: 8, borderWidth: 1, minWidth: 58, gap: 1 },
  logPanel: { flex: 1, borderWidth: 1, overflow: 'hidden' },
  logHead: { paddingHorizontal: 10, paddingVertical: 7, borderBottomWidth: 1, flexDirection: 'row', alignItems: 'center' },
  lvChip: { paddingHorizontal: 5, paddingVertical: 2, borderRadius: 6, borderWidth: 1 },
  syncWarn: { marginLeft: 'auto', paddingHorizontal: 8, paddingVertical: 3, borderRadius: 7, borderWidth: 1 },
  viewBtn: { paddingHorizontal: 8, paddingVertical: 3, borderRadius: 7, borderWidth: 1 },
  chipOverlay: { position: 'absolute', left: 8, top: 8, width: 340 },
  syncWarnC: { position: 'absolute', left: 8, bottom: 8, paddingHorizontal: 7, paddingVertical: 3, borderRadius: 7, borderWidth: 1 },
  playbarC: { marginHorizontal: 8, marginVertical: 6, height: 42, gap: 8 },
  videoPane: { flex: 1, flexDirection: 'row', borderWidth: 1, overflow: 'hidden', backgroundColor: '#0c0e12' },
  ln: { flexDirection: 'row', gap: 7, height: LINE_H, alignItems: 'center', paddingLeft: 5 },
  playbar: {
    flexDirection: 'row', alignItems: 'center', gap: 10, height: 46,
    marginHorizontal: 14, marginVertical: 10, paddingHorizontal: 12, borderWidth: 1,
  },
  stepBtn: { width: 27, height: 27, borderRadius: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  playBtn: { width: 33, height: 33, borderRadius: 9, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  speedBtn: { height: 25, paddingHorizontal: 9, borderRadius: 8, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
});
