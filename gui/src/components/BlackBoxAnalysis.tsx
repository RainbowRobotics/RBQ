import { useMemo, useState, type ReactNode } from 'react';
import { View, Text, StyleSheet, ScrollView, Pressable, TextInput } from 'react-native';
import Svg, { Line, Path, Text as SvgText } from 'react-native-svg';
import { useTheme } from '@/theme';
import { chanOpt, type BlackboxSession } from '@/lib/blackbox';
import {
  minMaxBuckets, niceRange, presetCharts, presetCols, PRESETS, LEG_ORDER, CUSTOM_MAX, LINE_COLORS,
  type ChartSpec, type LegSel, type PresetKey,
} from '@/lib/bbSeries';
import { t } from '@/lib/i18n';

const CHART_H = 150;
const MIN_CHART_H = 72;
const HEAD_H = 30;
const MIN_CARD_W = 250;
const GAP = 8;
const PAD_L = 44, PAD_R = 8, PAD_T = 8, PAD_B = 18;
const RANGES: ReadonlyArray<readonly [string, number]> = [['전체', 0], ['20s', 20], ['5s', 5], ['1s', 1]];

function fmtTick(v: number, step: number) {
  const dp = Math.max(0, Math.min(4, -Math.floor(Math.log10(step || 1))));
  return v.toFixed(dp);
}

function fmtVal(v: number | undefined) {
  if (v === undefined || !Number.isFinite(v)) return '-';
  const a = Math.abs(v);
  return v.toFixed(a >= 100 ? 0 : a >= 10 ? 1 : 2);
}

function Chart({ spec, sess, f0, f1, frame, onSeek, width, height }: {
  spec: ChartSpec; sess: BlackboxSession; f0: number; f1: number; frame: number; onSeek: (f: number) => void;
  width?: number; height: number;
}) {
  const { c, fonts, radius } = useTheme();
  const [w, setW] = useState(0);
  const CHART_H = height;
  const pw = Math.max(1, w - PAD_L - PAD_R), ph = CHART_H - PAD_T - PAD_B;
  const buckets = Math.max(1, Math.floor(pw / 2));

  const data = useMemo(() => spec.lines.map((ln) => ({
    ln, mm: minMaxBuckets(sess, ln.name, f0, f1, buckets, ln.scale ?? 1),
  })), [spec, sess, f0, f1, buckets]);

  const { lo, hi, ticks } = useMemo(() => {
    let a = Infinity, b = -Infinity;
    for (const { mm } of data) {
      if (!mm) continue;
      for (let k = 0; k < mm.min.length; k++) {
        if (mm.min[k] < a) a = mm.min[k];
        if (mm.max[k] > b) b = mm.max[k];
      }
    }
    for (const l of spec.limits ?? []) { if (l < a) a = l; if (l > b) b = l; }
    return niceRange(a, b);
  }, [data, spec.limits]);

  const xOf = (k: number) => PAD_L + ((k + 0.5) / buckets) * pw;
  const yOf = (v: number) => PAD_T + ph - ((v - lo) / (hi - lo || 1)) * ph;
  const path = (mm: { min: Float32Array; max: Float32Array }) => {
    let d = '', pen = false;
    for (let k = 0; k < mm.min.length; k++) {
      if (Number.isNaN(mm.min[k])) { pen = false; continue; }
      const x = xOf(k).toFixed(1);
      d += `${pen ? 'L' : 'M'}${x} ${yOf(mm.min[k]).toFixed(1)} L${x} ${yOf(mm.max[k]).toFixed(1)} `;
      pen = true;
    }
    return d;
  };
  const missing = data.every((d) => !d.mm);
  const cursorX = frame >= f0 && frame <= f1 ? PAD_L + ((frame - f0) / Math.max(1, f1 - f0)) * pw : null;

  const seekAt = (x: number) => {
    const r = Math.min(1, Math.max(0, (x - PAD_L) / pw));
    onSeek(f0 + r * (f1 - f0));
  };

  return (
    <View style={[styles.card, { width: width ?? '100%' }, { backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.md }]}>
      <View style={styles.cardHead}>
        <Text style={{ color: c.text, fontSize: 11, fontWeight: '700', flexShrink: 0 }} numberOfLines={1}>{spec.title}</Text>
        {!!spec.unit && <Text style={{ color: c.dim, fontSize: 9, flexShrink: 0 }}>{spec.unit}</Text>}
        <View style={styles.legend}>
          {spec.lines.map((ln) => (
            <View key={ln.name} style={styles.legendItem}>
              <View style={{ width: 12, height: 0, borderTopWidth: 2, borderColor: ln.color, borderStyle: ln.dash ? 'dashed' : 'solid' }} />
              <Text style={{ color: c.muted, fontSize: 9, fontFamily: fonts.mono }} numberOfLines={1}>
                {ln.label} <Text style={{ color: c.text }}>{fmtVal((() => { const v = chanOpt(sess, frame, ln.name); return v === undefined ? v : v * (ln.scale ?? 1); })())}</Text>
              </Text>
            </View>
          ))}
        </View>
      </View>
      <View style={{ height: CHART_H }} onLayout={(e) => setW(e.nativeEvent.layout.width)}
        onStartShouldSetResponder={() => true}
        onResponderGrant={(e) => seekAt(e.nativeEvent.locationX)}
        onResponderMove={(e) => seekAt(e.nativeEvent.locationX)}>
        {w > 0 && (
          <Svg width={w} height={CHART_H} pointerEvents="none">
            {ticks.map((v) => (
              <Line key={`g${v}`} x1={PAD_L} x2={w - PAD_R} y1={yOf(v)} y2={yOf(v)}
                stroke={v === 0 ? c.line : c.line2} strokeWidth={1} />
            ))}
            {ticks.map((v) => (
              <SvgText key={`t${v}`} x={PAD_L - 4} y={yOf(v) + 3} fontSize={9} fill={c.dim} textAnchor="end">{fmtTick(v, ticks[1] - ticks[0])}</SvgText>
            ))}
            {(spec.limits ?? []).map((l) => (
              <Line key={`l${l}`} x1={PAD_L} x2={w - PAD_R} y1={yOf(l)} y2={yOf(l)}
                stroke={c.redTx} strokeWidth={1} strokeDasharray="4 3" opacity={0.7} />
            ))}
            {data.map(({ ln, mm }) => mm && (
              <Path key={ln.name} d={path(mm)} stroke={ln.color} strokeWidth={1.2} fill="none"
                strokeDasharray={ln.dash ? '4 3' : undefined} />
            ))}
            {cursorX !== null && (
              <Line x1={cursorX} x2={cursorX} y1={PAD_T} y2={PAD_T + ph} stroke={c.accent2} strokeWidth={1.5} />
            )}
            <SvgText x={PAD_L} y={CHART_H - 4} fontSize={9} fill={c.dim}>{(f0 * sess.tickMs / 1000).toFixed(1)}s</SvgText>
            <SvgText x={w - PAD_R} y={CHART_H - 4} fontSize={9} fill={c.dim} textAnchor="end">{(f1 * sess.tickMs / 1000).toFixed(1)}s</SvgText>
          </Svg>
        )}
        {missing && (
          <View style={[StyleSheet.absoluteFill, styles.center]} pointerEvents="none">
            <Text style={{ color: c.dim, fontSize: 10 }}>{t('이 세션에 없는 채널입니다')}</Text>
          </View>
        )}
      </View>
    </View>
  );
}

function Seg<T extends string | number>({ items, value, onChange }: {
  items: ReadonlyArray<readonly [T, string]>; value: T; onChange: (v: T) => void;
}) {
  const { c } = useTheme();
  return (
    <View style={styles.seg}>
      {items.map(([k, label]) => (
        <Pressable key={String(k)} onPress={() => onChange(k)}
          style={[styles.segBtn, {
            borderColor: value === k ? 'rgba(77,156,245,0.6)' : c.line,
            backgroundColor: value === k ? 'rgba(77,156,245,0.14)' : c.elev,
          }]}>
          <Text style={{ color: value === k ? c.accent2 : c.dim, fontSize: 9.5, fontWeight: '700' }} numberOfLines={1}>{label}</Text>
        </Pressable>
      ))}
    </View>
  );
}

function CustomPicker({ sess, picked, onChange }: { sess: BlackboxSession; picked: string[]; onChange: (p: string[]) => void }) {
  const { c, fonts } = useTheme();
  const [q, setQ] = useState('');
  const hits = useMemo(() => {
    const n = q.trim().toLowerCase();
    if (!n) return [];
    return [...sess.cols.keys()].filter((k) => k.toLowerCase().includes(n) && !picked.includes(k)).slice(0, 40);
  }, [sess, q, picked]);
  const full = picked.length >= CUSTOM_MAX;
  return (
    <View style={{ gap: 6 }}>
      <View style={styles.seg}>
        {picked.map((p, k) => (
          <Pressable key={p} onPress={() => onChange(picked.filter((x) => x !== p))}
            style={[styles.pick, { borderColor: LINE_COLORS[k % LINE_COLORS.length] }]}>
            <Text style={{ color: c.text, fontSize: 10, fontFamily: fonts.mono }} numberOfLines={1}>{p} ✕</Text>
          </Pressable>
        ))}
        <Text style={{ color: c.dim, fontSize: 9, alignSelf: 'center' }}>{picked.length}/{CUSTOM_MAX}</Text>
      </View>
      <TextInput value={q} onChangeText={setQ} editable={!full}
        placeholder={full ? t('최대 개수입니다 — 하나를 빼고 고르세요') : t('채널 검색 (예: motor.cur, joint, can)')}
        placeholderTextColor={c.dim} autoCapitalize="none" autoCorrect={false}
        style={[styles.search, { color: c.text, backgroundColor: c.elev, borderColor: c.line, fontFamily: fonts.mono }]} />
      {hits.length > 0 && (
        <View style={styles.seg}>
          {hits.map((h) => (
            <Pressable key={h} onPress={() => { onChange([...picked, h]); }}
              style={[styles.pick, { borderColor: c.line, backgroundColor: c.elev }]}>
              <Text style={{ color: c.muted, fontSize: 10, fontFamily: fonts.mono }} numberOfLines={1}>{h}</Text>
            </Pressable>
          ))}
        </View>
      )}
    </View>
  );
}

export type AnalysisSel = { preset: PresetKey; leg: LegSel; rangeS: number; custom: string[] };
export const ANALYSIS_DEFAULT: AnalysisSel = { preset: 'overview', leg: 'ALL', rangeS: 0, custom: [] };

const POSE_MIN_W = 260, POSE_MAX_W = 420;
const SIDE_MIN_W = 900;
const TOP_POSE_H = 240;

export function BlackBoxAnalysis({ sess, frame, onSeek, sel, onSel, pose }: {
  sess: BlackboxSession; frame: number; onSeek: (f: number) => void;
  sel: AnalysisSel; onSel: (s: AnalysisSel) => void;
  pose?: ReactNode;
}) {
  const { c } = useTheme();
  const { preset, leg, rangeS, custom } = sel;
  const set = (p: Partial<AnalysisSel>) => onSel({ ...sel, ...p });
  const [w, setW] = useState(0);
  const [viewH, setViewH] = useState(0);
  const [body, setBody] = useState({ w: 0, y: 0 });
  const [gridY, setGridY] = useState(0);
  const perLeg = PRESETS.find((p) => p.key === preset)!.perLeg;
  const charts = useMemo(() => presetCharts(preset, leg, custom), [preset, leg, custom]);

  const last = sess.frameCount - 1;
  let f0 = 0, f1 = last;
  if (rangeS > 0) {
    const half = (rangeS * 1000) / sess.tickMs / 2;
    f0 = Math.max(0, Math.min(frame - half, last - 2 * half));
    f1 = Math.min(last, f0 + 2 * half);
  }
  f0 = Math.round(f0); f1 = Math.round(f1);

  const want = Math.min(presetCols(preset), charts.length || 1);
  const cols = w > 0 ? Math.max(1, Math.min(want, Math.floor((w + GAP) / (MIN_CARD_W + GAP)))) : 1;
  const cardW = w > 0 && cols > 1 ? Math.floor((w - GAP * (cols - 1)) / cols) : undefined;
  const rows = Math.ceil(charts.length / cols);
  const side = !!pose && body.w >= SIDE_MIN_W;
  const poseW = Math.round(Math.max(POSE_MIN_W, Math.min(POSE_MAX_W, body.w * 0.3)));
  const availH = viewH - body.y - 10;
  const fit = viewH > 0 && rows > 1 ? Math.floor((availH - gridY - GAP * (rows - 1)) / rows) - HEAD_H : CHART_H;
  const h = Math.max(MIN_CHART_H, Math.min(CHART_H, fit));

  return (
    <ScrollView style={{ flex: 1 }} contentContainerStyle={{ paddingHorizontal: 14, paddingBottom: 10, gap: 8 }}
      onLayout={(e) => setViewH(e.nativeEvent.layout.height)}>
      <View style={styles.bar}>
        <Seg items={PRESETS.map((p) => [p.key, t(p.label)] as const)} value={preset} onChange={(v) => set({ preset: v })} />
        {perLeg && <Seg items={[['ALL', t('4다리')] as const, ...LEG_ORDER.map((l) => [l, l] as const)]} value={leg} onChange={(v) => set({ leg: v })} />}
        <View style={{ marginLeft: 'auto' }}>
          <Seg items={RANGES.map(([l, s]) => [s, t(l)] as const)} value={rangeS} onChange={(v) => set({ rangeS: v })} />
        </View>
      </View>
      {preset === 'custom' && <CustomPicker sess={sess} picked={custom} onChange={(p) => set({ custom: p })} />}
      <View style={{ flexDirection: side ? 'row' : 'column', gap: GAP, alignItems: 'flex-start' }}
        onLayout={(e) => setBody({ w: e.nativeEvent.layout.width, y: e.nativeEvent.layout.y })}>
        {!!pose && (
          <View style={{ width: side ? poseW : '100%', height: side ? Math.max(TOP_POSE_H, availH) : TOP_POSE_H }}>{pose}</View>
        )}
        <View style={[styles.grid, side ? { flex: 1, minWidth: 0 } : { width: '100%' }]}
          onLayout={(e) => { setW(e.nativeEvent.layout.width); setGridY(side ? 0 : e.nativeEvent.layout.y); }}>
          {charts.length === 0 ? (
            <Text style={{ color: c.dim, fontSize: 11, marginTop: 8 }}>{t('그릴 채널을 골라 주세요.')}</Text>
          ) : charts.map((spec) => (
            <Chart key={`${preset}:${spec.title}`} spec={spec} sess={sess} f0={f0} f1={f1} frame={frame} onSeek={onSeek}
              width={cardW} height={h} />
          ))}
        </View>
      </View>
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  bar: { flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap', gap: 6 },
  grid: { flexDirection: 'row', flexWrap: 'wrap', gap: GAP },
  seg: { flexDirection: 'row', flexWrap: 'wrap', gap: 4 },
  segBtn: { height: 25, paddingHorizontal: 9, justifyContent: 'center', borderRadius: 7, borderWidth: 1 },
  card: { borderWidth: 1, paddingTop: 6, paddingHorizontal: 4 },
  cardHead: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 6 },
  legend: { flexDirection: 'row', flexWrap: 'wrap', justifyContent: 'flex-end', columnGap: 8, marginLeft: 'auto', flexShrink: 1 },
  legendItem: { flexDirection: 'row', alignItems: 'center', gap: 3 },
  center: { alignItems: 'center', justifyContent: 'center' },
  search: { borderWidth: 1, borderRadius: 7, paddingHorizontal: 8, paddingVertical: 4, fontSize: 11 },
  pick: { borderWidth: 1, borderRadius: 7, paddingHorizontal: 7, paddingVertical: 3, maxWidth: 260 },
});
