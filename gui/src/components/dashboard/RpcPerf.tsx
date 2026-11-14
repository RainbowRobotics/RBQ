import { useEffect, useMemo, useRef, useState } from 'react';
import { View, Text, type LayoutChangeEvent, type TextStyle } from 'react-native';
import { LinearGradient } from 'expo-linear-gradient';
import Svg, { Line, Path, Rect, Text as SvgText } from 'react-native-svg';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Segmented } from '@/components/ui/controls';
import { useDashType } from '@/lib/dashType';
import { niceTicks } from '@/lib/ptzQc';
import { rest, HttpError } from '@/lib/rest';
import { t } from '@/lib/i18n';
import type { SystemStats, SystemStatsSample } from '@/types/robot';

type Metric = 'cpu' | 'temp' | 'power' | 'mhz';
type HeatMode = 'cpu' | 'mhz' | 'temp';
type Layout = SystemStats['layout'];
const liveMhz = (s: SystemStatsSample, lay: Layout | null) => (lay?.offline?.length ? s.mhz.filter((_, k) => !lay.offline!.includes(k)) : s.mhz);
function tempLimits(lay: Layout | null) {
  const crit = lay?.temp_crit ?? 100;
  const high = lay?.temp_high != null && lay.temp_high < crit ? lay.temp_high : crit - 10;
  return { high, crit };
}

const NUM = (size: number, color: string, weight: TextStyle['fontWeight'] = '600'): TextStyle =>
  ({ fontSize: size, color, fontWeight: weight, fontVariant: ['tabular-nums'] });
const avg = (a: number[]) => (a.length ? a.reduce((x, y) => x + y, 0) / a.length : 0);
const clock = (sec: number) => {
  const d = new Date(sec * 1000), p = (x: number) => String(x).padStart(2, '0');
  return `${p(d.getHours())}:${p(d.getMinutes())}:${p(d.getSeconds())}`;
};

type SeriesDef = { name: string; get: (s: SystemStatsSample) => number | undefined; dashed?: boolean; color?: string };
const coreGet = (m: Metric, k: number): ((s: SystemStatsSample) => number | undefined) | null =>
  m === 'cpu' ? (s) => s.core[k] : m === 'temp' ? (s) => s.ctemp[k] : m === 'mhz' ? (s) => s.mhz[k] : null;
const CORE_COLORS = ['#E69F00', '#009E73', '#CC79A7', '#D55E00', '#999933'];
const coreColor = (slot: number) => CORE_COLORS[slot % CORE_COLORS.length];
type Pick = { k: number; slot: number };
const freeSlot = (sel: Pick[]) => {
  const free = CORE_COLORS.findIndex((_, i) => !sel.some((p) => p.slot === i));
  return free >= 0 ? free : sel.length;
};
function seriesOf(m: Metric, lay: Layout | null, sel: Pick[] = []): { unit: string; yMax?: number; yTicks?: number; lines: SeriesDef[]; extra?: SeriesDef[] } {
  const base = baseSeries(m, lay);
  const cores = sel.flatMap(({ k, slot }) => {
    const get = coreGet(m, k);
    return get ? [{ name: `CPU${k}`, get, color: coreColor(slot) }] : [];
  });
  return { ...base, lines: [...base.lines, ...cores] };
}
function baseSeries(m: Metric, lay: Layout | null): { unit: string; yMax?: number; yTicks?: number; lines: SeriesDef[]; extra?: SeriesDef[] } {
  const rt = lay?.isolated ?? [];
  switch (m) {
    case 'cpu': return { unit: '%', yMax: 100, lines: [
      { name: t('전체 CPU'), get: (s) => s.cpu },
      { name: t('RT 코어 평균'), get: (s) => (rt.length ? avg(rt.map((k) => s.core[k] ?? 0)) : undefined), dashed: true },
    ] };
    case 'temp': return { unit: '°C', yMax: Math.ceil(Math.max(100, tempLimits(lay).crit) / 20) * 20, yTicks: 6, lines: [
      { name: 'Package', get: (s) => s.temp },
      { name: t('가장 뜨거운 코어'), get: (s) => Math.max(...s.ctemp), dashed: true },
    ] };
    case 'power': return { unit: 'W', lines: [
      { name: 'Package', get: (s) => s.pw.pkg },
      { name: t('플랫폼 (psys)'), get: (s) => s.pw.psys, dashed: true },
    ], extra: [{ name: 'Core', get: (s) => s.pw.core }, { name: 'Uncore', get: (s) => s.pw.uncore }] };
    case 'mhz': return { unit: 'MHz', lines: [{ name: t('평균'), get: (s) => avg(liveMhz(s, lay)) }],
      extra: [{ name: t('최저'), get: (s) => Math.min(...liveMhz(s, lay)) }, { name: t('최고'), get: (s) => Math.max(...liveMhz(s, lay)) }] };
  }
}

function throttleSpans(ss: SystemStatsSample[]): { a: number; b: number }[] {
  const out: { a: number; b: number }[] = [];
  for (let i = 1; i < ss.length; i++) {
    if (ss[i].thr <= ss[i - 1].thr) continue;
    const last = out[out.length - 1];
    if (last && ss[i].t - last.b <= 2) last.b = ss[i].t; else out.push({ a: ss[i - 1].t, b: ss[i].t });
  }
  return out;
}

function spansWhere(ss: SystemStatsSample[], ok: (s: SystemStatsSample) => boolean) {
  const out: { a: number; b: number; i0: number; i1: number }[] = [];
  ss.forEach((s, i) => {
    if (!ok(s)) return;
    const last = out[out.length - 1];
    if (last && s.t - last.b <= 10) { last.b = s.t; last.i1 = i; } else out.push({ a: s.t, b: s.t, i0: i, i1: i });
  });
  return out.filter((sp) => sp.b - sp.a + 1 >= 5);
}

function Panel({ icon, title, right, children }: { icon: IconName; title: string; right?: React.ReactNode; children: React.ReactNode }) {
  const { c, radius } = useTheme();
  const ty = useDashType();
  return (
    <LinearGradient colors={[c.cardA, c.cardB]} style={{ minWidth: 0, borderWidth: 1, borderColor: c.line, borderRadius: radius.lg, padding: 13 }}>
      <View style={{ flexDirection: 'row', flexWrap: 'wrap', alignItems: 'center', gap: 7, marginBottom: 8, minHeight: 28 }}>
        <Icon name={icon} size={16} color={c.accent2} />
        <Text style={{ color: c.muted, fontSize: ty.title, fontWeight: '700' }}>{title}</Text>
        {right && <View style={{ marginLeft: 'auto' }}>{right}</View>}
      </View>
      {children}
    </LinearGradient>
  );
}

function Spark({ values, color, w, h }: { values: number[]; color: string; w: number; h: number }) {
  if (values.length < 2 || w <= 0) return <View style={{ height: h }} />;
  let lo = Math.min(...values), hi = Math.max(...values);
  if (hi - lo < 1e-6) { lo -= 1; hi += 1; }
  const d = values.map((v, i) => `${i ? 'L' : 'M'}${((i / (values.length - 1)) * w).toFixed(1)} ${(h - 2 - ((v - lo) / (hi - lo)) * (h - 4)).toFixed(1)}`).join(' ');
  return <Svg width={w} height={h}><Path d={d} stroke={color} strokeWidth={1.5} fill="none" strokeLinejoin="round" /></Svg>;
}

function Tile({ label, value, unit, sub, spark, tone, toneText, active, onPress }: {
  label: string; value: string; unit: string; sub: string; spark: number[];
  tone?: 'amber' | 'red'; toneText?: string; active: boolean; onPress: () => void;
}) {
  const { c, radius } = useTheme();
  const ty = useDashType();
  const [w, setW] = useState(0);
  const valColor = tone === 'red' ? c.redTx : tone === 'amber' ? c.amberTx : c.text;
  return (
    <Tappable onPress={onPress} style={{ flex: 1, minWidth: 150 }}>
      <View onLayout={(e: LayoutChangeEvent) => setW(e.nativeEvent.layout.width - 24)}
        style={{ borderWidth: 1, borderRadius: radius.md, padding: 12, gap: 2,
          borderColor: active ? c.accent : c.line, backgroundColor: active ? c.panel2 : c.panel }}>
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
          <Text style={{ color: c.muted, fontSize: ty.label, fontWeight: '700' }}>{label}</Text>
          {tone && (
            <View style={{ marginLeft: 'auto', flexDirection: 'row', alignItems: 'center', gap: 3 }}>
              <Icon name="warn" size={12} color={valColor} />
              <Text style={{ color: valColor, fontSize: ty.badge, fontWeight: '700' }}>{toneText}</Text>
            </View>
          )}
        </View>
        <View style={{ flexDirection: 'row', alignItems: 'baseline', gap: 4 }}>
          <Text style={NUM(ty.hero * 0.8, valColor, '600')}>{value}</Text>
          <Text style={{ color: c.muted, fontSize: ty.heroSub, fontWeight: '500' }}>{unit}</Text>
        </View>
        <Spark values={spark} color={active ? c.accent : c.muted} w={w} h={26} />
        <Text style={NUM(ty.caption, c.muted, '500')}>{sub}</Text>
      </View>
    </Tappable>
  );
}

function TimeChart({ ss, metric, lay, sel, fill = 0 }: { ss: SystemStatsSample[]; metric: Metric; lay: Layout | null; sel: Pick[]; fill?: number }) {
  const { c, fonts } = useTheme();
  const ty = useDashType();
  const [w, setW] = useState(0);
  const [hover, setHover] = useState<number | null>(null);
  const [legH, setLegH] = useState(0);
  const [labW, setLabW] = useState(0);
  const H = Math.max(220, Math.floor(fill - legH - 4)), padT = 14, padB = 22;
  const def = seriesOf(metric, lay, sel);
  const stroke = (l: SeriesDef) => l.color ?? (l.dashed ? c.muted : c.accent);
  const ph = H - padT - padB;
  const band = metric === 'mhz' ? { lo: ss.map((s) => Math.min(...liveMhz(s, lay))), hi: ss.map((s) => Math.max(...liveMhz(s, lay))) } : null;
  const ref = metric === 'temp' ? { v: tempLimits(lay).high, label: `${t('경고')} ${tempLimits(lay).high} °C`, warn: true }
    : metric === 'mhz' && lay?.max_mhz ? { v: lay.max_mhz, label: `${t('최대')} ${lay.max_mhz} MHz`, warn: false } : null;
  const vals = def.lines.flatMap((l) => ss.map(l.get)).filter((v): v is number => v != null && isFinite(v));
  const top = def.yMax ?? Math.max(...(vals.length ? vals : [1]), ...(band ? band.hi : []), ref?.v ?? 0) * 1.08;
  const yt = niceTicks(top, def.yTicks ?? 4);
  const yTop = yt[yt.length - 1] || 1;
  const padR = 2;
  const widest = yt.map(String).reduce((a, b) => (b.length > a.length ? b : a), '');
  const padL = padR + (labW || Math.ceil(widest.length * ty.badge * 0.6)) + 6;
  const pw = Math.max(1, w - padL - padR);
  const n = ss.length;
  const xOf = (i: number) => padL + (n > 1 ? (i / (n - 1)) * pw : pw);
  const yOf = (v: number) => padT + ph - (Math.min(v, yTop) / yTop) * ph;
  const pathOf = (get: (s: SystemStatsSample) => number | undefined) => {
    let d = '', pen = false;
    ss.forEach((s, i) => {
      const v = get(s);
      if (v == null || !isFinite(v)) { pen = false; return; }
      d += `${pen ? 'L' : 'M'}${xOf(i).toFixed(1)} ${yOf(v).toFixed(1)} `;
      pen = true;
    });
    return d;
  };
  const bandPath = band && n > 1
    ? band.hi.map((v, i) => `${i ? 'L' : 'M'}${xOf(i).toFixed(1)} ${yOf(v).toFixed(1)}`).join(' ')
      + ' ' + [...band.lo].map((v, i) => [i, v] as const).reverse().map(([i, v]) => `L${xOf(i).toFixed(1)} ${yOf(v).toFixed(1)}`).join(' ') + ' Z'
    : '';
  const t0 = ss[0]?.t ?? 0;
  const xT = (sec: number) => xOf(sec - t0);
  const spans = throttleSpans(ss);
  const span = n > 1 ? ss[n - 1].t - t0 : 0;
  const step = span <= 90 ? 15 : span <= 330 ? 60 : 120;
  const xt: { x: number; label: string }[] = [];
  for (let s = 0; s <= span; s += step) xt.push({ x: xOf(n - 1 - s), label: s === 0 ? t('지금') : step < 60 ? `-${s}s` : `-${s / 60}m` });
  const pick = (x: number) => {
    if (n < 2) return;
    setHover(Math.max(0, Math.min(n - 1, Math.round(((x - padL) / pw) * (n - 1)))));
  };
  const hs = hover != null ? ss[hover] : null;
  const tipLines = hs ? [...def.lines, ...(def.extra ?? [])].map((l) => ({ name: l.name, v: l.get(hs) })) : [];
  const hx = hover != null ? xOf(hover) : 0;
  const tipLeft = hx > padL + pw / 2;
  const fmt = (v?: number) => (v == null ? '—' : metric === 'mhz' ? String(Math.round(v)) : v.toFixed(1));
  return (
    <View>
      <Text onLayout={(e: LayoutChangeEvent) => setLabW(Math.ceil(e.nativeEvent.layout.width))} pointerEvents="none"
        style={{ position: 'absolute', opacity: 0, fontFamily: fonts.ui, fontSize: ty.badge }}>{widest}</Text>
      <View onLayout={(e: LayoutChangeEvent) => setLegH(Math.ceil(e.nativeEvent.layout.height))}
        style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 14, marginBottom: 4 }}>
        {def.lines.map((l) => (
          <View key={l.name} style={{ flexDirection: 'row', alignItems: 'center', gap: 6 }}>
            <Svg width={18} height={6}><Line x1={0} y1={3} x2={18} y2={3} stroke={stroke(l)} strokeWidth={2} strokeDasharray={l.dashed ? '4 3' : undefined} /></Svg>
            <Text style={{ color: c.muted, fontSize: ty.caption, fontWeight: '500' }}>{l.name}</Text>
            <Text style={NUM(ty.caption, c.text, '600')}>{n ? `${fmt(l.get(ss[n - 1]))} ${def.unit}` : '—'}</Text>
          </View>
        ))}
        {band && <Text style={{ color: c.muted, fontSize: ty.caption }}>{t('띠 = 스레드 최저~최고')}</Text>}
      </View>
      <View
        onLayout={(e: LayoutChangeEvent) => setW(e.nativeEvent.layout.width)}
        onStartShouldSetResponder={() => true}
        onResponderGrant={(e) => pick(e.nativeEvent.locationX)}
        onResponderMove={(e) => pick(e.nativeEvent.locationX)}
        onResponderRelease={() => setHover(null)}
        {...({ onMouseMove: (e: any) => pick(e.nativeEvent.offsetX ?? e.nativeEvent.locationX), onMouseLeave: () => setHover(null) } as object)}
        style={{ height: H }}>
        {w > 0 && (
          <Svg width={w} height={H}>
            {yt.map((v) => (
              <Line key={`g${v}`} x1={padL} x2={padL + pw} y1={yOf(v)} y2={yOf(v)} stroke={c.line} strokeWidth={1} />
            ))}
            {yt.map((v) => (
              <SvgText fontFamily={fonts.ui} key={`y${v}`} x={padL - 6} y={yOf(v) + 4} fontSize={ty.badge} fill={c.muted} textAnchor="end">{String(v)}</SvgText>
            ))}
            {xt.map((g) => (
              <SvgText fontFamily={fonts.ui} key={`x${g.label}`} x={g.x} y={H - 6} fontSize={ty.badge} fill={c.muted} textAnchor={g.label === t('지금') ? 'end' : 'middle'}>{g.label}</SvgText>
            ))}
            {spans.map((sp) => (
              <Rect key={`s${sp.a}`} x={xT(sp.a)} y={padT} width={Math.max(2, xT(sp.b) - xT(sp.a))} height={ph} fill={c.red} fillOpacity={0.12} />
            ))}
            {spans.filter((sp) => xT(sp.b) - xT(sp.a) > 34).map((sp) => (
              <SvgText fontFamily={fonts.ui} key={`sl${sp.a}`} x={(xT(sp.a) + xT(sp.b)) / 2} y={padT + 11} fontSize={ty.badge} fill={c.redTx} textAnchor="middle" fontWeight="700">{t('스로틀')}</SvgText>
            ))}
            {bandPath ? <Path d={bandPath} fill={c.accent} fillOpacity={0.14} /> : null}
            {ref && (
              <>
                <Line x1={padL} x2={padL + pw} y1={yOf(ref.v)} y2={yOf(ref.v)} stroke={ref.warn ? c.amber : c.muted} strokeWidth={1} strokeDasharray="5 4" />
                <SvgText fontFamily={fonts.ui} x={padL + 4} y={yOf(ref.v) - 4} fontSize={ty.badge} fill={ref.warn ? c.amberTx : c.muted}>{ref.label}</SvgText>
              </>
            )}
            {def.lines.map((l) => (
              <Path key={l.name} d={pathOf(l.get)} stroke={stroke(l)} strokeWidth={l.color ? 1.5 : 2} fill="none"
                strokeDasharray={l.dashed ? '5 4' : undefined} strokeLinejoin="round" />
            ))}
            {hs && (
              <>
                <Line x1={hx} x2={hx} y1={padT} y2={padT + ph} stroke={c.text} strokeOpacity={0.35} strokeWidth={1} />
                {def.lines.map((l) => {
                  const v = l.get(hs);
                  return v == null ? null : <Rect key={`d${l.name}`} x={hx - 4} y={yOf(v) - 4} width={8} height={8} rx={4} fill={stroke(l)} stroke={c.cardA} strokeWidth={2} />;
                })}
              </>
            )}
          </Svg>
        )}
        {hs && (
          <View pointerEvents="none" style={{ position: 'absolute', top: padT, [tipLeft ? 'right' : 'left']: tipLeft ? w - hx + 10 : hx + 10,
            backgroundColor: c.panel, borderColor: c.line, borderWidth: 1, borderRadius: 8, paddingHorizontal: 9, paddingVertical: 6, gap: 2 }}>
            <Text style={NUM(ty.badge, c.muted, '600')}>{clock(hs.t)}</Text>
            {tipLines.map((l) => (
              <View key={l.name} style={{ flexDirection: 'row', gap: 10, justifyContent: 'space-between' }}>
                <Text style={{ color: c.muted, fontSize: ty.caption }}>{l.name}</Text>
                <Text style={NUM(ty.caption, c.text, '600')}>{`${fmt(l.v)} ${def.unit}`}</Text>
              </View>
            ))}
            {spans.some((sp) => hs.t >= sp.a && hs.t <= sp.b) && <Text style={{ color: c.redTx, fontSize: ty.caption, fontWeight: '700' }}>{t('스로틀')}</Text>}
          </View>
        )}
      </View>
    </View>
  );
}

function CoreHeatmap({ ss, lay, mode, scale }: { ss: SystemStatsSample[]; lay: Layout; mode: HeatMode; scale: Scale }) {
  const { c, fonts } = useTheme();
  const ty = useDashType();
  const [w, setW] = useState(0);
  const [hover, setHover] = useState<{ r: number; col: number } | null>(null);
  const roleOf = (k: number) => (Object.entries(lay.roles).find(([, v]) => v === k)?.[0]);
  const rt = lay.isolated;
  const rows = lay.types.map((_, k) => k).filter((k) => !lay.offline?.includes(k));
  const labelW = 124, rowH = 11, gap = 1, headH = 4;
  const cols = Math.max(1, Math.min(60, ss.length));
  const per = ss.length / cols;
  const cw = Math.max(1, (w - labelW) / cols);
  const pick = (s: SystemStatsSample, k: number) => (mode === 'cpu' ? s.core[k] : mode === 'mhz' ? s.mhz[k] : s.ctemp[k]) ?? 0;
  const lo = mode === 'temp' ? 40 : 0;
  const hi = mode === 'cpu' ? 100 : mode === 'mhz' ? (lay.max_mhz ?? 5000) : tempLimits(lay).crit;
  const unit = mode === 'cpu' ? '%' : mode === 'mhz' ? 'MHz' : '°C';
  const cell = (k: number, col: number) => {
    const vs = ss.slice(Math.floor(col * per), Math.max(Math.floor(col * per) + 1, Math.floor((col + 1) * per))).map((s) => pick(s, k));
    return mode === 'temp' ? Math.max(...vs) : avg(vs);
  };
  const gridH = rows.length * (rowH + gap) + headH;
  const yOfRow = (r: number) => headH + r * (rowH + gap);
  const onPick = (x: number, y: number) => {
    const col = Math.floor((x - labelW) / cw);
    let r = -1;
    rows.forEach((_, i) => { if (y >= yOfRow(i) && y < yOfRow(i) + rowH) r = i; });
    setHover(col >= 0 && col < cols && r >= 0 ? { r, col } : null);
  };
  const hv = hover ? { k: rows[hover.r], v: cell(rows[hover.r], hover.col), t: ss[Math.min(ss.length - 1, Math.floor(hover.col * per))]?.t } : null;
  return (
    <View>
      <View
        onLayout={(e: LayoutChangeEvent) => setW(e.nativeEvent.layout.width)}
        onStartShouldSetResponder={() => true}
        onResponderGrant={(e) => onPick(e.nativeEvent.locationX, e.nativeEvent.locationY)}
        onResponderMove={(e) => onPick(e.nativeEvent.locationX, e.nativeEvent.locationY)}
        onResponderRelease={() => setHover(null)}
        {...({ onMouseMove: (e: any) => onPick(e.nativeEvent.offsetX ?? e.nativeEvent.locationX, e.nativeEvent.offsetY ?? e.nativeEvent.locationY), onMouseLeave: () => setHover(null) } as object)}
        style={{ height: gridH }}>
        {w > 0 && ss.length > 0 && (
          <Svg width={w} height={gridH}>
            {rows.map((k, r) => {
              const role = roleOf(k);
              const isRt = rt.includes(k);
              return (
                <SvgText fontFamily={fonts.ui} key={`l${k}`} x={0} y={yOfRow(r) + rowH - 2} fontSize={ty.badge} fontWeight={isRt ? '700' : '400'} fill={isRt ? c.text : c.muted}>
                  {`CPU${k} ${lay.types[k] ?? ''}${role ? ` · ${role}` : ''}`}
                </SvgText>
              );
            })}
            {rows.map((k, r) => Array.from({ length: cols }, (_, col) => {
              const raw = cell(k, col);
              const v = Math.max(0, Math.min(1, (raw - lo) / (hi - lo)));
              return <Rect key={`${k}-${col}`} x={labelW + col * cw} y={yOfRow(r)} width={Math.max(1, cw - gap)} height={rowH} fill={scaleColor(scale, v)} />;
            }))}
            {hover && <Rect x={labelW + hover.col * cw - 0.5} y={yOfRow(hover.r) - 0.5} width={cw} height={rowH + 1} fill="none" stroke={c.text} strokeWidth={1} />}
          </Svg>
        )}
        {hv && (
          <View pointerEvents="none" style={{ position: 'absolute', right: 0, top: -30, backgroundColor: c.panel, borderColor: c.line, borderWidth: 1, borderRadius: 8, paddingHorizontal: 9, paddingVertical: 4 }}>
            <Text style={NUM(ty.caption, c.text, '600')}>{`CPU${hv.k} · ${hv.t ? clock(hv.t) : ''} · ${mode === 'mhz' ? Math.round(hv.v) : hv.v.toFixed(1)} ${unit}`}</Text>
          </View>
        )}
      </View>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 6, marginTop: 8, marginLeft: labelW }}>
        <Text style={NUM(ty.badge, c.muted, '500')}>{`${lo} ${unit}`}</Text>
        <Svg width={120} height={10}>
          {Array.from({ length: 40 }, (_, i) => <Rect key={i} x={i * 3} y={0} width={3.5} height={10} fill={scaleColor(scale, i / 39)} />)}
        </Svg>
        <Text style={NUM(ty.badge, c.muted, '500')}>{`${hi} ${unit}`}</Text>
      </View>
    </View>
  );
}

const YLORRD = ['#ffffcc', '#ffeda0', '#fed976', '#feb24c', '#fd8d3c', '#fc4e2a', '#e31a1c', '#bd0026', '#800026'];
const TURBO = ['#30123b', '#4145ab', '#4675ed', '#39a2fc', '#1bcfd4', '#24eca6', '#61fc6c', '#a4fc3b', '#d1e834', '#f3c63a', '#fe9b2d', '#f36315', '#d93806', '#b11901', '#7a0402'];
const toLin = (u: number) => (u <= 0.04045 ? u / 12.92 : ((u + 0.055) / 1.055) ** 2.4);
const toSrgb = (u: number) => (u <= 0.0031308 ? 12.92 * u : 1.055 * u ** (1 / 2.4) - 0.055);
function hexToLab(hex: string) {
  const [r, g, b] = [0, 1, 2].map((k) => toLin(parseInt(hex.slice(1 + k * 2, 3 + k * 2), 16) / 255));
  const l = Math.cbrt(0.4122214708 * r + 0.5363325363 * g + 0.0514459929 * b);
  const m = Math.cbrt(0.2119034982 * r + 0.6806995451 * g + 0.1073969566 * b);
  const s = Math.cbrt(0.0883024619 * r + 0.2817188376 * g + 0.6299787005 * b);
  return [0.2104542553 * l + 0.793617785 * m - 0.0040720468 * s, 1.9779984951 * l - 2.428592205 * m + 0.4505937099 * s, 0.0259040371 * l + 0.7827717662 * m - 0.808675766 * s];
}
function labToHex([L, A, B]: number[]) {
  const l = (L + 0.3963377774 * A + 0.2158037573 * B) ** 3, m = (L - 0.1055613458 * A - 0.0638541728 * B) ** 3, s = (L - 0.0894841775 * A - 1.291485548 * B) ** 3;
  const rgb = [4.0767416621 * l - 3.3077115913 * m + 0.2309699292 * s, -1.2684380046 * l + 2.6097574011 * m - 0.3413193965 * s, -0.0041960863 * l - 0.7034186147 * m + 1.707614701 * s];
  return `#${rgb.map((u) => Math.round(Math.max(0, Math.min(1, toSrgb(u))) * 255).toString(16).padStart(2, '0')).join('')}`;
}
function rampColor(stops: string[], v: number) {
  const x = Math.max(0, Math.min(1, v)) * (stops.length - 1);
  const i = Math.min(stops.length - 2, Math.floor(x)), f = x - i;
  const a = hexToLab(stops[i]), b = hexToLab(stops[i + 1]);
  return labToHex(a.map((u, k) => u + (b[k] - u) * f));
}
type Scale = 'turbo' | 'heat';
const RAMPS: Record<Scale, string[]> = { turbo: TURBO, heat: YLORRD };
const scaleColor = (scale: Scale, v: number) => rampColor(RAMPS[scale], v);

type Ev = { a: number; b: number; title: string; detail: string; tone: 'red' | 'amber' };
function eventsOf(ss: SystemStatsSample[], lay: Layout | null): Ev[] {
  const out: Ev[] = [];
  const dur = (a: number, b: number) => `${b - a + 1} s`;
  const peak = (i0: number, i1: number, f: (s: SystemStatsSample) => number) => Math.max(...ss.slice(i0, i1 + 1).map(f));
  const idx = (sec: number) => Math.max(0, ss.findIndex((s) => s.t >= sec));
  for (const sp of throttleSpans(ss)) {
    out.push({ a: sp.a, b: sp.b, tone: 'red', title: t('스로틀'), detail: `${dur(sp.a, sp.b)} · ${t('최고')} ${peak(idx(sp.a), idx(sp.b), (s) => s.temp).toFixed(1)} °C` });
  }
  const { high } = tempLimits(lay);
  for (const sp of spansWhere(ss, (s) => s.temp >= high)) {
    if (out.some((e) => e.tone === 'red' && sp.a <= e.b && sp.b >= e.a)) continue;
    out.push({ a: sp.a, b: sp.b, tone: 'amber', title: `${t('고온')} ≥ ${high} °C`, detail: `${dur(sp.a, sp.b)} · ${t('최고')} ${peak(sp.i0, sp.i1, (s) => s.temp).toFixed(1)} °C` });
  }
  for (const sp of spansWhere(ss, (s) => s.cpu >= 80)) {
    out.push({ a: sp.a, b: sp.b, tone: 'amber', title: `${t('CPU 부하')} ≥ 80 %`, detail: `${dur(sp.a, sp.b)} · ${t('최고')} ${peak(sp.i0, sp.i1, (s) => s.cpu).toFixed(0)} %` });
  }
  for (const sp of spansWhere(ss, (s) => s.swap >= 50)) {
    out.push({ a: sp.a, b: sp.b, tone: 'amber', title: `${t('스왑')} ≥ 50 %`, detail: dur(sp.a, sp.b) });
  }
  return out.sort((x, y) => y.a - x.a);
}

function EventList({ evs }: { evs: Ev[] }) {
  const { c } = useTheme();
  const ty = useDashType();
  if (!evs.length) return <Text style={{ color: c.muted, fontSize: ty.body }}>{t('최근 10분 동안 이상 없음')}</Text>;
  return (
    <View style={{ gap: 6 }}>
      {evs.slice(0, 12).map((e) => (
        <View key={`${e.title}${e.a}`} style={{ flexDirection: 'row', gap: 8, alignItems: 'flex-start' }}>
          <Icon name="warn" size={13} color={e.tone === 'red' ? c.redTx : c.amberTx} />
          <View style={{ flex: 1 }}>
            <View style={{ flexDirection: 'row', justifyContent: 'space-between', gap: 8 }}>
              <Text style={{ color: e.tone === 'red' ? c.redTx : c.amberTx, fontSize: ty.body, fontWeight: '700' }}>{e.title}</Text>
              <Text style={NUM(ty.caption, c.muted, '500')}>{clock(e.a)}</Text>
            </View>
            <Text style={NUM(ty.caption, c.muted, '500')}>{e.detail}</Text>
          </View>
        </View>
      ))}
    </View>
  );
}

function CorePicker({ lay, sel, onChange }: { lay: Layout; sel: Pick[]; onChange: (v: Pick[]) => void }) {
  const { c, radius } = useTheme();
  const ty = useDashType();
  const ks = lay.types.map((_, k) => k).filter((k) => !lay.offline?.includes(k));
  const chip = (key: string, label: string, on: boolean, onPress: () => void, color?: string, bold?: boolean) => (
    <Tappable key={key} onPress={onPress}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 5, paddingHorizontal: 8, paddingVertical: 3, borderRadius: radius.sm,
        borderWidth: 1, borderColor: on ? (color ?? c.accent) : c.line, backgroundColor: on ? c.panel2 : c.panel }}>
        {on && color && <View style={{ width: 8, height: 8, borderRadius: 4, backgroundColor: color }} />}
        <Text style={NUM(ty.badge, on || bold ? c.text : c.muted, bold ? '700' : '500')}>{label}</Text>
      </View>
    </Tappable>
  );
  return (
    <View style={{ flexDirection: 'row', flexWrap: 'wrap', alignItems: 'center', gap: 5, marginTop: 8 }}>
      <Text style={{ color: c.muted, fontSize: ty.caption, fontWeight: '600', marginRight: 3 }}>{t('코어 선택')}</Text>
      {ks.map((k) => {
        const p = sel.find((x) => x.k === k);
        return chip(`k${k}`, `CPU${k}${lay.types[k] ? ` ${lay.types[k]}` : ''}`, !!p,
          () => onChange(p ? sel.filter((x) => x.k !== k) : [...sel, { k, slot: freeSlot(sel) }]), p ? coreColor(p.slot) : undefined, lay.isolated.includes(k));
      })}
      {sel.length > 0 && chip('clear', t('선택 해제'), false, () => onChange([]))}
    </View>
  );
}

const HIDDEN = { position: 'absolute', top: 0, left: 0, right: 0, opacity: 0 } as const;

export type RpcStats = { lay: Layout | null; all: SystemStatsSample[]; old: boolean };

export function useSystemStats(ip: string, conn: string, on: boolean): RpcStats {
  const [lay, setLay] = useState<Layout | null>(null);
  const [all, setAll] = useState<SystemStatsSample[]>([]);
  const [old, setOld] = useState(false);
  const last = useRef(0);

  useEffect(() => {
    if (!on || conn !== 'connected') return;
    let dead = false;
    last.current = 0;
    setAll([]);
    const pull = () => rest.systemStats(ip, last.current || undefined)
      .then((r) => {
        if (dead || !r.samples.length) return;
        setLay(r.layout);
        last.current = r.samples[r.samples.length - 1].t;
        const cut = last.current - r.window_s;
        setAll((p) => [...p.filter((s) => s.t > cut && s.t < r.samples[0].t), ...r.samples]);
      })
      .catch((e) => { if (!dead && e instanceof HttpError && e.status === 404) setOld(true); });
    pull();
    const id = setInterval(pull, 1000);
    return () => { dead = true; clearInterval(id); };
  }, [ip, conn, on]);
  return { lay, all, old };
}

export function RpcEvents({ stats }: { stats: RpcStats }) {
  const evs = useMemo(() => eventsOf(stats.all, stats.lay), [stats.all, stats.lay]);
  return <EventList evs={evs} />;
}

export function RpcPerf({ stats }: { stats: RpcStats }) {
  const { c } = useTheme();
  const ty = useDashType();
  const { lay, all, old } = stats;
  const [metric, setMetric] = useState<Metric | null>(null);
  const [win, setWin] = useState<'60' | '300' | '600'>('300');
  const [view, setView] = useState<'trend' | 'core'>('trend');
  const [heat, setHeat] = useState<HeatMode>('temp');
  const [viewH, setViewH] = useState({ trend: 0, core: 0 });
  const measure = (k: 'trend' | 'core') => (e: LayoutChangeEvent) => {
    const h = Math.ceil(e.nativeEvent.layout.height);
    setViewH((o) => (o[k] === h ? o : { ...o, [k]: h }));
  };
  const [scale, setScale] = useState<Scale>('turbo');
  const [sel, setSel] = useState<Pick[]>([]);
  useEffect(() => {
    if (!lay) return;
    setSel((o) => {
      const live = o.filter((p) => p.k < lay.types.length && !lay.offline?.includes(p.k));
      return live.length === o.length ? o : live;
    });
  }, [lay]);

  const ss = useMemo(() => {
    const n = all.length;
    if (!n) return all;
    const from = all[n - 1].t - Number(win) + 1;
    return all.filter((s) => s.t >= from);
  }, [all, win]);

  if (old) {
    return (
      <Text style={{ color: c.muted, fontSize: ty.caption, marginHorizontal: 15, marginTop: 8 }}>
        {t('로봇 소프트웨어가 구버전이라 성능 기록을 보내지 않습니다.')}
      </Text>
    );
  }
  const cur = all[all.length - 1];
  const rt = lay?.isolated ?? [];
  const spark = (f: (s: SystemStatsSample) => number) => ss.map(f);
  const hiOf = (f: (s: SystemStatsSample) => number) => (ss.length ? Math.max(...ss.map(f)) : 0);
  const loOf = (f: (s: SystemStatsSample) => number) => (ss.length ? Math.min(...ss.map(f)) : 0);
  const recentThr = all.length > 10 && cur ? cur.thr - all[all.length - 11].thr : 0;
  const tempTone = !cur ? undefined : recentThr > 0 ? 'red' as const : cur.temp >= tempLimits(lay).high ? 'amber' as const : undefined;
  const avgMhz = (s: SystemStatsSample) => avg(liveMhz(s, lay));
  const tiles: { m: Metric; label: string; value: string; unit: string; sub: string; f: (s: SystemStatsSample) => number; tone?: 'amber' | 'red'; toneText?: string }[] = [
    { m: 'cpu', label: t('CPU 사용률'), value: cur ? cur.cpu.toFixed(0) : '—', unit: '%', f: (s) => s.cpu,
      sub: cur && rt.length ? `RT ${avg(rt.map((k) => cur.core[k] ?? 0)).toFixed(0)} % · ${t('최고')} ${hiOf((s) => s.cpu).toFixed(0)} %` : '—',
      tone: cur && cur.cpu >= 80 ? 'amber' : undefined, toneText: t('과부하') },
    { m: 'temp', label: t('CPU 온도'), value: cur ? cur.temp.toFixed(1) : '—', unit: '°C', f: (s) => s.temp,
      sub: ss.length ? `${loOf((s) => s.temp).toFixed(0)}–${hiOf((s) => s.temp).toFixed(0)} °C` : '—',
      tone: tempTone, toneText: tempTone === 'red' ? t('스로틀') : t('고온') },
    { m: 'power', label: t('CPU 전력'), value: cur?.pw.pkg != null ? cur.pw.pkg.toFixed(1) : '—', unit: 'W', f: (s) => s.pw.pkg ?? 0,
      sub: cur?.pw.psys != null ? `${t('플랫폼')} ${cur.pw.psys.toFixed(1)} W` : '—' },
    { m: 'mhz', label: t('CPU 클럭'), value: cur ? (avgMhz(cur) / 1000).toFixed(2) : '—', unit: 'GHz', f: avgMhz,
      sub: cur ? `${(Math.min(...liveMhz(cur, lay)) / 1000).toFixed(1)}–${(Math.max(...liveMhz(cur, lay)) / 1000).toFixed(1)} GHz` : '—' },
  ];
  const winOpts = [{ key: '60' as const, label: t('1분') }, { key: '300' as const, label: t('5분') }, { key: '600' as const, label: t('10분') }];
  const metricName = tiles.find((x) => x.m === metric)?.label ?? '';
  const heatOpts = [{ key: 'cpu' as const, label: t('사용률') }, { key: 'mhz' as const, label: t('클럭') }, { key: 'temp' as const, label: t('온도') }];
  const core = view === 'core' && !!lay;
  const pickTile = (m: Metric) => {
    if (m === metric) { setMetric(null); return; }
    setMetric(m);
    if (view !== 'core') return;
    const hm: HeatMode | null = m === 'cpu' ? 'cpu' : m === 'temp' ? 'temp' : m === 'mhz' ? 'mhz' : null;
    if (hm) setHeat(hm); else setView('trend');
  };
  return (
    <View style={{ paddingHorizontal: 15, paddingTop: 13, paddingBottom: 15, gap: 13 }}>
      <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 10 }}>
        {tiles.map((x) => (
          <Tile key={x.m} label={x.label} value={x.value} unit={x.unit} sub={x.sub} spark={spark(x.f)}
            tone={x.tone} toneText={x.toneText} active={metric === x.m} onPress={() => pickTile(x.m)} />
        ))}
      </View>
      {metric && (
        <Panel icon={core ? 'cpu' : 'pulse'}
          title={core ? `${t('코어별')} — ${heatOpts.find((o) => o.key === heat)?.label}` : `${metricName} — ${t('추이')}`}
          right={
            <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 10 }}>
              {lay && <View style={{ width: 170 }}><Segmented options={[{ key: 'trend' as const, label: t('추이') }, { key: 'core' as const, label: t('코어별') }]} value={view} onChange={setView} /></View>}
              <View style={{ width: 210 }}><Segmented options={winOpts} value={win} onChange={setWin} /></View>
            </View>
          }>
          <View style={{ minHeight: Math.max(viewH.trend, lay ? viewH.core : 0) }}>
            <View onLayout={measure('trend')} pointerEvents={core ? 'none' : 'auto'} style={core ? HIDDEN : undefined}>
              <TimeChart ss={ss} metric={metric} lay={lay} sel={sel} fill={lay ? viewH.core : 0} />
              {lay && coreGet(metric, 0) && <CorePicker lay={lay} sel={sel} onChange={setSel} />}
            </View>
            {lay && (
              <View onLayout={measure('core')} pointerEvents={core ? 'auto' : 'none'} style={core ? undefined : HIDDEN}>
                <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 10, marginBottom: 10 }}>
                  <View style={{ width: 240 }}><Segmented options={heatOpts} value={heat} onChange={setHeat} /></View>
                  <View style={{ width: 210 }}><Segmented options={[{ key: 'turbo' as const, label: 'Turbo' }, { key: 'heat' as const, label: t('히트맵') }]} value={scale} onChange={setScale} /></View>
                </View>
                <CoreHeatmap ss={ss} lay={lay} mode={heat} scale={scale} />
              </View>
            )}
          </View>
        </Panel>
      )}
    </View>
  );
}
