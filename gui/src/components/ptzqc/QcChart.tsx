import { useState } from 'react';
import { View } from 'react-native';
import Svg, { Line, Path, Rect, Text as SvgText } from 'react-native-svg';
import { useTheme } from '@/theme';
import { niceTicks, timeTicks } from '@/lib/ptzQc';

export type Series = { values: (number | null)[]; color: string; width?: number; band?: { min: (number | null)[]; max: (number | null)[] } };
export type Marks = { lat?: number[]; real?: number[] };

export function QcChart({ series, bucketS, totalS, unit, height = 180, limit, limitLabel, yMax, marks, baseS }: {
  series: Series[];
  bucketS: number;
  totalS: number;
  unit?: string;
  height?: number;
  limit?: number | null;
  limitLabel?: string;
  yMax?: number;
  marks?: Marks;
  baseS?: number;
}) {
  const { c, fonts } = useTheme();
  const [w, setW] = useState(0);
  const padL = 40, padR = 10, padT = 8, padB = 22;
  const all = series.flatMap((s) => [...s.values, ...(s.band?.max ?? [])]).filter((x): x is number => x != null && isFinite(x));
  const top = yMax ?? Math.max(...(all.length ? all : [1]), limit ?? 0) * 1.12;
  const yt = niceTicks(top, 4);
  const yTop = yt[yt.length - 1] || 1;
  const pw = Math.max(1, w - padL - padR), ph = height - padT - padB;
  const total = Math.max(totalS, bucketS);
  const xOf = (k: number) => padL + ((k + 0.5) * bucketS / total) * pw;
  const yOf = (v: number) => padT + ph - (Math.min(v, yTop) / yTop) * ph;
  const path = (vals: (number | null)[]) => {
    let d = '', pen = false;
    vals.forEach((v, k) => {
      if (v == null || !isFinite(v)) { pen = false; return; }
      d += `${pen ? 'L' : 'M'}${xOf(k).toFixed(1)} ${yOf(v).toFixed(1)} `;
      pen = true;
    });
    return d;
  };
  const band = (lo: (number | null)[], hi: (number | null)[]) => {
    let d = '';
    let run: number[] = [];
    const flush = () => {
      if (run.length > 1) {
        d += run.map((k, i) => `${i ? 'L' : 'M'}${xOf(k).toFixed(1)} ${yOf(hi[k] as number).toFixed(1)}`).join(' ');
        d += ' ' + [...run].reverse().map((k) => `L${xOf(k).toFixed(1)} ${yOf(lo[k] as number).toFixed(1)}`).join(' ') + ' Z ';
      }
      run = [];
    };
    hi.forEach((v, k) => { if (v != null && lo[k] != null) run.push(k); else flush(); });
    flush();
    return d;
  };
  const tt = timeTicks(total);
  return (
    <View onLayout={(e) => { const nw = Math.floor(e.nativeEvent.layout.width); if (nw !== w) setW(nw); }} style={{ width: '100%', height }}>
      {w > 0 ? (
        <Svg width={w} height={height}>
          {baseS ? <Rect x={padL} y={padT} width={Math.min(pw, (baseS / total) * pw)} height={ph} fill={c.elev} opacity={0.6} /> : null}
          {yt.map((v) => (
            <Line key={`y${v}`} x1={padL} x2={padL + pw} y1={yOf(v)} y2={yOf(v)} stroke={c.line} strokeWidth={1} />
          ))}
          {yt.map((v) => (
            <SvgText key={`yl${v}`} x={padL - 6} y={yOf(v) + 4} fontSize={10} fill={c.dim} textAnchor="end" fontFamily={fonts.mono}>{`${+v.toFixed(2)}`}</SvgText>
          ))}
          {tt.map((x) => (
            <SvgText key={`t${x.s}`} x={padL + (x.s / total) * pw} y={height - 6} fontSize={10} fill={c.dim} textAnchor="middle">{x.label}</SvgText>
          ))}
          {series.map((s, i) => (s.band ? <Path key={`b${i}`} d={band(s.band.min, s.band.max)} fill={s.color} opacity={0.18} /> : null))}
          {series.map((s, i) => <Path key={`s${i}`} d={path(s.values)} stroke={s.color} strokeWidth={s.width ?? 1.6} fill="none" />)}
          {limit != null && limit <= yTop ? (
            <>
              <Line x1={padL} x2={padL + pw} y1={yOf(limit)} y2={yOf(limit)} stroke={c.red} strokeWidth={1.2} strokeDasharray="5 4" />
              {limitLabel ? (
                <SvgText x={padL + pw - 2} y={yOf(limit) - 4} fontSize={10.5} fill={c.redTx} textAnchor="end">{limitLabel}</SvgText>
              ) : null}
            </>
          ) : null}
          {(marks?.lat ?? []).map((k) => <Line key={`ml${k}`} x1={xOf(k)} x2={xOf(k)} y1={padT + ph} y2={padT + ph - 6} stroke={c.amber} strokeWidth={1.5} />)}
          {(marks?.real ?? []).map((k) => <Line key={`mr${k}`} x1={xOf(k)} x2={xOf(k)} y1={padT + ph} y2={padT + ph - 14} stroke={c.red} strokeWidth={2} />)}
        </Svg>
      ) : null}
    </View>
  );
}
