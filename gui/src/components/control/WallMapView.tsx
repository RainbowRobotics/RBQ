import { useMemo, useState } from 'react';
import { View, Text } from 'react-native';
import Svg, { Rect, Polygon, Line } from 'react-native-svg';
import { useTheme } from '@/theme';
import { useWallMap } from '@/lib/wallMap';
import { inflateWallMap } from '@/lib/wallMapInflate';
import { useSettings } from '@/store/settings';
import { t } from '@/lib/i18n';
import { GroundCamOverlay } from './GroundCamOverlay';

const DISPLAY_CELLS = 320;

function downsample(cells: Uint8Array, rows: number, cols: number, target: number) {
  const bs = Math.max(1, Math.ceil(Math.max(rows, cols) / target));
  const outRows = Math.ceil(rows / bs);
  const outCols = Math.ceil(cols / bs);
  const out = new Uint8Array(outRows * outCols);
  for (let by = 0; by < outRows; by++) {
    const y0 = by * bs, y1 = Math.min(rows, y0 + bs);
    for (let bx = 0; bx < outCols; bx++) {
      const x0 = bx * bs, x1 = Math.min(cols, x0 + bs);
      let maxV = 0;
      for (let y = y0; y < y1; y++) {
        const rowOff = y * cols;
        for (let x = x0; x < x1; x++) {
          const v = cells[rowOff + x];
          if (v > maxV) maxV = v;
        }
      }
      out[by * outCols + bx] = maxV;
    }
  }
  return { cells: out, rows: outRows, cols: outCols, bs };
}
type DownsampledGrid = ReturnType<typeof downsample>;

function mergedRectSpans(rows: number, cols: number, test: (x: number, y: number) => boolean): { x: number; y: number; w: number; h: number }[] {
  const out: { x: number; y: number; w: number; h: number }[] = [];
  let pending = new Map<string, { x: number; y: number; w: number; h: number }>();
  for (let y = 0; y < rows; y++) {
    const next = new Map<string, { x: number; y: number; w: number; h: number }>();
    let x = 0;
    while (x < cols) {
      if (!test(x, y)) { x++; continue; }
      const x0 = x;
      while (x < cols && test(x, y)) x++;
      const key = `${x0}-${x}`;
      const prev = pending.get(key);
      if (prev) { prev.h++; next.set(key, prev); }
      else next.set(key, { x: x0, y, w: x - x0, h: 1 });
    }
    for (const [key, rect] of pending) if (!next.has(key)) out.push(rect);
    pending = next;
  }
  for (const rect of pending.values()) out.push(rect);
  return out;
}

function renderSpans(spans: { x: number; y: number; w: number; h: number }[], color: string, opacity: number, keyPrefix: string, pad = 0): React.ReactNode[] {
  return spans.map((r, i) => (
    <Rect key={`${keyPrefix}-${i}`} x={r.x - pad} y={r.y - pad} width={r.w + 2 * pad} height={r.h + 2 * pad} fill={color} opacity={opacity} />
  ));
}

function boundaryRects(grid: DownsampledGrid, color: string, opacity: number, keyPrefix: string, pad = 0): React.ReactNode[] {
  const at = (x: number, y: number) => grid.cells[y * grid.cols + x];
  const isBoundary = (x: number, y: number) => {
    if (at(x, y) === 0) return false;
    const up    = y > 0              ? at(x, y - 1) : 0;
    const down  = y < grid.rows - 1  ? at(x, y + 1) : 0;
    const left  = x > 0              ? at(x - 1, y) : 0;
    const right = x < grid.cols - 1  ? at(x + 1, y) : 0;
    return !(up !== 0 && down !== 0 && left !== 0 && right !== 0);
  };
  return renderSpans(mergedRectSpans(grid.rows, grid.cols, isBoundary), color, opacity, keyPrefix, pad);
}

function filledRects(grid: DownsampledGrid, color: string, opacity: number, keyPrefix: string): React.ReactNode[] {
  return renderSpans(mergedRectSpans(grid.rows, grid.cols, (x, y) => grid.cells[y * grid.cols + x] !== 0), color, opacity, keyPrefix);
}

const RAW_FILL_INFLATE_M = 0.01;

const RAW_FILL_COLOR = '#ff0000';

const MARGIN_COLOR = '#ff6b6b';

const MARGIN_LINE_PAD = 0.25;

const BODY_WID_M = 0.20;

const AVOID_RANGE_COLOR = '#00c951';
const AVOID_RANGE_LINE_WIDTH = 0.5;

export function WallMapView({ active = true }: { active?: boolean } = {}) {
  const { c } = useTheme();
  const frame = useWallMap((s) => s.frame);
  const margin = useSettings((s) => s.obsAvoidMargin);
  const obsAvoidRange = useSettings((s) => s.obsAvoidRange);
  const obsAvoidMode = useSettings((s) => s.obsAvoidMode);
  const obsAvoidOn = useSettings((s) => s.obsAvoidEnabled);
  const [box, setBox] = useState({ w: 0, h: 0 });
  const size = box.h * 0.94;

  const grid = useMemo(() => {
    if (!frame) return null;
    return downsample(frame.cells, frame.rows, frame.cols, DISPLAY_CELLS);
  }, [frame]);

  const inflatedGrid = useMemo(() => {
    if (!frame || !obsAvoidOn) return null;
    const inflated = inflateWallMap(frame.cells, frame.rows, frame.cols, frame.gs, margin);
    return downsample(inflated, frame.rows, frame.cols, DISPLAY_CELLS);
  }, [frame, margin, obsAvoidOn]);

  const rawFillGrid = useMemo(() => {
    if (!frame || !obsAvoidOn) return null;
    const filled = inflateWallMap(frame.cells, frame.rows, frame.cols, frame.gs, RAW_FILL_INFLATE_M);
    return downsample(filled, frame.rows, frame.cols, DISPLAY_CELLS);
  }, [frame, obsAvoidOn]);

  const onLayout = (e: { nativeEvent: { layout: { width: number; height: number } } }) => {
    const { width, height } = e.nativeEvent.layout;
    setBox({ w: width, h: height });
  };

  if (!grid || size <= 0) {
    return (
      <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', backgroundColor: c.bg }} onLayout={onLayout}>
        {grid == null && <Text style={{ color: c.dim, fontSize: 12 }}>{t('장애물 지도 수신 대기 중')}</Text>}
      </View>
    );
  }

  const inflatedRects = inflatedGrid ? boundaryRects(inflatedGrid, MARGIN_COLOR, 0.85, 'inf', MARGIN_LINE_PAD) : [];
  const rawRects = rawFillGrid ? filledRects(rawFillGrid, RAW_FILL_COLOR, 0.85, 'raw') : [];

  const cx = grid.cols / 2;
  const cy = grid.rows / 2;
  const effGs = frame!.gs * grid.bs;
  const bodyWidCells = BODY_WID_M / effGs;
  const tri = Math.max(1, bodyWidCells * 0.6);

  const rangeCells = obsAvoidRange / effGs;

  return (
    <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', backgroundColor: c.bg }} onLayout={onLayout}>
      <View style={{ position: 'absolute', width: size, height: size }}>
        <GroundCamOverlay size={size} active={active} />
      </View>
      <Svg width={size} height={size} style={{ position: 'absolute' }} viewBox={`0 0 ${grid.cols} ${grid.rows}`}>
        {inflatedRects}
        {rawRects}
        {obsAvoidMode === 'avoid' && (
          <>
            <Line x1={cx - rangeCells} y1={0} x2={cx - rangeCells} y2={grid.rows}
              stroke={AVOID_RANGE_COLOR} strokeWidth={AVOID_RANGE_LINE_WIDTH} strokeDasharray="3,3" />
            <Line x1={cx + rangeCells} y1={0} x2={cx + rangeCells} y2={grid.rows}
              stroke={AVOID_RANGE_COLOR} strokeWidth={AVOID_RANGE_LINE_WIDTH} strokeDasharray="3,3" />
          </>
        )}
        <Polygon
          points={`${cx},${cy - tri} ${cx - tri * 0.7},${cy + tri * 0.6} ${cx + tri * 0.7},${cy + tri * 0.6}`}
          fill={c.accent2}
        />
      </Svg>
    </View>
  );
}
