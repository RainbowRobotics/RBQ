import { useMemo, useState } from 'react';
import { View, Text } from 'react-native';
import Svg, { Rect, Polygon } from 'react-native-svg';
import { useTheme } from '@/theme';
import { useWallMap } from '@/lib/wallMap';
import { useSettings } from '@/store/settings';
import { t } from '@/lib/i18n';
import { Tappable } from '@/components/anim';
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

const BODY_WID_M = 0.20;

export function WallMapView() {
  const { c, radius } = useTheme();
  const frame = useWallMap((s) => s.frame);
  const [box, setBox] = useState({ w: 0, h: 0 });
  const size = Math.min(box.w, box.h) * 0.94;
  const showCam = useSettings((s) => s.showObsMapCam);
  const setShowCam = useSettings((s) => s.setShowObsMapCam);

  const grid = useMemo(() => {
    if (!frame) return null;
    return downsample(frame.cells, frame.rows, frame.cols, DISPLAY_CELLS);
  }, [frame]);

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

  const at = (x: number, y: number) => grid.cells[y * grid.cols + x];
  const rects: React.ReactNode[] = [];
  for (let y = 0; y < grid.rows; y++) {
    for (let x = 0; x < grid.cols; x++) {
      if (at(x, y) === 0) continue;
      const up    = y > 0             ? at(x, y - 1) : 0;
      const down  = y < grid.rows - 1 ? at(x, y + 1) : 0;
      const left  = x > 0             ? at(x - 1, y) : 0;
      const right = x < grid.cols - 1 ? at(x + 1, y) : 0;
      if (up !== 0 && down !== 0 && left !== 0 && right !== 0) continue;
      rects.push(
        <Rect key={`${x}-${y}`} x={x} y={y} width={1} height={1} fill={c.red} opacity={0.85} />,
      );
    }
  }

  const cx = grid.cols / 2;
  const cy = grid.rows / 2;
  const effGs = frame!.gs * grid.bs;
  const bodyWidCells = BODY_WID_M / effGs;
  const tri = Math.max(1, bodyWidCells * 0.6);

  return (
    <View style={{ flex: 1, alignItems: 'center', justifyContent: 'center', backgroundColor: c.bg }} onLayout={onLayout}>
      {showCam && (
        <View style={{ position: 'absolute', width: size, height: size }}>
          <GroundCamOverlay size={size} />
        </View>
      )}
      <Svg width={size} height={size} viewBox={`0 0 ${grid.cols} ${grid.rows}`}>
        {rects}
        <Polygon
          points={`${cx},${cy - tri} ${cx - tri * 0.7},${cy + tri * 0.6} ${cx + tri * 0.7},${cy + tri * 0.6}`}
          fill={c.accent2}
        />
      </Svg>
      <Tappable onPress={() => setShowCam(!showCam)}
        accessibilityLabel={showCam ? t('카메라 끄기') : t('카메라 켜기')}
        style={{
          position: 'absolute', bottom: 8, alignSelf: 'center',
          width: 28, height: 28, borderRadius: radius.md, alignItems: 'center', justifyContent: 'center',
          backgroundColor: showCam ? 'rgba(77,156,245,0.12)' : c.elev,
          borderWidth: 1, borderColor: showCam ? 'rgba(77,156,245,0.5)' : c.line,
        }}>
        <Text style={{ fontSize: 13, opacity: showCam ? 1 : 0.4 }}>📷</Text>
      </Tappable>
    </View>
  );
}
