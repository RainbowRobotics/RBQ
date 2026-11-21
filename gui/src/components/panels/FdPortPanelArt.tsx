import { View } from 'react-native';
import Svg, { G, Rect, Circle, Text as SvgText } from 'react-native-svg';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { t } from '@/lib/i18n';
import { FD_PORT, FD_PORTS, type PduFdState, type FdPortDef } from '@/lib/pduFd';

const VW = 1000;
const VH = 520;

type Pins = 4 | 6 | 12;
type Conn = { key: string; x: number; y: number; r: number; pins: Pins; label?: string; port?: number; emo?: boolean };
const CONNS: Conn[] = [
  { key: 'addon', x: 320, y: 160, r: 52, pins: 4, label: 'ADDON 48V', port: FD_PORT.ARM_48V },
  { key: 'lan2', x: 590, y: 150, r: 72, pins: 12, label: 'LAN 2', port: FD_PORT.LAN_12V_3 },
  { key: 'lan1', x: 862, y: 150, r: 72, pins: 12, label: 'LAN 1', port: FD_PORT.LAN_12V_2 },
  { key: 'aux', x: 115, y: 375, r: 52, pins: 6 },
  { key: 'emo', x: 318, y: 375, r: 52, pins: 6, label: 'EMO', emo: true },
  { key: 'wan', x: 590, y: 375, r: 72, pins: 12, label: 'WAN', port: FD_PORT.WAN_12V },
  { key: 'lan0', x: 862, y: 375, r: 72, pins: 12, label: 'LAN 0', port: FD_PORT.LAN_12V_1 },
];

function pinXY(pins: Pins): [number, number][] {
  const ring = (n: number, rr: number, a0: number) =>
    Array.from({ length: n }, (_, i) => {
      const a = a0 + (i * 2 * Math.PI) / n;
      return [rr * Math.cos(a), rr * Math.sin(a)] as [number, number];
    });
  if (pins === 12) return [...ring(3, 0.22, -Math.PI / 2), ...ring(9, 0.5, -Math.PI / 2)];
  if (pins === 6) return [[0, 0], ...ring(5, 0.36, -Math.PI / 2)];
  return [[-0.2, -0.2], [0.2, -0.2], [-0.2, 0.2], [0.2, 0.2]];
}

const LED_X = [85, 145];
const LED_Y = [120, 165, 210, 255];

export function FdPortPanelArt({ fd, onPress }: { fd: PduFdState | null; onPress: (p: FdPortDef) => void }) {
  const { c } = useTheme();
  const defOf = (idx: number) => FD_PORTS.find((p) => p.index === idx);

  const look = (k: Conn) => {
    const s = k.emo ? fd?.portIn[0] : k.port != null ? fd?.portOut[k.port] : undefined;
    if (!k.label) return { color: c.line2, sub: '' };
    if (!s) return { color: c.dim, sub: fd ? '' : t('수신 전') };
    const color = s.state === 1 ? c.green : s.state === -1 ? c.redbright : c.muted;
    const st = s.state === 1 ? 'ON' : s.state === -1 ? 'ERROR' : 'OFF';
    return { color, sub: k.emo ? st : `${st} ${s.voltage.toFixed(1)}V` };
  };

  return (
    <View style={{ width: '100%', maxWidth: 560, aspectRatio: VW / VH, alignSelf: 'center', marginVertical: 12 }}>
      <Svg width="100%" height="100%" viewBox={`0 0 ${VW} ${VH}`}>
        <Rect x={8} y={8} width={VW - 16} height={VH - 16} rx={24} fill="none" stroke={c.line2} strokeWidth={3} />
        {LED_X.flatMap((x) => LED_Y.map((y) => (
          <Circle key={`led-${x}-${y}`} cx={x} cy={y} r={17} fill="none" stroke={c.line2} strokeWidth={2} />
        )))}
        {CONNS.map((k) => {
          const { color, sub } = look(k);
          const on = color === c.green;
          return (
            <G key={k.key}>
              {k.key !== 'aux' && [-18, 18].map((dy) => (
                <Circle key={dy} cx={k.x - k.r - 34} cy={k.y + dy} r={6} fill="none" stroke={c.line2} strokeWidth={2} />
              ))}
              <Circle cx={k.x} cy={k.y} r={k.r} fill={color} fillOpacity={on ? 0.14 : 0} stroke={color} strokeWidth={on ? 5 : 3} />
              <Circle cx={k.x} cy={k.y} r={k.r * 0.72} fill="none" stroke={color} strokeWidth={2} />
              {pinXY(k.pins).map(([px, py], i) => (
                <Circle key={i} cx={k.x + px * k.r} cy={k.y + py * k.r} r={k.r * 0.075} fill={color} />
              ))}
              {k.label && (
                <SvgText x={k.x} y={k.y + k.r + 28} fontSize={24} fontWeight="700" fill={c.text} textAnchor="middle">{k.label}</SvgText>
              )}
              {!!sub && (
                <SvgText x={k.x} y={k.y + k.r + 54} fontSize={18} fill={color === c.line2 ? c.dim : color} textAnchor="middle">{sub}</SvgText>
              )}
            </G>
          );
        })}
      </Svg>
      {CONNS.filter((k) => k.port != null).map((k) => {
        const def = defOf(k.port!);
        if (!def) return null;
        return (
          <Tappable key={k.key} disabled={!fd} onPress={() => onPress(def)} accessibilityLabel={`${k.label} — ${t(def.nm)}`}
            style={{
              position: 'absolute', borderRadius: 999,
              left: `${((k.x - k.r) / VW) * 100}%`, top: `${((k.y - k.r) / VH) * 100}%`,
              width: `${((2 * k.r) / VW) * 100}%`, height: `${((2 * k.r) / VH) * 100}%`,
            }}>
            <View style={{ flex: 1 }} />
          </Tappable>
        );
      })}
    </View>
  );
}
