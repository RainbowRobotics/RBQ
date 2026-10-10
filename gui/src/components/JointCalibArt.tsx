import Svg, { Defs, LinearGradient, Stop, Rect, Path, Line, Circle, Polyline } from 'react-native-svg';
import { useTheme } from '@/theme';

export function SittingRobotArt({ width = 200 }: { width?: number }) {
  const { c } = useTheme();
  const body = c.elev2;
  const edge = c.muted;
  return (
    <Svg width={width} height={width * (116 / 200)} viewBox="0 0 200 116">
      <Line x1={10} y1={102} x2={190} y2={102} stroke={c.muted} strokeWidth={2} strokeLinecap="round" opacity={0.55} />

      <Polyline points="62,80 76,96 48,100" fill="none" stroke={edge} strokeWidth={7}
        strokeLinecap="round" strokeLinejoin="round" opacity={0.55} />
      <Polyline points="146,80 160,96 132,100" fill="none" stroke={edge} strokeWidth={7}
        strokeLinecap="round" strokeLinejoin="round" opacity={0.55} />
      <Polyline points="68,80 82,96 54,100" fill="none" stroke={edge} strokeWidth={7}
        strokeLinecap="round" strokeLinejoin="round" />
      <Polyline points="140,80 154,96 126,100" fill="none" stroke={edge} strokeWidth={7}
        strokeLinecap="round" strokeLinejoin="round" />

      <Rect x={46} y={54} width={112} height={28} rx={7} fill={body} stroke={edge} strokeWidth={1.6} />
      <Rect x={78} y={47} width={62} height={8} rx={3} fill={body} stroke={edge} strokeWidth={1.3} />
      <Rect x={96} y={31} width={22} height={16} rx={3} fill={body} stroke={edge} strokeWidth={1.3} />
      <Rect x={99} y={26} width={16} height={6} rx={2} fill={edge} opacity={0.7} />
      <Rect x={35} y={58} width={13} height={17} rx={3} fill={body} stroke={edge} strokeWidth={1.3} />

      <Circle cx={68} cy={80} r={3.4} fill={edge} />
      <Circle cx={140} cy={80} r={3.4} fill={edge} />
    </Svg>
  );
}

export function PitchJigArt({ width = 200 }: { width?: number }) {
  const { c } = useTheme();
  return (
    <Svg width={width} height={width * (104 / 196)} viewBox="0 0 196 104">
      <Defs>
        <LinearGradient id="jigBar" x1="0" y1="0" x2="0" y2="1">
          <Stop offset="0" stopColor="#C8D0D9" />
          <Stop offset="0.45" stopColor="#98A2AE" />
          <Stop offset="1" stopColor="#6E7884" />
        </LinearGradient>
        <LinearGradient id="jigPin" x1="0" y1="0" x2="1" y2="0">
          <Stop offset="0" stopColor="#7C8794" />
          <Stop offset="0.35" stopColor="#C3CBD4" />
          <Stop offset="1" stopColor="#6E7884" />
        </LinearGradient>
      </Defs>

      <Path d="M18,50 L34,50 L34,80 A8,8 0 0 1 18,80 Z" fill="url(#jigPin)" stroke="#5A626E" strokeWidth={1.2} />
      <Rect x={21} y={53} width={3.5} height={24} rx={1.75} fill="#E6ECF2" opacity={0.55} />

      <Rect x={18} y={28} width={160} height={22} rx={2.5} fill="url(#jigBar)" stroke="#5A626E" strokeWidth={1.2} />
      <Rect x={20} y={30} width={156} height={4.5} rx={2} fill="#E6ECF2" opacity={0.6} />

      <Line x1={14} y1={92} x2={182} y2={92} stroke={c.muted} strokeWidth={2} strokeLinecap="round" opacity={0.55} />
    </Svg>
  );
}

export function RobotTopViewArt({ width = 96, legColor }: { width?: number; legColor: string[] }) {
  const { c } = useTheme();
  const body = c.elev2;
  const edge = c.muted;
  const legs: [number, number, number, number][] = [[3, 34, 58, 14], [2, 86, 58, 106], [1, 34, 150, 14], [0, 86, 150, 106]];
  return (
    <Svg width={width} height={width * (200 / 120)} viewBox="0 0 120 200">
      <Path d="M60 4 L68 16 L52 16 Z" fill={c.accent2} />
      {legs.map(([leg, hx, hy, fx]) => (
        <Line key={`l${leg}`} x1={hx} y1={hy} x2={fx} y2={hy + 6} stroke={edge} strokeWidth={7} strokeLinecap="round" />
      ))}
      <Rect x={34} y={38} width={52} height={132} rx={10} fill={body} stroke={edge} strokeWidth={1.6} />
      <Rect x={46} y={24} width={28} height={16} rx={4} fill={body} stroke={edge} strokeWidth={1.3} />
      <Rect x={50} y={92} width={20} height={24} rx={3} fill={edge} opacity={0.35} />
      {legs.map(([leg, hx, hy, fx]) => (
        <Circle key={`f${leg}`} cx={fx} cy={hy + 6} r={8} fill={legColor[leg]} stroke={edge} strokeWidth={1.2} />
      ))}
      {legs.map(([leg, hx, hy]) => <Circle key={`h${leg}`} cx={hx} cy={hy} r={4} fill={edge} />)}
    </Svg>
  );
}
