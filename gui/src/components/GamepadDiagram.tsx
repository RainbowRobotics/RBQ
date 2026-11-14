import { View, Text, StyleSheet, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { keyLabel } from '@/lib/gamepad/profiles';

export type DiagramAxes = { lx: number; ly: number; rx: number; ry: number; hatX: number; hatY: number };

type Props = {
  deviceName?: string;
  keys?: number[];
  pressed?: number[];
  axes?: DiagramAxes;
  bound?: number[];
  onPressKey?: (keyCode: number, label: string) => void;
  compact?: boolean;
  scale?: number;
};

const K = { A: 96, B: 97, X: 99, Y: 100, L1: 102, R1: 103, L3: 106, R3: 107, U: 19, D: 20, L: 21, R: 22 };

export function GamepadDiagram({ deviceName, keys, pressed = [], axes, bound = [], onPressKey, compact, scale }: Props) {
  const { c } = useTheme();
  const s = scale ?? (compact ? 0.8 : 1);
  const PAD = 48 * s;
  const SEG = 15 * s;
  const CTR = 14 * s;
  const STICK = 36 * s;
  const DOT = 10 * s;

  const has = (k: number) => !keys || keys.includes(k);
  const colors = (k: number) => {
    const on = pressed.includes(k);
    const bd = bound.includes(k);
    return {
      backgroundColor: on ? 'rgba(77,156,245,0.5)' : c.elev,
      borderColor: on || bd ? 'rgba(77,156,245,0.7)' : c.line,
      borderWidth: bd ? 1.5 : 1,
      opacity: has(k) ? 1 : 0.2,
    };
  };
  const Btn = ({ k, st, children }: { k: number; st: object[]; children?: React.ReactNode }) => (
    <Pressable disabled={!onPressKey || !has(k)} onPress={() => onPressKey?.(k, keyLabel(k))} style={[...st, colors(k)]}>
      {children}
    </Pressable>
  );

  const stick = (x: number, y: number) => (
    <View style={{ width: STICK, height: STICK, borderRadius: STICK / 2, borderWidth: 1, backgroundColor: c.elev, borderColor: c.line, alignItems: 'center', justifyContent: 'center' }}>
      <View style={{
        width: DOT, height: DOT, borderRadius: DOT / 2,
        backgroundColor: Math.hypot(x, y) > 0.02 ? c.accent2 : c.dim,
        transform: [{ translateX: x * DOT }, { translateY: -y * DOT }],
      }} />
    </View>
  );

  const cell = (col: number, row: number, w = SEG, h = SEG) => ({
    position: 'absolute' as const,
    left: (PAD / 3) * col + (PAD / 3 - w) / 2,
    top: (PAD / 3) * row + (PAD / 3 - h) / 2,
    width: w, height: h,
  });

  const dpad = (
    <View style={{ width: PAD, height: PAD }}>
      <Btn k={K.U} st={[styles.sq, cell(1, 0)]} />
      <Btn k={K.D} st={[styles.sq, cell(1, 2)]} />
      <Btn k={K.L} st={[styles.sq, cell(0, 1)]} />
      <Btn k={K.R} st={[styles.sq, cell(2, 1)]} />
      <Btn k={K.L3} st={[styles.round, cell(1, 1, CTR, CTR)]} />
    </View>
  );
  const faceTx = (k: number, l: string) => (
    <Text style={{ color: pressed.includes(k) ? '#fff' : c.dim, fontSize: 7.5 * s, fontWeight: '700' }}>{l}</Text>
  );
  const diamond = (
    <View style={{ width: PAD, height: PAD }}>
      <Btn k={K.Y} st={[styles.round, cell(1, 0)]}>{faceTx(K.Y, 'Y')}</Btn>
      <Btn k={K.X} st={[styles.round, cell(0, 1)]}>{faceTx(K.X, 'X')}</Btn>
      <Btn k={K.B} st={[styles.round, cell(2, 1)]}>{faceTx(K.B, 'B')}</Btn>
      <Btn k={K.A} st={[styles.round, cell(1, 2)]}>{faceTx(K.A, 'A')}</Btn>
      <Btn k={K.R3} st={[styles.round, cell(1, 1, CTR, CTR)]} />
    </View>
  );
  const shoulder = (k: number, l: string) => (
    <Btn k={k} st={[{ width: 34 * s, height: 15 * s, borderRadius: 5, alignItems: 'center', justifyContent: 'center' }]}>
      <Text style={{ color: pressed.includes(k) ? '#fff' : c.dim, fontSize: 8 * s, fontWeight: '700' }}>{l}</Text>
    </Btn>
  );

  return (
    <View style={{ flexDirection: 'row', alignItems: 'center', gap: 16 * s }}>
      <View style={{ alignItems: 'center', gap: 6 * s, width: PAD }}>
        {shoulder(K.L1, 'L1')}
        {stick(axes?.lx ?? 0, axes?.ly ?? 0)}
        {dpad}
      </View>
      <View style={{ alignItems: 'center', minWidth: 90 * s }}>
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 5 }}>
          <Icon name="gamepad" size={12} color={c.green} />
          <Text style={{ color: c.text, fontSize: 8.5, fontWeight: '600', maxWidth: 86 }} numberOfLines={1}>{deviceName ?? ''}</Text>
          <View style={{ width: 5, height: 5, borderRadius: 3, backgroundColor: c.green }} />
        </View>
      </View>
      <View style={{ alignItems: 'center', gap: 6 * s, width: PAD }}>
        {shoulder(K.R1, 'R1')}
        {stick(axes?.rx ?? 0, axes?.ry ?? 0)}
        {diamond}
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  sq: { borderRadius: 3 },
  round: { borderRadius: 99, alignItems: 'center', justifyContent: 'center' },
});
