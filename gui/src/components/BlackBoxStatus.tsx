import { useState } from 'react';
import { View, Text, StyleSheet, Pressable } from 'react-native';
import { AttitudeDial } from '@/components/AttitudeDial';
import { GamepadDiagram } from '@/components/GamepadDiagram';
import { useTheme } from '@/theme';
import { chan, chanOpt, type BlackboxSession } from '@/lib/blackbox';
import { t } from '@/lib/i18n';
import { gaitIdName } from '@/lib/gaitNames';

const R2D = 180 / Math.PI;
const LEG_JOINTS = 12;
const GAP = 5;
const CARD_H = 40;
const MIN_CARD_W = 62;
const PAD_H = 111;
const ROW_GAP = 8;
const RPY_W = 54;

export type BbStatusItem = { k: string; v: string; color?: string; jump: string };

export const gaitName = gaitIdName;

function jointList(ids: number[]): string {
  return ids.length <= 2 ? ids.join(',') : `${ids.slice(0, 2).join(',')} +${ids.length - 2}`;
}

type Colors = { greenTx: string; redTx: string };

const BTN_KEY: Record<number, number> = { 0: 96, 1: 97, 2: 99, 3: 100, 4: 102, 5: 103, 9: 106, 10: 107, 11: 19, 12: 20, 13: 22, 14: 21 };
const BTN_NAME: Record<number, string> = { 6: 'BACK', 7: 'START', 8: 'LOGI', 15: 'SELECT' };

export function joyState(sess: BlackboxSession, frame: number) {
  const v = (n: string) => chanOpt(sess, frame, n) ?? 0;
  const pressed: number[] = [];
  const extra: string[] = [];
  if (v('joy.lt') > 0.5) extra.push('LT');
  if (v('joy.rt') > 0.5) extra.push('RT');
  for (let i = 0; i < 16; i++) {
    if (v(`joy.btn[${i}]`) === 0) continue;
    if (BTN_KEY[i] !== undefined) pressed.push(BTN_KEY[i]);
    else if (BTN_NAME[i]) extra.push(BTN_NAME[i]);
  }
  return {
    axes: { lx: v('joy.l_rl'), ly: v('joy.l_ud'), rx: v('joy.r_rl'), ry: v('joy.r_ud'), hatX: 0, hatY: 0 },
    pressed, extra,
  };
}

export function baseStatusItems(sess: BlackboxSession, frame: number, rpy: number[], c: Colors): BbStatusItem[] {
  const fall = chan(sess, frame, 'status.is_fall') !== 0;
  return [
    { k: 'ROLL', v: `${(rpy[0] * R2D).toFixed(1)}°`, jump: 'imu' },
    { k: 'PITCH', v: `${(rpy[1] * R2D).toFixed(1)}°`, jump: 'imu' },
    { k: 'YAW', v: `${(rpy[2] * R2D).toFixed(1)}°`, jump: 'imu' },
    { k: 'GAIT', v: gaitName(chan(sess, frame, 'status.gait_id')), color: c.greenTx, jump: 'status' },
    { k: 'FALL', v: fall ? 'YES' : 'no', color: fall ? c.redTx : undefined, jump: 'status' },
    { k: 'BAT L', v: `${chan(sess, frame, 'pdu.bat.left.voltage').toFixed(1)}V`, jump: 'pdu.bat' },
    { k: 'BAT R', v: `${chan(sess, frame, 'pdu.bat.right.voltage').toFixed(1)}V`, jump: 'pdu.bat' },
    { k: 'CMD VX', v: chan(sess, frame, 'cmd.vel_x').toFixed(2), jump: 'cmd' },
    { k: 'JOY L', v: `${chan(sess, frame, 'joy.l_rl').toFixed(2)},${chan(sess, frame, 'joy.l_ud').toFixed(2)}`, jump: 'joy' },
  ];
}

export function systemStatusItems(sess: BlackboxSession, frame: number, c: Colors): BbStatusItem[] {
  const dash = '-';
  const cpuT = chanOpt(sess, frame, 'cpu.temp');
  const thr = chanOpt(sess, frame, 'cpu.throttled');
  const miss = chanOpt(sess, frame, 'deadline_miss');

  const jam: number[] = [];
  const curErr: number[] = [];
  for (let i = 0; i < LEG_JOINTS; i++) {
    if ((chanOpt(sess, frame, `motor.err.jam[${i}]`) ?? 0) !== 0) jam.push(i);
    if ((chanOpt(sess, frame, `motor.err.cur[${i}]`) ?? 0) !== 0) curErr.push(i);
  }
  const hasMotor = sess.cols.has('motor.err.jam[0]');
  return [
    { k: 'CPU TEMP', v: cpuT === undefined ? dash : `${cpuT.toFixed(0)}°C`, jump: 'cpu' },
    { k: 'THROTTLE', v: thr === undefined ? dash : thr !== 0 ? 'YES' : 'no',
      color: thr ? c.redTx : undefined, jump: 'cpu' },
    { k: 'STALL', v: !hasMotor ? dash : jam.length ? `J${jointList(jam)}` : 'OK',
      color: !hasMotor ? undefined : jam.length ? c.redTx : c.greenTx, jump: 'joint' },
    { k: 'OVERCUR', v: !hasMotor ? dash : curErr.length ? `J${jointList(curErr)}` : 'OK',
      color: !hasMotor ? undefined : curErr.length ? c.redTx : c.greenTx, jump: 'joint' },
    { k: 'MISS', v: miss === undefined ? dash : miss.toFixed(0),
      color: miss ? c.redTx : undefined, jump: 'deadline_miss' },
  ];
}

function fitFont(v: string, width?: number) {
  if (width === undefined) return 11;
  return Math.max(8, Math.min(11, Math.floor(((width - 14) / (v.length * 0.62)) * 2) / 2));
}

export function StatusChip({ item, onPress, width, height }: {
  item: BbStatusItem; onPress?: (jump: string) => void; width?: number; height?: number;
}) {
  const { c } = useTheme();
  return (
    <Pressable disabled={!onPress} onPress={() => onPress?.(item.jump)}
      style={({ pressed }) => [styles.chip, width !== undefined && { width, minWidth: 0, paddingHorizontal: 6 }, height !== undefined && { height },
        { backgroundColor: pressed ? c.panel2 : c.elev, borderColor: c.line }]}>
      <Text numberOfLines={1} style={{ fontSize: 7.5, fontWeight: '700', letterSpacing: 0.5, color: c.dim }}>{item.k}</Text>
      <Text numberOfLines={1} style={{ fontFamily: 'monospace', fontSize: fitFont(item.v, width), fontWeight: '600', color: item.color ?? c.text }}>
        {item.v}
      </Text>
    </Pressable>
  );
}

export function quadStatusItems(sess: BlackboxSession, frame: number, rpy: number[], c: Colors): BbStatusItem[] {
  const base = baseStatusItems(sess, frame, rpy, c);
  const by = (k: string) => base.find((it) => it.k === k)!;
  const bat = `${chan(sess, frame, 'pdu.bat.left.voltage').toFixed(1)}/${chan(sess, frame, 'pdu.bat.right.voltage').toFixed(1)}`;
  const extra = joyState(sess, frame).extra;
  return [
    by('GAIT'), by('FALL'), { k: 'BAT L/R V', v: bat, jump: 'pdu.bat' }, by('CMD VX'),
    { k: 'BTN', v: extra.length ? extra.join(' ') : '-', color: extra.length ? c.greenTx : undefined, jump: 'joy.btn' },
    ...systemStatusItems(sess, frame, c),
  ];
}

export function BlackBoxStatusGrid({ items, rpy, joy, onJump }: {
  items: BbStatusItem[]; rpy: number[]; joy: ReturnType<typeof joyState>; onJump: (jump: string) => void;
}) {
  const { c, fonts, radius } = useTheme();
  const [w, setW] = useState(0);
  const dial = CARD_H * 2 + GAP;
  const padScale = Math.min(0.9, dial / PAD_H);
  const padW = 112 * padScale;
  const cardsW = (rpy: boolean) => w - dial - (rpy ? RPY_W + 6 : 0) - padW - ROW_GAP * 2;
  const cardW = (rpy: boolean, n: number) => {
    const cols = Math.ceil(items.length / n);
    return Math.floor((cardsW(rpy) - GAP * (cols - 1)) / cols);
  };
  const showRpy = w === 0 || cardW(true, 2) >= MIN_CARD_W;
  const rows = w === 0 || cardW(showRpy, 2) >= MIN_CARD_W ? 2 : 3;
  const cols = Math.ceil(items.length / rows);
  const cw = w > 0 ? cardW(showRpy, rows) : undefined;
  const blockH = CARD_H * rows + GAP * (rows - 1);
  const deg = (r: number) => `${(r * R2D).toFixed(1)}°`;
  return (
    <View style={[styles.wrap, { backgroundColor: c.panel2, borderColor: c.line, borderRadius: radius.md }]}>
      <Text style={{ color: c.dim, fontSize: 9, fontWeight: '700', letterSpacing: 0.5, marginBottom: 6 }} numberOfLines={1}>
        {t('상태 요약')} <Text style={{ fontWeight: '400' }}>{t('— 누르면 세부 데이터')}</Text>
      </Text>
      <View style={{ flexDirection: 'row', gap: ROW_GAP, alignItems: 'center' }} onLayout={(e) => setW(e.nativeEvent.layout.width)}>
        <Pressable onPress={() => onJump('imu')} style={styles.imu}>
          <AttitudeDial size={dial} roll={rpy[0] * R2D} pitch={rpy[1] * R2D}
            colors={{ sky: c.accent2, ground: c.elev, line: c.line, cross: c.text, border: c.line }} />
          {showRpy && <View style={{ justifyContent: 'center', gap: 2, width: RPY_W }}>
            {(['R', 'P', 'Y'] as const).map((a, i) => (
              <Text key={a} numberOfLines={1} style={{ fontFamily: fonts.mono, fontSize: 10, color: c.text }}>
                <Text style={{ color: c.dim }}>{a} </Text>{deg(rpy[i])}
              </Text>
            ))}
          </View>}
        </Pressable>
        <Pressable onPress={() => onJump('joy')}>
          <GamepadDiagram scale={padScale} pressed={joy.pressed} axes={joy.axes} />
        </Pressable>
        <View style={{ flex: 1, minWidth: 0, height: blockH, gap: GAP }}>
          {Array.from({ length: rows }, (_, r) => (
            <View key={r} style={{ flexDirection: 'row', gap: GAP }}>
              {items.slice(r * cols, (r + 1) * cols).map((it) => (
                <StatusChip key={it.k} item={it} onPress={onJump} width={cw} height={CARD_H} />
              ))}
            </View>
          ))}
        </View>
      </View>
    </View>
  );
}

const styles = StyleSheet.create({
  wrap: { borderWidth: 1, padding: 8 },
  imu: { flexDirection: 'row', gap: 6, alignItems: 'center' },
  chip: { paddingHorizontal: 9, paddingVertical: 5, borderRadius: 8, borderWidth: 1, minWidth: 58, gap: 1 },
});
