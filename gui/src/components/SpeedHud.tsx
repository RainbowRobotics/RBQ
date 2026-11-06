import { useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { useTelemetry } from '@/store/telemetry';
import { useSettings } from '@/store/settings';

const TICK_MS = 250;
const ALPHA = 0.35;
const wrapPi = (a: number) => ((a + Math.PI) % (2 * Math.PI) + 2 * Math.PI) % (2 * Math.PI) - Math.PI;

function useDerivedSpeed() {
  const [v, setV] = useState({ speed: 0, yawRate: 0, live: false });
  const prev = useRef<{ t: number; x: number; y: number; yaw: number } | null>(null);
  const ema = useRef({ speed: 0, yawRate: 0 });
  useEffect(() => {
    const t = setInterval(() => {
      const tel = useTelemetry.getState().robot;
      if (!tel) { prev.current = null; setV((s) => (s.live ? { speed: 0, yawRate: 0, live: false } : s)); return; }
      const now = Date.now();
      const cur = { t: now, x: tel.worldPos[0], y: tel.worldPos[1], yaw: tel.imu.rpy[2] };
      const p = prev.current;
      prev.current = cur;
      if (!p || cur.t - p.t <= 0 || cur.t - p.t > 2000) return;
      const dt = (cur.t - p.t) / 1000;
      const speed = Math.hypot(cur.x - p.x, cur.y - p.y) / dt;
      const yawRate = (wrapPi(cur.yaw - p.yaw) / dt) * (180 / Math.PI);
      ema.current.speed += ALPHA * (speed - ema.current.speed);
      ema.current.yawRate += ALPHA * (yawRate - ema.current.yawRate);
      setV({ speed: ema.current.speed, yawRate: ema.current.yawRate, live: true });
    }, TICK_MS);
    return () => clearInterval(t);
  }, []);
  return v;
}

function Item({ v, k, tone, dense }: { v: string; k: string; tone?: string; dense?: boolean }) {
  const { c, fonts } = useTheme();
  return (
    <View style={[styles.item, dense && styles.itemDense]}>
      <Text style={{ color: tone ?? c.accent2, fontFamily: fonts.mono, fontSize: dense ? 11 : 13, fontWeight: '600' }}>{v}</Text>
      <Text style={{ color: c.dim, fontSize: dense ? 6.5 : 7.5, fontWeight: '700', letterSpacing: 0.4 }}>{k}</Text>
    </View>
  );
}

export function SpeedHud({ dense }: { dense?: boolean } = {}) {
  const { c, radius } = useTheme();
  const { speed, yawRate, live } = useDerivedSpeed();
  const maxPct = useSettings((s) => s.walk.max_speed);
  return (
    <View style={[styles.pill, dense && styles.pillDense, { backgroundColor: c.glass, borderColor: c.glassLine, borderRadius: radius.md }]} pointerEvents="none">
      <Item dense={dense} v={live ? speed.toFixed(2) : '—'} k="M/S" />
      <View style={[styles.sep, dense && styles.sepDense, { backgroundColor: c.line }]} />
      <Item dense={dense} v={live ? `${yawRate >= 0 ? '+' : ''}${yawRate.toFixed(0)}` : '—'} k="YAW °/S" />
      <View style={[styles.sep, dense && styles.sepDense, { backgroundColor: c.line }]} />
      <Item dense={dense} v={`${maxPct}%`} k="MAX" tone={undefined} />
    </View>
  );
}

const styles = StyleSheet.create({
  pill: { flexDirection: 'row', alignItems: 'center', gap: 12, paddingHorizontal: 14, paddingVertical: 5, borderWidth: 1 },
  pillDense: { gap: 7, paddingHorizontal: 9, paddingVertical: 2 },
  item: { alignItems: 'center', minWidth: 44 },
  itemDense: { minWidth: 32 },
  sep: { width: 1, height: 20 },
  sepDense: { height: 15 },
});
