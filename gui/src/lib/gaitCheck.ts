import { useCallback, useEffect, useRef, useState } from 'react';
import { useTelemetry } from '@/store/telemetry';
import type { RobotState } from '@/lib/robotState';

export const IMU_STILL_MS = 3000;
export const ACC_NORM_MIN = 9.75;
export const ACC_NORM_MAX = 9.85;
export const GYRO_BIAS_MAX_DPS = 1.0;
export const LEVEL_MS = 2000;
export const LEVEL_MAX_DEG = 0.3;
const MIN_SAMPLES = 10;
const R2D = 180 / Math.PI;

export type SampleState =
  | { phase: 'idle' }
  | { phase: 'measuring'; progress: number }
  | { phase: 'done'; mean: number[] }
  | { phase: 'failed' };

export function useSampleAverage(ms: number, pick: (r: RobotState) => number[]): { state: SampleState; start: () => void } {
  const [state, setState] = useState<SampleState>({ phase: 'idle' });
  const pickRef = useRef(pick);
  pickRef.current = pick;
  const timers = useRef<{ end?: ReturnType<typeof setTimeout>; tick?: ReturnType<typeof setInterval>; unsub?: () => void }>({});
  const stop = useCallback(() => {
    const tm = timers.current;
    if (tm.end) clearTimeout(tm.end);
    if (tm.tick) clearInterval(tm.tick);
    tm.unsub?.();
    timers.current = {};
  }, []);
  useEffect(() => stop, [stop]);

  const start = useCallback(() => {
    stop();
    const p = pickRef.current;
    const cols: number[][] = [];
    let n = 0;
    let last: RobotState | undefined = useTelemetry.getState().robot;
    const take = (r: RobotState | undefined) => {
      if (!r || r === last) return;
      last = r;
      p(r).forEach((v, i) => (cols[i] ??= []).push(v));
      n++;
    };
    const t0 = Date.now();
    timers.current.unsub = useTelemetry.subscribe((s) => take(s.robot));
    timers.current.tick = setInterval(
      () => setState({ phase: 'measuring', progress: Math.min(1, (Date.now() - t0) / ms) }), 100);
    setState({ phase: 'measuring', progress: 0 });
    timers.current.end = setTimeout(() => {
      stop();
      if (n < MIN_SAMPLES) { setState({ phase: 'failed' }); return; }
      setState({ phase: 'done', mean: cols.map((xs) => xs.reduce((a, b) => a + b, 0) / xs.length) });
    }, ms);
  }, [ms, stop]);

  return { state, start };
}

export const pickImuStill = (r: RobotState) => [Math.hypot(...r.imu.acc), ...r.imu.gyro.map((g) => g * R2D)];
export const pickLevel = (r: RobotState) => [r.imu.rpy[0] * R2D, r.imu.rpy[1] * R2D];
