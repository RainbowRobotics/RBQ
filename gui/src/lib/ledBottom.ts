
export type LedSide = 'right' | 'left';
export type LedTarget = LedSide | 'both';
export type LedMode = 'off' | 'on' | 'blink';
export type Rgb = [number, number, number];
export type LedSetting = { mode: LedMode; rgb: Rgb; on_ms: number; off_ms: number; count: number };
export type LedReport = LedSetting & { actual: Rgb; limited: boolean; lpf_w: number; legacy: boolean };
export type LedBottomState = { ageMs: number; configured: boolean; right: LedReport; left: LedReport; ifFw: number };
export type LedRead = { st: LedBottomState; receivedAt: number };

export const LED_SIDES: LedSide[] = ['right', 'left'];
export const LED_POLL_MS = 1000;
export const LED_STALE_MS = 3000;
export const LED_VERIFY_MS = 1500;
export const LED_VERIFY_STEP_MS = 300;

const MODES: LedMode[] = ['off', 'on', 'blink'];
const isByte = (v: unknown): v is number => typeof v === 'number' && Number.isInteger(v) && v >= 0 && v <= 255;
const num = (v: unknown): number => (typeof v === 'number' && Number.isFinite(v) ? v : 0);

export function validRgb(v: unknown): v is Rgb {
  return Array.isArray(v) && v.length === 3 && v.every(isByte);
}

function rgbOf(v: unknown): Rgb | null {
  return validRgb(v) ? [v[0], v[1], v[2]] : null;
}

function reportOf(raw: unknown): LedReport | null {
  if (!raw || typeof raw !== 'object') return null;
  const o = raw as Record<string, unknown>;
  const mode = o.mode as LedMode;
  const rgb = rgbOf(o.rgb);
  const actual = rgbOf(o.actual);
  if (!MODES.includes(mode) || !rgb || !actual) return null;
  const blink = mode === 'blink';
  return {
    mode, rgb, on_ms: blink ? num(o.on_ms) : 0, off_ms: blink ? num(o.off_ms) : 0, count: blink ? num(o.count) : 0,
    actual, limited: o.limited === true, lpf_w: num(o.lpf_w), legacy: o.legacy === true,
  };
}

export function normalizeLedBottom(resp: unknown): LedBottomState | null {
  const b = (resp as { led_bottom?: unknown } | null | undefined)?.led_bottom;
  if (!b || typeof b !== 'object') return null;
  const o = b as Record<string, unknown>;
  const right = reportOf(o.right);
  const left = reportOf(o.left);
  if (!right || !left || typeof o.age_ms !== 'number') return null;
  const ifFw = typeof o.if_fw === 'number' && Number.isInteger(o.if_fw) && o.if_fw > 0 ? o.if_fw : 0;
  return { ageMs: o.age_ms, configured: o.configured === true, right, left, ifFw };
}

export function ledStale(r: LedRead | null | undefined, now: number): boolean {
  return !r || r.st.ageMs < 0 || now - r.receivedAt + r.st.ageMs > LED_STALE_MS;
}

export const LED_MIN_IF_FW = 261005;

export function ledOldIfFw(r: LedRead | null | undefined, now: number): number | null {
  if (!r || !ledStale(r, now)) return null;
  return r.st.ifFw > 0 && r.st.ifFw < LED_MIN_IF_FW ? r.st.ifFw : null;
}

export function ledUsable(r: LedRead | null | undefined, now: number): r is LedRead {
  return !ledStale(r, now) && !!r?.st.configured;
}

export function ledReportedAt(r: LedRead): number {
  return r.receivedAt - Math.max(0, r.st.ageMs);
}

export function ledField(s: string): number {
  return /^\s*\d+\s*$/.test(s) ? Number(s) : NaN;
}

export function roundBlinkMs(ms: number): number {
  return Math.min(2550, Math.max(10, Math.round(ms / 10) * 10));
}

export function normalizeLed(s: LedSetting): LedSetting {
  const rgb: Rgb = [s.rgb[0], s.rgb[1], s.rgb[2]];
  if (s.mode !== 'blink') return { mode: s.mode, rgb, on_ms: 0, off_ms: 0, count: 0 };
  return { mode: 'blink', rgb, on_ms: roundBlinkMs(s.on_ms), off_ms: roundBlinkMs(s.off_ms), count: s.count };
}

export function ledInputError(s: LedSetting): string | null {
  if (!validRgb(s.rgb)) return 'R·G·B는 0~255 정수로 넣어 주세요';
  if (s.mode !== 'blink') return null;
  const ms = (v: number) => Number.isInteger(v) && v >= 10 && v <= 2550;
  if (!ms(s.on_ms) || !ms(s.off_ms)) return '켜는·끄는 시간은 10~2550 ms로 넣어 주세요';
  if (!isByte(s.count)) return '반복은 0~255로 넣어 주세요';
  return null;
}

export function ledPutBody(target: LedTarget, s: LedSetting) {
  return { side: target, ...normalizeLed(s) };
}

export function sameLed(a: LedSetting, b: LedSetting): boolean {
  const x = normalizeLed(a);
  const y = normalizeLed(b);
  return x.mode === y.mode && x.rgb.every((v, i) => v === y.rgb[i])
    && x.on_ms === y.on_ms && x.off_ms === y.off_ms && x.count === y.count;
}

export type LedVerdict = 'same' | 'finished' | 'differs';
export function verifyLed(sent: LedSetting, got: LedReport): LedVerdict {
  if (got.legacy) return 'differs';
  if (sameLed(sent, got)) return 'same';
  const s = normalizeLed(sent);
  if (s.mode === 'blink' && s.count > 0 && got.mode === 'off' && got.rgb.every((v, i) => v === s.rgb[i])) return 'finished';
  return 'differs';
}

export function judgeApply(sent: LedSetting, sides: LedSide[], putAt: number, reads: LedRead[],
  before: LedRead | null | undefined): LedVerdict | 'unknown' {
  const post = reads.filter((r) => r.st.ageMs >= 0 && r.st.ageMs <= LED_STALE_MS && ledReportedAt(r) >= putAt);
  const last = post[post.length - 1];
  if (!last) return 'unknown';
  if (!last.st.configured) return 'differs';
  const v = sides.map((sd) => {
    const one = verifyLed(sent, last.st[sd]);
    if (one !== 'finished') return one;
    const already = !!before && before.st.configured && verifyLed(sent, before.st[sd]) === 'finished';
    return already ? 'differs' : 'finished';
  });
  return v.includes('differs') ? 'differs' : v.includes('finished') ? 'finished' : 'same';
}

export const LED_OFF: LedSetting = { mode: 'off', rgb: [0, 0, 0], on_ms: 0, off_ms: 0, count: 0 };

export function ledChoiceOf(s: LedSetting): string {
  if (s.mode === 'off') return 'off';
  return LED_PRESETS.find((p) => sameLed(p.s, s))?.key ?? '';
}

export const LED_PRESETS: { key: string; label: string; s: LedSetting }[] = [
  { key: 'red', label: '빨강 100% 켜 두기', s: { mode: 'on', rgb: [255, 0, 0], on_ms: 0, off_ms: 0, count: 0 } },
  { key: 'green', label: '초록 50% 켜 두기', s: { mode: 'on', rgb: [0, 127, 0], on_ms: 0, off_ms: 0, count: 0 } },
  { key: 'greenBlink', label: '초록 100% 0.8초 깜빡이기', s: { mode: 'blink', rgb: [0, 255, 0], on_ms: 800, off_ms: 800, count: 0 } },
  { key: 'blue', label: '파랑 50% 켜 두기', s: { mode: 'on', rgb: [0, 0, 127], on_ms: 0, off_ms: 0, count: 0 } },
  { key: 'blueBlink', label: '파랑 100% 0.8초 깜빡이기', s: { mode: 'blink', rgb: [0, 0, 255], on_ms: 800, off_ms: 800, count: 0 } },
  { key: 'white', label: '흰색 (R11·G25·B11%) 켜 두기', s: { mode: 'on', rgb: [28, 64, 28], on_ms: 0, off_ms: 0, count: 0 } },
];


export const LED_BODY_POS: Record<LedSide, [number, number, number]> = { right: [0, -0.09, -0.0525], left: [0, 0.09, -0.0525] };
export const LED_REF_HEIGHT = 0.4565;
export const LED_CONE_DEG = 39.4;
export const LED_TILT_DEG = 0;

export type LedTrack = { s: LedSetting; since: number };

function restarts(a: LedSetting, b: LedSetting): boolean {
  const x = normalizeLed(a);
  const y = normalizeLed(b);
  return x.mode !== y.mode || x.on_ms !== y.on_ms || x.off_ms !== y.off_ms || x.count !== y.count;
}

export function ledTrackNext(prev: LedTrack | null, rep: LedSetting, reportedAt: number, sentAt: number | null): LedTrack {
  if (prev && !restarts(prev.s, rep)) return { s: normalizeLed(rep), since: prev.since };
  return { s: normalizeLed(rep), since: sentAt ?? reportedAt };
}

export function ledShownAt(live: LedRead | null | undefined, track: LedTrack | null | undefined, side: LedSide, now: number): Rgb | null {
  if (!ledUsable(live, now)) return null;
  const rep = live.st[side];
  if (rep.mode === 'off') return null;
  const rgb: Rgb = rep.actual.some((v) => v > 0) ? rep.actual : rep.rgb;
  if (!rgb.some((v) => v > 0)) return null;
  if (rep.mode === 'on') return rgb;
  if (!track || restarts(track.s, rep)) return null;
  const period = rep.on_ms + rep.off_ms + 1;
  const t = Math.max(0, now - track.since);
  if (rep.count > 0 && Math.floor(t / period) >= rep.count) return null;
  return t % period < rep.on_ms ? rgb : null;
}

const LED_WHITE: Rgb = [28, 64, 28];

export function ledDisplay(rgb: Rgb): { color: [number, number, number]; k: number } {
  const w = rgb.map((v, i) => v / LED_WHITE[i]);
  const m = Math.max(...w);
  const duty = Math.max(...rgb) / 255;
  if (m <= 0) return { color: [0, 0, 0], k: 0 };
  return { color: [w[0] / m, w[1] / m, w[2] / m], k: 0.35 + 0.65 * Math.sqrt(duty) };
}
