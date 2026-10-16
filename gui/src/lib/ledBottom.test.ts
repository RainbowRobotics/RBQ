import { describe, it, expect } from 'vitest';
import {
  normalizeLedBottom, ledStale, ledUsable, ledField, ledInputError, ledPutBody, sameLed, verifyLed, judgeApply, roundBlinkMs,
  ledChoiceOf, ledOldIfFw, LED_OFF, LED_MIN_IF_FW, LED_PRESETS, type LedSetting, type LedReport, type LedRead,
} from './ledBottom';

const RESP = {
  led_bottom: {
    age_ms: 420,
    configured: true,
    right: { mode: 'blink', rgb: [0, 255, 0], on_ms: 800, off_ms: 800, count: 0, actual: [0, 255, 0], limited: false, lpf_w: 10.0, legacy: false },
    left: { mode: 'on', rgb: [0, 127, 0], on_ms: 0, off_ms: 0, count: 0, actual: [0, 127, 0], limited: false, lpf_w: 10.0, legacy: false },
    if_fw: 261006,
  },
};

const set = (p: Partial<LedSetting>): LedSetting => ({ mode: 'on', rgb: [0, 127, 0], on_ms: 0, off_ms: 0, count: 0, ...p });
const rep = (p: Partial<LedReport>): LedReport => ({ ...set({}), actual: [0, 0, 0], limited: false, lpf_w: 0, legacy: false, ...p });

describe('normalizeLedBottom', () => {
  it('계약 예시를 그대로 읽는다', () => {
    const st = normalizeLedBottom(RESP)!;
    expect(st.ageMs).toBe(420);
    expect(st.configured).toBe(true);
    expect(st.right).toMatchObject({ mode: 'blink', rgb: [0, 255, 0], on_ms: 800, off_ms: 800, count: 0, lpf_w: 10 });
    expect(st.left).toMatchObject({ mode: 'on', actual: [0, 127, 0], limited: false, legacy: false });
    expect(st.ifFw).toBe(261006);
  });
  it('if_fw 가 없거나(그 전 로봇 소프트웨어) 정수가 아니면 0(모름)', () => {
    expect(normalizeLedBottom({ led_bottom: { ...RESP.led_bottom, if_fw: undefined } })!.ifFw).toBe(0);
    expect(normalizeLedBottom({ led_bottom: { ...RESP.led_bottom, if_fw: '261003' } })!.ifFw).toBe(0);
    expect(normalizeLedBottom({ led_bottom: { ...RESP.led_bottom, if_fw: 2.5 } })!.ifFw).toBe(0);
  });
  it('모양이 다르면 null — 시뮬의 {result:"ok"}·한쪽 누락·모르는 동작·rgb 범위 밖', () => {
    expect(normalizeLedBottom({ result: 'ok' })).toBeNull();
    expect(normalizeLedBottom(undefined)).toBeNull();
    expect(normalizeLedBottom({ led_bottom: { ...RESP.led_bottom, left: undefined } })).toBeNull();
    expect(normalizeLedBottom({ led_bottom: { ...RESP.led_bottom, right: { ...RESP.led_bottom.right, mode: 'flash' } } })).toBeNull();
    expect(normalizeLedBottom({ led_bottom: { ...RESP.led_bottom, right: { ...RESP.led_bottom.right, rgb: [0, 256, 0] } } })).toBeNull();
    expect(normalizeLedBottom({ led_bottom: { ...RESP.led_bottom, age_ms: undefined } })).toBeNull();
  });
  it('깜빡이기가 아니면 시간·횟수는 0 으로 읽는다', () => {
    const st = normalizeLedBottom({ led_bottom: { ...RESP.led_bottom, left: { ...RESP.led_bottom.left, on_ms: 5, count: 3 } } })!;
    expect(st.left).toMatchObject({ on_ms: 0, off_ms: 0, count: 0 });
  });
});

describe('ledStale — 읽지 못함', () => {
  const st = normalizeLedBottom(RESP)!;
  const at = (ageMs: number, receivedAt = 10_000): LedRead => ({ st: { ...st, ageMs }, receivedAt });
  it('응답 없음 · 한 번도 못 받음(-1) · IF 가 알려 준 지 3초 넘음', () => {
    expect(ledStale(null, 10_000)).toBe(true);
    expect(ledStale(at(-1), 10_000)).toBe(true);
    expect(ledStale(at(3001), 10_000)).toBe(true);
    expect(ledStale(at(3000), 10_000)).toBe(false);
    expect(ledStale(at(420), 10_000)).toBe(false);
  });
  it('GET 이 답하지 않는 동안에도 나이를 지금 기준으로 다시 잰다', () => {
    expect(ledStale(at(420), 12_580)).toBe(false);
    expect(ledStale(at(420), 12_581)).toBe(true);
  });
  it('쓸 수 있는 읽기 = 3초 안 + 설정 있음', () => {
    expect(ledUsable(at(420), 10_000)).toBe(true);
    expect(ledUsable({ st: { ...st, configured: false }, receivedAt: 10_000 }, 10_000)).toBe(false);
  });
});

describe('ledOldIfFw — 옛 IF 펌웨어 안내(계약 ③ if_fw)', () => {
  const st = normalizeLedBottom(RESP)!;
  const at = (ageMs: number, ifFw: number): LedRead => ({ st: { ...st, ageMs, configured: ageMs >= 0, ifFw }, receivedAt: 10_000 });
  it('읽지 못함이고 버전이 0xB6 보다 옛것이면 그 버전', () => {
    expect(LED_MIN_IF_FW).toBe(261005);
    expect(ledOldIfFw(at(-1, 261003), 10_000)).toBe(261003);
    expect(ledOldIfFw(at(5200, 261004), 10_000)).toBe(261004);
  });
  it('새 펌웨어인데 답이 없거나(STANDBY) 버전을 모르면 null — 그때는 읽지 못함 그대로', () => {
    expect(ledOldIfFw(at(-1, 261005), 10_000)).toBeNull();
    expect(ledOldIfFw(at(-1, 261007), 10_000)).toBeNull();
    expect(ledOldIfFw(at(-1, 0), 10_000)).toBeNull();
    expect(ledOldIfFw(null, 10_000)).toBeNull();
  });
  it('읽고 있는 동안에는 지난 버전이 옛것이어도 null — 업데이트 뒤 남은 값으로 안내하지 않는다', () => {
    expect(ledOldIfFw(at(400, 261003), 10_000)).toBeNull();
  });
});

describe('ledField — 입력 칸', () => {
  it('숫자만 받는다 — 빈 칸·공백·소수·지수는 NaN', () => {
    expect(ledField('12')).toBe(12);
    expect(ledField(' 7 ')).toBe(7);
    for (const bad of ['', ' ', '1.5', '1e2', '-3', 'abc']) expect(ledField(bad)).toBeNaN();
    expect(ledInputError(set({ rgb: [ledField(''), 0, 0] }))).not.toBeNull();
    expect(ledInputError(set({ mode: 'blink', on_ms: 800, off_ms: 800, count: ledField('') }))).not.toBeNull();
  });
});

describe('입력 검사와 PUT 바디', () => {
  it('범위(계약 ③ PUT) — rgb 0~255 정수, 깜빡이기 시간 10~2550, 횟수 0~255', () => {
    expect(ledInputError(set({}))).toBeNull();
    expect(ledInputError(set({ rgb: [0, 256, 0] }))).not.toBeNull();
    expect(ledInputError(set({ rgb: [0, 1.5, 0] }))).not.toBeNull();
    expect(ledInputError(set({ rgb: [0, NaN, 0] }))).not.toBeNull();
    expect(ledInputError(set({ mode: 'blink', on_ms: 800, off_ms: 800 }))).toBeNull();
    expect(ledInputError(set({ mode: 'blink', on_ms: 9, off_ms: 800 }))).not.toBeNull();
    expect(ledInputError(set({ mode: 'blink', on_ms: 800, off_ms: 2551 }))).not.toBeNull();
    expect(ledInputError(set({ mode: 'blink', on_ms: 800, off_ms: 800, count: 256 }))).not.toBeNull();
    expect(ledInputError(set({ mode: 'on', on_ms: 0, count: 999 }))).toBeNull();
  });
  it('바디 — 깜빡이기가 아니면 시간·횟수 0, 깜빡임 시간은 10 ms 단위', () => {
    expect(ledPutBody('both', set({ on_ms: 500, count: 9 }))).toEqual({ side: 'both', mode: 'on', rgb: [0, 127, 0], on_ms: 0, off_ms: 0, count: 0 });
    expect(ledPutBody('right', set({ mode: 'blink', on_ms: 804, off_ms: 1005, count: 3 })))
      .toEqual({ side: 'right', mode: 'blink', rgb: [0, 127, 0], on_ms: 800, off_ms: 1010, count: 3 });
    expect(roundBlinkMs(2554)).toBe(2550);
    expect(roundBlinkMs(1)).toBe(10);
  });
});

describe('verifyLed — [적용] 확인', () => {
  const blink = set({ mode: 'blink', rgb: [0, 255, 0], on_ms: 800, off_ms: 800 });
  it('같은 설정이면 same — 깜빡임 시간은 IF 의 10 ms 단위로 비교한다', () => {
    expect(verifyLed(blink, rep(blink))).toBe('same');
    expect(verifyLed({ ...blink, on_ms: 803 }, rep(blink))).toBe('same');
    expect(sameLed(set({ count: 5 }), set({}))).toBe(true);
  });
  it('옛 값을 보면 differs — 곧바로 한 번만 읽으면 이게 나온다', () => {
    expect(verifyLed(blink, rep(set({})))).toBe('differs');
    expect(verifyLed(blink, rep({ ...blink, rgb: [0, 254, 0] }))).toBe('differs');
  });
  it('횟수 있는 깜빡임이 벌써 끝나 꺼 두기(색은 남음)면 finished — 계약 ① 규칙 5', () => {
    const three = { ...blink, count: 3 };
    expect(verifyLed(three, rep({ mode: 'off', rgb: [0, 255, 0] }))).toBe('finished');
    expect(verifyLed(three, rep({ mode: 'off', rgb: [0, 0, 0] }))).toBe('differs');
    expect(verifyLed(blink, rep({ mode: 'off', rgb: [0, 255, 0] }))).toBe('differs');
  });
  it('옛 명령(0xB5)으로 바뀌어 있으면 값이 같아 보여도 differs', () => {
    expect(verifyLed(blink, rep({ ...blink, legacy: true }))).toBe('differs');
  });
});

describe('judgeApply — [적용] 판정은 PUT 뒤에 IF 가 알려 준 읽기로만', () => {
  const putAt = 100_000;
  const base = normalizeLedBottom(RESP)!;
  const readAt = (dt: number, right: LedReport, age = 50, configured = true): LedRead =>
    ({ st: { ...base, ageMs: age, configured, right }, receivedAt: putAt + dt + age });
  const sent = set({ rgb: [255, 0, 0] });
  const old = rep(set({}));
  it('PUT 전의 읽기는 빼고, PUT 뒤 읽기가 없으면 unknown', () => {
    expect(judgeApply(sent, ['right'], putAt, [readAt(-200, rep(sent))], null)).toBe('unknown');
    expect(judgeApply(sent, ['right'], putAt, [], null)).toBe('unknown');
  });
  it('마지막 유효 읽기로 정한다 — 옛 값을 본 뒤 새 값이면 same, 그 반대면 differs', () => {
    expect(judgeApply(sent, ['right'], putAt, [readAt(20, old), readAt(320, rep(sent))], null)).toBe('same');
    expect(judgeApply(sent, ['right'], putAt, [readAt(20, rep(sent)), readAt(320, old)], null)).toBe('differs');
  });
  it('유효한 읽기 뒤에 3초 넘은 읽기만 오면 그 앞 읽기로, 처음부터 3초 넘었으면 unknown', () => {
    expect(judgeApply(sent, ['right'], putAt, [readAt(20, old, 5200)], null)).toBe('unknown');
    expect(judgeApply(sent, ['right'], putAt, [readAt(20, rep(sent)), readAt(320, old, 5200)], null)).toBe('same');
  });
  it('설정 없음(IF 재부팅)이면 differs', () => {
    expect(judgeApply(sent, ['right'], putAt, [readAt(20, rep(sent), 50, false)], null)).toBe('differs');
  });
  it('finished 는 PUT 직전에 이미 "꺼 두기 + 같은 색"이 아니었던 LED 만 — 반영 안 된 옛 끝난 깜빡임을 성공으로 읽지 않는다', () => {
    const three = set({ mode: 'blink', rgb: [0, 255, 0], on_ms: 100, off_ms: 100, count: 3 });
    const done = rep({ mode: 'off', rgb: [0, 255, 0] });
    const wasDone = readAt(-500, done);
    const wasOn = readAt(-500, rep(set({})));
    expect(judgeApply(three, ['right'], putAt, [readAt(300, done), readAt(900, done)], wasDone)).toBe('differs');
    expect(judgeApply(three, ['right'], putAt, [readAt(300, done)], wasOn)).toBe('finished');
    expect(judgeApply(three, ['right'], putAt, [readAt(300, done)], null)).toBe('finished');
    expect(judgeApply(three, ['right'], putAt, [readAt(300, rep(three))], wasDone)).toBe('same');
  });
  it('양쪽이면 둘 다 맞아야 same', () => {
    const r: LedRead = { st: { ...base, ageMs: 50, right: rep(sent), left: old }, receivedAt: putAt + 400 };
    expect(judgeApply(sent, ['right', 'left'], putAt, [r], null)).toBe('differs');
    expect(judgeApply(sent, ['right'], putAt, [r], null)).toBe('same');
  });
});

describe('ledChoiceOf — 일반 사용자(L1) 목록의 어느 항목인가', () => {
  it('꺼 두기는 색과 무관하게 off, 추천 설정은 그 key, 목록에 없는 값은 ""', () => {
    expect(ledChoiceOf(LED_OFF)).toBe('off');
    expect(ledChoiceOf(set({ mode: 'off', rgb: [9, 9, 9] }))).toBe('off');
    expect(ledChoiceOf(set({ rgb: [0, 127, 0] }))).toBe('green');
    expect(ledChoiceOf(set({ mode: 'blink', rgb: [0, 255, 0], on_ms: 804, off_ms: 800 }))).toBe('greenBlink');
    expect(ledChoiceOf(set({ mode: 'blink', rgb: [0, 255, 0], on_ms: 800, off_ms: 800, count: 3 }))).toBe('');
    expect(ledChoiceOf(set({ rgb: [255, 128, 0] }))).toBe('');
    for (const p of LED_PRESETS) expect(ledChoiceOf(p.s)).toBe(p.key);
  });
});

describe('추천 설정 — 결정표(2026-08-18 "RBQ 실 적용")', () => {
  it('값이 결정표와 같고 입력 검사를 통과한다', () => {
    const by = Object.fromEntries(LED_PRESETS.map((p) => [p.key, p.s]));
    expect(by.red).toMatchObject({ mode: 'on', rgb: [255, 0, 0] });
    expect(by.green).toMatchObject({ mode: 'on', rgb: [0, 127, 0] });
    expect(by.blue).toMatchObject({ mode: 'on', rgb: [0, 0, 127] });
    expect(by.greenBlink).toMatchObject({ mode: 'blink', rgb: [0, 255, 0], on_ms: 800, off_ms: 800, count: 0 });
    expect(by.blueBlink).toMatchObject({ mode: 'blink', rgb: [0, 0, 255], on_ms: 800, off_ms: 800, count: 0 });
    expect(by.white).toMatchObject({ mode: 'on', rgb: [28, 64, 28] });
    for (const p of LED_PRESETS) expect(ledInputError(p.s)).toBeNull();
    expect(new Set(LED_PRESETS.map((p) => p.key)).size).toBe(LED_PRESETS.length);
  });
});
