import { useCallback, useEffect, useRef, useState } from 'react';
import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { Modal } from '@/components/ui/overlays';
import { Segmented, Select } from '@/components/ui/controls';
import { EditField, SBtn } from '@/components/panels/settings/common';
import { RBCheckBox } from '@/rb/components/RBCheckBox';
import { TRow } from '@/components/panels/PowerPanel';
import { useRobot } from '@/store/robot';
import { useDevMode } from '@/store/settings';
import { useLedBottom, type LedKnown } from '@/store/ledBottom';
import { actions } from '@/lib/rest';
import { useFocusedInterval } from '@/lib/useFocusedInterval';
import { t } from '@/lib/i18n';
import {
  normalizeLedBottom, normalizeLed, ledStale, ledUsable, ledReportedAt, ledField, ledInputError, ledPutBody, judgeApply,
  ledChoiceOf, ledOldIfFw, validRgb, LED_OFF, LED_MIN_IF_FW, LED_PRESETS, LED_SIDES, LED_POLL_MS, LED_VERIFY_MS, LED_VERIFY_STEP_MS,
  type LedRead, type LedMode, type LedSetting, type LedSide, type LedVerdict, type Rgb,
} from '@/lib/ledBottom';

const SIDE_NAME: Record<LedSide, string> = { right: '오른쪽 LED', left: '왼쪽 LED' };
const MODE_WORD: Record<LedMode, string> = { off: '꺼짐', on: '켜짐', blink: '깜빡임' };
const UNSUPPORTED = '로봇 소프트웨어에 이 기능이 없습니다. 업데이트가 필요합니다';
const NOT_SET = '설정 없음 — IF가 재부팅한 뒤 아직 설정을 받지 못했습니다';
const oldIfText = (v: number) =>
  t('IF 펌웨어 업데이트가 필요합니다 — 지금 {v}, {min} 이상이어야 합니다').replace('{v}', String(v)).replace('{min}', String(LED_MIN_IF_FW));
const LED_UNSUPPORTED_POLL_MS = 30_000;

function describe(s: LedSetting): string {
  if (s.mode === 'off') return t(MODE_WORD.off);
  const base = `${t(MODE_WORD[s.mode])} · R${s.rgb[0]} G${s.rgb[1]} B${s.rgb[2]}`;
  if (s.mode !== 'blink') return base;
  const times = t('{on}초/{off}초').replace('{on}', String(s.on_ms / 1000)).replace('{off}', String(s.off_ms / 1000));
  const times2 = s.count === 0 ? t('계속 반복') : t('{n}번 반복').replace('{n}', String(s.count));
  return `${base} · ${times} · ${times2}`;
}

function knownText(k: LedKnown): string {
  if (!k) return '';
  return `${t(k.from === 'robot' ? '마지막으로 읽은 값' : '이 기기가 마지막으로 보낸 값')}: ${describe(k.s)}`;
}

function statusOf(e: unknown): number {
  const s = (e as { status?: unknown } | null)?.status;
  return typeof s === 'number' ? s : 0;
}

function Swatch({ rgb }: { rgb: Rgb | null }) {
  const { c } = useTheme();
  return (
    <View style={{ width: 20, height: 20, borderRadius: 5, borderWidth: 1, borderColor: c.line2,
      backgroundColor: rgb ? `rgb(${rgb[0]},${rgb[1]},${rgb[2]})` : 'transparent' }} />
  );
}

function useLedBottomPoll() {
  const ip = useRobot((s) => s.ip);
  const remember = useLedBottom((s) => s.remember);
  const [read, setRead] = useState<LedRead | null | undefined>(undefined);
  const [unsupported, setUnsupported] = useState(false);
  const [, setTick] = useState(0);
  const gen = useRef(0);
  const busy = useRef(false);
  useEffect(() => { gen.current++; busy.current = false; setRead(undefined); setUnsupported(false); }, [ip]);
  const refresh = useCallback(async (): Promise<LedRead | null> => {
    const g = gen.current;
    let next: LedRead | null = null;
    let missing = false;
    try {
      const st = normalizeLedBottom(await actions.ledBottom(ip));
      next = st ? { st, receivedAt: Date.now() } : null;
    } catch (e) {
      missing = statusOf(e) === 404;
    }
    if (g !== gen.current) return null;
    setRead(next);
    if (missing) setUnsupported(true);
    else if (next) setUnsupported(false);
    if (ledUsable(next, Date.now())) for (const sd of LED_SIDES) remember(sd, next.st[sd], 'robot', ledReportedAt(next));
    return next;
  }, [ip, remember]);
  const lastTry = useRef(0);
  useFocusedInterval(() => {
    setTick((n) => n + 1);
    if (busy.current) return;
    if (unsupported && Date.now() - lastTry.current < LED_UNSUPPORTED_POLL_MS) return;
    lastTry.current = Date.now();
    busy.current = true;
    const g = gen.current;
    refresh().finally(() => { if (g === gen.current) busy.current = false; });
  }, LED_POLL_MS, true, () => { gen.current++; busy.current = false; setRead(undefined); });
  return { read, unsupported, refresh };
}

export function LedBottomGroup() {
  const { c, fonts, radius } = useTheme();
  const { read, unsupported, refresh } = useLedBottomPoll();
  const known = { right: useLedBottom((s) => s.right), left: useLedBottom((s) => s.left) };
  const [editing, setEditing] = useState<LedSide | null>(null);
  const now = Date.now();

  const row = (sd: LedSide) => {
    const rep = ledUsable(read, now) ? read.st[sd] : null;
    let sub: string;
    let warn = false;
    const oldIf = ledOldIfFw(read, now);
    if (unsupported) { sub = t(UNSUPPORTED); warn = true; }
    else if (read === undefined) sub = t('읽는 중…');
    else if (oldIf !== null) { sub = oldIfText(oldIf); warn = true; }
    else if (read === null || ledStale(read, now)) { sub = [t('읽지 못함'), knownText(known[sd])].filter(Boolean).join(' — '); warn = true; }
    else if (!read.st.configured) sub = t(NOT_SET);
    else sub = `${describe(read.st[sd])}${read.st[sd].legacy ? ` · ${t('옛 명령(0xB5)으로 설정됨 — 근삿값')}` : ''}`;
    return (
      <TRow key={sd} nm={t(SIDE_NAME[sd])} sub={sub} subColor={warn ? c.amber : undefined}
        right={<View style={{ flexDirection: 'row', alignItems: 'center', gap: 8 }}>
          {rep?.limited && <Text style={{ color: c.redbright, fontSize: 10.5, fontWeight: '700' }}>{t('제한 중')}</Text>}
          {rep && <Text style={{ color: c.text, fontSize: 10.5, fontFamily: fonts.mono }}>{`${rep.lpf_w.toFixed(1)} W`}</Text>}
          <Swatch rgb={rep ? rep.actual : null} />
          <Tappable onPress={() => setEditing(sd)}
            style={[styles.action, { borderRadius: radius.sm, borderColor: c.line2, backgroundColor: c.panel2 }]}>
            <Text style={{ color: c.text, fontSize: 11, fontWeight: '700' }}>{t('설정')}</Text>
          </Tappable>
        </View>} />
    );
  };

  return (
    <View>
      <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginTop: 10, marginBottom: 2 }}>
        {t('하단 상태 LED (IF 보드)')}
      </Text>
      {row('right')}
      {row('left')}
      {editing && (
        <LedEditModal side={editing} read={read} unsupported={unsupported} known={known[editing]} refresh={refresh}
          onClose={() => setEditing(null)} />
      )}
    </View>
  );
}

const DEFAULT_SETTING: LedSetting = { mode: 'off', rgb: [0, 0, 0], on_ms: 800, off_ms: 800, count: 0 };
type Msg = { text: string; ok: boolean };

function putErrorText(e: unknown): string {
  const status = statusOf(e);
  if (status === 404) return t(UNSUPPORTED);
  if (status === 0) return t('적용하지 못했습니다. 연결을 확인해 주세요');
  return `${t('적용하지 못했습니다')} (HTTP ${status})`;
}

const VERDICT_MSG: Record<LedVerdict | 'unknown', Msg> = {
  same: { text: '적용했습니다 — 로봇 값과 일치합니다', ok: true },
  finished: { text: '적용했습니다 — 횟수만큼 깜빡이고 벌써 꺼졌습니다', ok: true },
  differs: { text: '로봇에 설정된 값이 보낸 값과 다릅니다. [다시 읽기]로 확인해 주세요', ok: false },
  unknown: { text: '보냈지만 로봇에서 값을 읽지 못해 결과를 확인하지 못했습니다', ok: false },
};

function LedEditModal({ side, read, unsupported, known, refresh, onClose }: {
  side: LedSide; read: LedRead | null | undefined; unsupported: boolean; known: LedKnown;
  refresh: () => Promise<LedRead | null>; onClose: () => void;
}) {
  const { c } = useTheme();
  const ip = useRobot((s) => s.ip);
  const remember = useLedBottom((s) => s.remember);
  const free = useDevMode();
  const [both, setBoth] = useState(false);
  const [mode, setMode] = useState<LedMode>('off');
  const [r, setR] = useState('0');
  const [g, setG] = useState('0');
  const [b, setB] = useState('0');
  const [onMs, setOnMs] = useState('800');
  const [offMs, setOffMs] = useState('800');
  const [count, setCount] = useState('0');
  const [msg, setMsg] = useState<Msg | null>(null);
  const [busy, setBusy] = useState(false);
  const alive = useRef(true);
  useEffect(() => { alive.current = true; return () => { alive.current = false; }; }, []);

  const fill = (s: LedSetting) => {
    setMode(s.mode);
    setR(String(s.rgb[0])); setG(String(s.rgb[1])); setB(String(s.rgb[2]));
    if (s.mode === 'blink') { setOnMs(String(s.on_ms)); setOffMs(String(s.off_ms)); setCount(String(s.count)); }
  };
  const fillFrom = (cur: LedRead | null | undefined): Msg | null => {
    const now = Date.now();
    if (ledUsable(cur, now)) { fill(cur.st[side]); return null; }
    fill(known?.s ?? DEFAULT_SETTING);
    if (unsupported) return { text: t(UNSUPPORTED), ok: false };
    const oldIf = ledOldIfFw(cur, now);
    if (oldIf !== null) return { text: oldIfText(oldIf), ok: false };
    if (cur && !ledStale(cur, now)) {
      return { text: known ? t('IF가 재부팅한 뒤 아직 설정을 받지 못했습니다 — 마지막으로 알던 값을 채웠습니다')
        : t('IF가 재부팅한 뒤 아직 설정을 받지 못했습니다'), ok: false };
    }
    return known
      ? { text: t('로봇에서 값을 읽지 못해 마지막으로 알던 값을 채웠습니다'), ok: false }
      : { text: t('로봇에서 값을 읽지 못했고, 이 기기에도 기록이 없습니다'), ok: false };
  };
  const reload = async () => {
    setBusy(true);
    const cur = await refresh();
    if (!alive.current) return;
    setMsg(fillFrom(cur));
    setBusy(false);
  };
  useEffect(() => { if (read === undefined) reload(); else setMsg(fillFrom(read)); }, []); // eslint-disable-line react-hooks/exhaustive-deps

  const applyPreset = (k: string) => {
    const p = LED_PRESETS.find((x) => x.key === k);
    if (p) fill(p.s);
  };

  const rgb: Rgb = [ledField(r), ledField(g), ledField(b)];
  const blink = mode === 'blink';
  const choice = ledChoiceOf({ mode, rgb, on_ms: ledField(onMs), off_ms: ledField(offMs), count: ledField(count) });
  const robotNow = ledUsable(read, Date.now()) ? read.st[side] : null;
  const oldIf = ledOldIfFw(read, Date.now()) !== null;

  const apply = async () => {
    const sent: LedSetting = { mode, rgb, on_ms: ledField(onMs), off_ms: ledField(offMs), count: ledField(count) };
    const err = ledInputError(sent);
    if (err) { setMsg({ text: t(err), ok: false }); return; }
    const sides = both ? LED_SIDES : [side];
    const before = ledUsable(read, Date.now()) ? read : null;
    setBusy(true); setMsg(null);
    try {
      await actions.ledBottomSet(ip, ledPutBody(both ? 'both' : side, sent));
    } catch (e) {
      if (alive.current) { setMsg({ text: putErrorText(e), ok: false }); setBusy(false); }
      return;
    }
    const putAt = Date.now();
    for (const sd of sides) remember(sd, sent, 'sent', putAt);
    const reads: LedRead[] = [];
    let verdict: LedVerdict | 'unknown' = 'unknown';
    for (;;) {
      await new Promise((res) => setTimeout(res, LED_VERIFY_STEP_MS));
      if (!alive.current) return;
      const cur = await refresh();
      if (!alive.current) return;
      if (cur) reads.push(cur);
      verdict = judgeApply(sent, sides, putAt, reads, before);
      if (verdict === 'same' || verdict === 'finished' || Date.now() - putAt >= LED_VERIFY_MS) break;
    }
    if (verdict === 'same') fill(normalizeLed(sent));
    setMsg({ text: t(VERDICT_MSG[verdict].text), ok: VERDICT_MSG[verdict].ok });
    setBusy(false);
  };

  const label = (s: string) => <Text style={{ color: c.dim, fontSize: 11, fontWeight: '700', marginBottom: 6, marginTop: 12 }}>{s}</Text>;
  return (
    <Modal onClose={onClose}>
      <View style={{ width: 560, maxWidth: '94%', borderRadius: 14, padding: 18, backgroundColor: c.panel }}>
        <View style={{ flexDirection: 'row', alignItems: 'center' }}>
          <Text style={{ color: c.text, fontSize: 15, fontWeight: '700', flex: 1 }}>{t(SIDE_NAME[side])} {t('설정')}</Text>
          <Tappable onPress={onClose} accessibilityLabel={t('닫기')}><Icon name="x" size={18} color={c.dim} /></Tappable>
        </View>

        {free ? (
          <>
            {label(t('동작'))}
            <Segmented value={mode} onChange={setMode}
              options={[{ key: 'off', label: t('꺼 두기') }, { key: 'on', label: t('켜 두기') }, { key: 'blink', label: t('깜빡이기') }]} />

            {label(t('추천 설정'))}
            <Select label={t('RBQ에 쓰기로 정한 값 고르기')} value={choice === 'off' ? '' : choice} onChange={applyPreset}
              options={LED_PRESETS.map((p) => ({ key: p.key, label: t(p.label) }))} />

            <View style={{ flexDirection: 'row', gap: 10, alignItems: 'center', marginTop: 14 }}>
              <EditField label="R" value={r} onChangeText={(v) => { setR(v); }} keyboardType="number-pad" />
              <EditField label="G" value={g} onChangeText={(v) => { setG(v); }} keyboardType="number-pad" />
              <EditField label="B" value={b} onChangeText={(v) => { setB(v); }} keyboardType="number-pad" />
              <View style={{ marginBottom: 14 }}><Swatch rgb={mode !== 'off' && validRgb(rgb) ? rgb : null} /></View>
            </View>
            {blink && (
              <View style={{ flexDirection: 'row', gap: 10 }}>
                <EditField label={t('켜는 시간 (ms)')} value={onMs} onChangeText={(v) => { setOnMs(v); }} keyboardType="number-pad" />
                <EditField label={t('끄는 시간 (ms)')} value={offMs} onChangeText={(v) => { setOffMs(v); }} keyboardType="number-pad" />
                <EditField label={t('반복 (0이면 계속)')} value={count} onChangeText={(v) => { setCount(v); }} keyboardType="number-pad" />
                <View style={{ width: 20 }} />
              </View>
            )}
          </>
        ) : (
          <>
            {label(t('추천 설정'))}
            <View style={{ flexDirection: 'row', gap: 10, alignItems: 'center' }}>
              <View style={{ flex: 1 }}>
                <Select label={t('RBQ에 쓰기로 정한 값 고르기')} value={choice}
                  onChange={(k) => (k === 'off' ? fill(LED_OFF) : applyPreset(k))}
                  options={[{ key: 'off', label: t('꺼 두기') }, ...LED_PRESETS.map((p) => ({ key: p.key, label: t(p.label) }))]} />
              </View>
              <Swatch rgb={mode !== 'off' && validRgb(rgb) ? rgb : null} />
            </View>
            {robotNow && (
              <Text style={{ color: c.dim, fontSize: 11, marginTop: 8 }}>{`${t('지금 설정')}: ${describe(robotNow)}`}</Text>
            )}
            <Text style={{ color: c.dim, fontSize: 10, marginTop: 6, marginBottom: 10 }}>
              {t('R·G·B와 시간을 직접 넣는 것은 개발자 모드에서 할 수 있습니다.')}
            </Text>
          </>
        )}

        {msg && <Text style={{ color: msg.ok ? c.greenTx : c.amber, fontSize: 11, marginTop: 4, marginBottom: 8 }}>{msg.text}</Text>}
        <View style={{ flexDirection: 'row', gap: 10, alignItems: 'center', marginTop: 4 }}>
          <View style={{ flex: 1 }}>
            <RBCheckBox checked={both} onChange={setBoth}>{t('반대쪽 LED에도 같은 값 적용')}</RBCheckBox>
          </View>
          <SBtn kind="ghost" icon="download" label={t('다시 읽기')} onPress={reload} disabled={busy} />
          <SBtn kind="primary" icon="save" label={busy ? t('확인 중…') : t('적용')} onPress={apply} disabled={busy || oldIf || (!free && !choice)} />
        </View>
        <Text style={{ color: c.dim, fontSize: 10, marginTop: 12 }}>
          {t('전력 제한(순간 20 W, 지속 10.5 W)은 IF 펌웨어가 적용합니다. 그래서 실제 출력이 보낸 값보다 어두울 수 있습니다.')}
        </Text>
      </View>
    </Modal>
  );
}

const styles = StyleSheet.create({
  action: { flexDirection: 'row', alignItems: 'center', height: 34, paddingHorizontal: 14, borderWidth: 1 },
});
