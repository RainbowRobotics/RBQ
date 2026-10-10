import { useEffect, useState } from 'react';
import { View, Text, TextInput, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Tappable } from '@/components/anim';
import { ConfirmModal } from '@/components/ui/ConfirmModal';
import { useRobot } from '@/store/robot';
import { useFeatures } from '@/store/capability';
import { payloadSlotFilters } from '@/modules/registry';
import { actions, rest } from '@/lib/rest';
import {
  toRows, toggleRow, nudgeRow, patchRow, combine, limitOf, applyServerLimits, exceedsTotal,
  PAYLOAD_LIMITS, type Axis, type PayloadRow,
} from '@/lib/payload';
import { H2, Desc, SBtn, S } from './common';
import { inputVFix } from '@/components/ui/controls';
import { PayloadPreview3D } from '@/components/PayloadPreview3D';
import { t } from '@/lib/i18n';

const FIELDS: { k: 'mass' | Axis; label: string; color: string }[] = [
  { k: 'mass', label: 'Mass:', color: '#ef6c00' },
  { k: 'x', label: 'X:', color: '#e8443b' },
  { k: 'y', label: 'Y:', color: '#4caf50' },
  { k: 'z', label: 'Z:', color: '#4a9fe0' },
];
const STEPS = [0.001, 0.01, 0.1, 1];
const TWO_COL_MIN = 900;
const HELP_LINES = [
  '로봇에 장착한 payload 의 질량과 무게중심(CoM)을 등록합니다. 등록값은 보행 동역학(몸통 기준 위치·힘 배분)에 반영됩니다.',
  '정확한 사양을 모르더라도 대략적인 질량과 무게중심 높이(Z)는 입력하기를 권장합니다. 근사값만으로도 보행 안정성이 크게 좋아지며, 비워 두는 것이 가장 불리합니다.',
  '질량·무게중심을 바꾼 뒤에는 무게 중심 보정(ZMP)을 다시 수행하기를 권장합니다. 보정값은 현재 payload 상태 기준으로 저장되므로 적재물이 바뀌면 이전 보정이 맞지 않습니다.',
];

export function PayloadPanel() {
  const { c, fonts, radius } = useTheme();
  const { width } = useWindowDimensions();
  const twoCol = width >= TWO_COL_MIN;
  const ip = useRobot((s) => s.ip);
  const conn = useRobot((s) => s.conn);
  const keep = useFeatures((s) => payloadSlotFilters.find((f) => s.features?.[f.feature])?.keep);
  const [rows, setRows] = useState<PayloadRow[]>([]);
  const [sel, setSel] = useState(-1);
  const [step, setStep] = useState(0.01);
  const [msg, setMsg] = useState('');
  const [confirm, setConfirm] = useState(false);
  const [legacyOnly, setLegacyOnly] = useState(false);
  const [help, setHelp] = useState(false);
  const [edit, setEdit] = useState<{ k: 'mass' | Axis; text: string } | null>(null);

  const load = () =>
    rest.payload(ip).then((r) => {
      applyServerLimits(r.limits);
      if (!r.slots) { setLegacyOnly(true); setRows([]); setMsg(t('구버전 로봇 — 13슬롯 미지원')); return; }
      setLegacyOnly(false);
      setEdit(null);
      setRows(toRows(r.slots, keep));
      setMsg('');
    }).catch(() => { setRows([]); setMsg(t('읽기 실패 — 로봇 연결 확인')); });
  useEffect(() => { if (conn === 'connected') load(); }, [conn, ip, keep]); // eslint-disable-line react-hooks/exhaustive-deps

  const toggle = (id: number) => { setRows((rs) => toggleRow(rs, id)); setSel(id); };
  const nudge = (id: number, k: Axis | 'mass', d: number) => {
    setEdit(null);
    setRows((rs) => nudgeRow(rs, id, k, d));
  };
  const inRange = (k: 'mass' | Axis, n: number) =>
    k === 'mass' ? n >= PAYLOAD_LIMITS.massMin && n <= PAYLOAD_LIMITS.massMax
                 : Math.abs(n) <= limitOf(k);

  const apply = async () => {
    try {
      await actions.setPayloadSlots(ip, rows);
      await load();
      setMsg(t('적용됨 (로봇 값으로 재확인)'));
    } catch { setMsg(t('적용 실패 — 허용 범위 초과이거나 연결 끊김')); }
  };

  const total = combine(rows);
  const totalOver = exceedsTotal(total.mass);
  const selRow = rows.find((r) => r.id === sel) ?? rows[0] ?? null;
  const selId = selRow?.id ?? -1;
  useEffect(() => { setEdit(null); }, [selId]);
  const num = (v: number) => v.toFixed(3).replace(/\.?0+$/, '') || '0';
  const mounted = rows.filter((r) => r.enabled).length;

  const view3d = (
    <PayloadPreview3D rows={rows} total={total.mass > 1e-3 ? total : null} selectedId={selId}
      onSelect={setSel}
      onMove={(id, k, v) => setRows((rs) => patchRow(rs, id, { [k]: v }))}
      height={twoCol ? 300 : 260} />
  );

  const slotList = (
    <>
      <Text style={{ color: c.muted, fontSize: 11, marginBottom: 4 }}>
        {t('슬롯')} ({mounted}/{rows.length || 13})
      </Text>
      {rows.map((r) => {
        const picked = r.id === selId;
        return (
          <Tappable key={r.id} onPress={() => setSel(r.id)}
            style={{
              flexDirection: 'row', alignItems: 'center', gap: 8, height: 30, paddingHorizontal: 8,
              borderWidth: 1, borderColor: picked ? c.accent : c.line, borderRadius: radius.sm,
              marginBottom: 4, backgroundColor: picked ? c.elev : 'transparent',
            }}>
            <Tappable onPress={() => toggle(r.id)} hitSlop={8}
              style={{
                width: 17, height: 17, borderRadius: 4, borderWidth: 1,
                borderColor: r.enabled ? c.accent : c.line,
                backgroundColor: r.enabled ? c.accent : 'transparent',
                alignItems: 'center', justifyContent: 'center',
              }}>
              {r.enabled ? <Text style={{ color: c.onAccent, fontSize: 10, fontWeight: '700' }}>✓</Text> : null}
            </Tappable>
            <Text numberOfLines={1} style={{ flex: 1, color: r.enabled ? c.text : c.dim, fontSize: 11 }}>
              {r.isCustom ? '◻ ' : '● '}{r.label}
            </Text>
            <Text style={{ color: r.enabled ? c.text : c.dim, fontSize: 10, fontFamily: fonts.mono }}>
              {num(r.mass)} kg · ({num(r.x)}, {num(r.y)}, {num(r.z)})
            </Text>
          </Tappable>
        );
      })}
    </>
  );

  const editor = selRow ? (
    <View style={{ borderWidth: 1, borderColor: c.accent, borderRadius: radius.sm, padding: 10, marginTop: 10 }}>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, marginBottom: 6 }}>
        <Text numberOfLines={1} style={{ flex: 1, color: c.text, fontSize: 12, fontWeight: '700' }}>
          {selRow.label} <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono }}>#{selRow.id} {selRow.name}</Text>
        </Text>
        <View style={{ flexDirection: 'row', alignItems: 'center', justifyContent: 'flex-end',
                       flexWrap: 'wrap', flexShrink: 1, gap: 6 }}>
          <Text style={{ color: c.muted, fontSize: 10 }}>{t('간격')}</Text>
          {STEPS.map((sv) => (
            <Tappable key={sv} onPress={() => setStep(sv)}
              style={{ paddingHorizontal: 7, height: 22, borderWidth: 1, borderRadius: radius.sm, alignItems: 'center', justifyContent: 'center',
                       borderColor: step === sv ? c.accent : c.line, backgroundColor: step === sv ? c.accent : 'transparent' }}>
              <Text style={{ color: step === sv ? c.onAccent : c.muted, fontSize: 10 }}>{sv}</Text>
            </Tappable>
          ))}
        </View>
      </View>
      {FIELDS.map(({ k, label, color }) => {
        const shown = edit?.k === k
          ? edit.text
          : (k === 'mass' ? num(selRow.mass) : selRow[k].toFixed(3));
        return (
          <View key={k} style={{ flexDirection: 'row', alignItems: 'center', gap: 8, marginBottom: 6 }}>
            <Text style={{ color, fontSize: 10, fontWeight: '700', width: 34 }}>{label}</Text>
            <Tappable onPress={() => nudge(selRow.id, k, -step)}
              style={{ width: 32, height: 28, borderWidth: 1, borderColor: c.line, borderRadius: radius.sm, alignItems: 'center', justifyContent: 'center' }}>
              <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>−</Text>
            </Tappable>
            <TextInput value={shown}
              onChangeText={(v) => {
                const n = parseFloat(v);
                if (Number.isFinite(n) && !inRange(k, n)) return;
                setEdit({ k, text: v });
                if (Number.isFinite(n)) setRows((rs) => patchRow(rs, selRow.id, { [k]: n }));
              }}
              onBlur={() => setEdit(null)}
              keyboardType="numbers-and-punctuation" autoCapitalize="none"
              style={[{ flex: 1, height: 28, borderWidth: 1, borderColor: c.line, borderRadius: radius.sm,
                        textAlign: 'center', backgroundColor: c.bg, color: c.text, fontSize: 11, fontFamily: fonts.mono }, inputVFix]} />
            <Tappable onPress={() => nudge(selRow.id, k, step)}
              style={{ width: 32, height: 28, borderWidth: 1, borderColor: c.line, borderRadius: radius.sm, alignItems: 'center', justifyContent: 'center' }}>
              <Text style={{ color: c.text, fontSize: 14, fontWeight: '700' }}>+</Text>
            </Tappable>
          </View>
        );
      })}
      <Text style={{ color: c.dim, fontSize: 9, textAlign: 'right' }}>
        {t('질량(Mass)')} {PAYLOAD_LIMITS.massMin}~{PAYLOAD_LIMITS.massMax} kg
        {' · '}{t('합계')} ≤{PAYLOAD_LIMITS.massTotalMax} kg
        {' · '}{t('무게중심(CoM)')} |X|≤{PAYLOAD_LIMITS.xMax} |Y|≤{PAYLOAD_LIMITS.yMax} |Z|≤{PAYLOAD_LIMITS.zMax} m
      </Text>
    </View>
  ) : (
    <Text style={{ color: c.dim, fontSize: 11, marginTop: 10 }}>
      {t('(로봇에서 슬롯을 읽어오면 여기서 편집합니다)')}
    </Text>
  );

  const actionsRow = (
    <>
      <View style={[S.actions, { justifyContent: 'flex-end', marginTop: 10 }]}>
        <SBtn kind="ghost" icon="download" label="Get" onPress={load} />
        <SBtn kind="primary" icon="save" label="Set Payload" disabled={!rows.length || totalOver}
          onPress={() => setConfirm(true)} />
      </View>
      {totalOver ? (
        <Text style={{ color: c.redbright, fontSize: 11, textAlign: 'right', marginTop: 6 }}>
          {t('합계가 상한 {n} kg 을 넘습니다').replace('{n}', String(PAYLOAD_LIMITS.massTotalMax))}
        </Text>
      ) : msg ? <Text style={{ color: c.muted, fontSize: 11, textAlign: 'right', marginTop: 6 }}>{msg}</Text> : null}
    </>
  );

  const totalBox = (
    <View style={{ borderWidth: 1, borderColor: c.line2, borderRadius: radius.sm, padding: 10, marginTop: 10, backgroundColor: c.bg }}>
      <Text style={{ color: totalOver ? c.redbright : c.text, fontSize: 11, fontFamily: fonts.mono }}>
        <Text style={{ color: '#a855c7' }}>● </Text>
        {t('합계')} {num(total.mass)} kg
        {totalOver ? ` > ${PAYLOAD_LIMITS.massTotalMax} kg` : ''}
        {'   '}{t('무게중심')} ({num(total.x)}, {num(total.y)}, {num(total.z)}) m
      </Text>
    </View>
  );

  return (
    <>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 10 }}>
        <H2>📦 {t('로봇 페이로드 설정')}</H2>
        <Tappable onPress={() => setHelp((v) => !v)} hitSlop={6}
          style={{ height: 22, paddingHorizontal: 9, borderRadius: 11, borderWidth: 1, marginBottom: 3,
                   alignItems: 'center', justifyContent: 'center',
                   borderColor: help ? c.accent : c.line, backgroundColor: c.elev }}>
          <Text style={{ color: help ? c.accent : c.muted, fontSize: 10.5 }}>? {t('도움말')}</Text>
        </Tappable>
      </View>
      <Desc>{t('적재물 13슬롯의 질량·무게중심 → 보행 동역학에 반영. 체크한 슬롯만 로봇에 적용됩니다.')}</Desc>
      {help ? (
        <View style={[S.note, { backgroundColor: c.elev, borderColor: c.line, marginTop: -6 }]}>
          <Text style={{ color: c.muted, fontSize: 10, fontWeight: '600', letterSpacing: 0.4, marginBottom: 6 }}>{t('PAYLOAD 설정 안내')}</Text>
          {HELP_LINES.map((s, i) => (
            <Text key={i} style={{ color: c.text, fontSize: 12, lineHeight: 18, marginBottom: i < HELP_LINES.length - 1 ? 6 : 0 }}>• {t(s)}</Text>
          ))}
        </View>
      ) : null}

      {legacyOnly ? (
        <Text style={{ color: c.dim, fontSize: 11, marginBottom: 10 }}>
          {t('이 로봇은 payload 슬롯 API가 없습니다. 로봇 소프트웨어를 갱신하세요.')}
        </Text>
      ) : null}

      {twoCol ? (
        <View style={{ flexDirection: 'row', gap: 16, alignItems: 'flex-start' }}>
          <View style={{ flex: 50 }}>{view3d}{editor}</View>
          <View style={{ flex: 50 }}>{slotList}{totalBox}{actionsRow}</View>
        </View>
      ) : (
        <>
          {view3d}
          <View style={{ height: 12 }} />
          {slotList}{editor}{totalBox}{actionsRow}
        </>
      )}

      {confirm && (
        <ConfirmModal title="Set Payload"
          message={`${t('장착')} ${mounted}${t('개')} · ${num(total.mass)} kg · ${t('무게중심')} (${num(total.x)}, ${num(total.y)}, ${num(total.z)}) m\n${t('체크 해제한 슬롯은 mass 0 으로 비워집니다. 저장 즉시 보행 동역학에 반영되며, 1~2초에 걸쳐 천천히 적용됩니다.')}\n\n${t('질량·무게중심을 바꾼 뒤에는 무게 중심 보정(ZMP)을 다시 수행하기를 권장합니다.')}`}
          confirmLabel={t('적용')} onConfirm={() => { setConfirm(false); apply(); }} onClose={() => setConfirm(false)} />
      )}
    </>
  );
}
