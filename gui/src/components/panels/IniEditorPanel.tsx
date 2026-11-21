import { useCallback, useEffect, useMemo, useState } from 'react';
import { View, Text, StyleSheet, ScrollView, Pressable, TextInput, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { inputVFix } from '@/components/ui/controls';
import { Modal } from '@/components/ui/overlays';
import { RBToggle } from '@/rb/components/RBToggle';
import { RBAccordion } from '@/rb/components/RBAccordion';
import { useRobot } from '@/store/robot';
import { iniSources } from '@/modules/registry';
import { t } from '@/lib/i18n';
import { INI_SOURCES, isBoolValue, toggleBoolValue, type IniFile, type IniKV, type IniSource } from '@/lib/iniSetting';
import { notify } from '@/components/DoneToast';

const SEP = '\u0000';
const ekey = (src: string, sec: string, key: string) => `${src}${SEP}${sec}${SEP}${key}`;
const profileOf = (k: IniKV) => (k.source === 'profile' ? k.value : k.profile);
type Loaded = { file?: IniFile; err?: string };

function splitCols<T extends { keys: unknown[] }>(secs: T[], n: number): T[][] {
  const cols: T[][] = Array.from({ length: n }, () => []);
  const h = new Array(n).fill(0);
  for (const s of secs) {
    const i = h.indexOf(Math.min(...h));
    cols[i].push(s);
    h[i] += s.keys.length + 2;
  }
  return cols;
}

export function IniEditorPanel({ initialTarget }: { initialTarget?: string } = {}) {
  const { c, fonts, radius } = useTheme();
  const { width } = useWindowDimensions();
  const ip = useRobot((s) => s.ip);
  const extra = iniSources.useExtra(ip);
  const sources = useMemo(() => [...INI_SOURCES, ...extra], [extra]);
  const [on, setOn] = useState<string[]>([initialTarget ?? 'motion']);
  const [files, setFiles] = useState<Record<string, Loaded>>({});
  const [edits, setEdits] = useState<Record<string, string>>({});
  const [q, setQ] = useState('');
  const [changedOnly, setChangedOnly] = useState(false);
  const [open, setOpen] = useState<Record<string, boolean>>({});
  const [confirm, setConfirm] = useState(false);
  const [saving, setSaving] = useState(false);
  const [saveErr, setSaveErr] = useState<string | null>(null);

  const dropEdits = (key: string) =>
    setEdits((e) => Object.fromEntries(Object.entries(e).filter(([k]) => !k.startsWith(key + SEP))));
  const load = useCallback((src: IniSource) => {
    setFiles((f) => ({ ...f, [src.key]: {} }));
    dropEdits(src.key);
    src.get(ip)
      .then((file) => setFiles((f) => ({ ...f, [src.key]: { file } })))
      .catch((e) => setFiles((f) => ({ ...f, [src.key]: { err: String(e?.message ?? e) } })));
  }, [ip]);
  useEffect(() => { setFiles({}); setEdits({}); }, [ip]);
  const shown = sources.filter((s) => on.includes(s.key));
  useEffect(() => { for (const s of shown) if (!files[s.key]) load(s); });

  const toggleFile = (key: string) =>
    setOn((o) => (o.includes(key) ? (o.length > 1 ? o.filter((k) => k !== key) : o) : [...o, key]));
  const current = (src: string, sec: string, k: IniKV) => edits[ekey(src, sec, k.key)] ?? k.value;
  const setVal = (src: string, sec: string, k: IniKV, v: string) =>
    setEdits((e) => {
      const id = ekey(src, sec, k.key);
      if (v === k.value) { const { [id]: _drop, ...rest } = e; return rest; }
      return { ...e, [id]: v };
    });
  const dirtyIn = (key: string) => Object.keys(edits).filter((k) => k.startsWith(key + SEP)).length;

  const needle = q.trim().toLowerCase();
  const filtering = !!needle || changedOnly;
  useEffect(() => { setOpen({}); }, [needle, changedOnly]);
  const view = shown.map((src) => {
    const f = files[src.key] ?? {};
    const secs = (f.file?.sections ?? []).map((sec) => {
      const secHit = !!needle && sec.name.toLowerCase().includes(needle);
      const keys = sec.keys.filter((k) => {
        if (changedOnly && edits[ekey(src.key, sec.name, k.key)] === undefined) return false;
        return !needle || secHit || k.key.toLowerCase().includes(needle) || current(src.key, sec.name, k).toLowerCase().includes(needle);
      });
      return { sec, keys, id: src.key + SEP + sec.name };
    }).filter((x) => x.keys.length > 0 || !filtering);
    return { src, f, secs };
  });
  const ids = view.flatMap((v) => v.secs.map((s) => s.id));
  const isOpen = (id: string) => open[id] ?? filtering;
  const allOpen = ids.length > 0 && ids.every(isOpen);
  const hasProfileKeys = view.some((v) => v.f.file?.sections.some((s) => s.keys.some((k) => k.source)));
  const cols = width >= 1100 ? 3 : width >= 720 ? 2 : 1;

  const dirty = Object.keys(edits).length;
  const dirtySources = sources.filter((s) => dirtyIn(s.key) > 0);
  const diffs = useMemo(() => Object.entries(edits).map(([k, next]) => {
    const [src, sec, key] = k.split(SEP);
    const kv = files[src]?.file?.sections.find((s) => s.name === sec)?.keys.find((x) => x.key === key);
    return { id: k, src, sec, key, old: kv?.value ?? '', next, toProfile: !!kv && profileOf(kv) === next };
  }), [edits, files]);
  const needApply = dirtySources.filter((s) => diffs.some((d) => d.src === s.key && !s.liveSections?.includes(d.sec)));
  const notes = [...new Set(needApply.map((s) => s.cannotApply?.()).filter((m): m is string => !!m))];
  const applies = [...new Set(needApply.filter((s) => !s.cannotApply?.()).map((s) => s.apply))]
    .sort((a, b) => Number(!!a.service) - Number(!!b.service));
  const gait = useRobot((s) => s.gait);
  const isMine = useRobot((s) => s.isMine);
  const resting = gait === 'SITTING' || gait === 'CONTROL_OFF';
  const blocked = !applies.length ? null
    : !resting ? t('로봇을 앉히거나 제어를 끈 뒤에 저장할 수 있습니다 (지금 {gait})').replace('{gait}', gait)
    : applies.some((a) => a.service) && !isMine ? t('로봇 서비스 재시작은 제어권을 가진 기기에서만 할 수 있습니다')
    : null;

  const save = async () => {
    if (blocked) return;
    setSaving(true); setSaveErr(null);
    try {
      for (const src of dirtySources) {
        const file = files[src.key]?.file;
        if (!file) continue;
        const sections = file.sections.map((s) => ({
          name: s.name,
          keys: s.keys.map((k) => ({ key: k.key, value: current(src.key, s.name, k) })),
        }));
        const next = await src.put(ip, sections);
        setFiles((f) => ({ ...f, [src.key]: { file: next } }));
        dropEdits(src.key);
      }
    } catch (e: any) {
      setSaveErr(String(e?.message ?? e));
      setSaving(false);
      return;
    }
    try {
      for (const a of applies) await a.run(ip);
      notify(applies.some((a) => a.service) ? t('로봇을 재시작하는 중입니다 — 잠시 뒤 다시 연결해 주세요')
        : applies.length ? t('저장하고 다시 시작했습니다') : t('설정 파일을 저장했습니다'));
      for (const m of notes) notify(m, 'warn');
    } catch (e: any) {
      notify(`${t('저장했지만 다시 시작하지 못했습니다')}: ${e?.message ?? e}`, 'warn');
    } finally {
      setConfirm(false);
      setSaving(false);
    }
  };

  const keyRow = (src: string, sec: string, k: IniKV) => {
    const v = current(src, sec, k);
    const isDirty = edits[ekey(src, sec, k.key)] !== undefined;
    const fromProfile = k.source === 'profile';
    const overrides = k.source === 'ini';
    const prof = profileOf(k);
    const keyColor = isDirty ? c.accent2 : fromProfile ? c.dim : overrides ? c.amberTx : c.muted;
    return (
      <View key={k.key} style={styles.iniRow}>
        <Text numberOfLines={1} style={{ flex: 1, minWidth: 0, color: keyColor, fontSize: 11, fontFamily: fonts.mono }}>
          {k.key}
        </Text>
        {prof !== undefined && v !== prof ? (
          <Pressable onPress={() => setVal(src, sec, k, prof)} hitSlop={6}
            accessibilityLabel={t('기본값으로')} style={{ padding: 2 }}>
            <Icon name="recover" size={12} color={c.muted} />
          </Pressable>
        ) : null}
        {isBoolValue(v) ? (
          <Pressable onPress={() => setVal(src, sec, k, toggleBoolValue(v))}
            style={[styles.iniBool, {
              borderRadius: radius.sm,
              backgroundColor: /^t/i.test(v) ? 'rgba(63,185,80,0.14)' : c.elev,
              borderColor: /^t/i.test(v) ? 'rgba(63,185,80,0.55)' : c.line,
            }]}>
            <Text style={{ color: /^t/i.test(v) ? c.greenTx : c.dim, fontSize: 11, fontFamily: fonts.mono, fontWeight: '700' }}>{v}</Text>
          </Pressable>
        ) : (
          <TextInput value={v} onChangeText={(nv) => setVal(src, sec, k, nv)}
            autoCapitalize="none" autoCorrect={false}
            style={[styles.iniInput, inputVFix, {
              borderRadius: radius.sm, color: fromProfile && !isDirty ? c.dim : c.text, fontFamily: fonts.mono,
              backgroundColor: isDirty ? 'rgba(77,156,245,0.10)' : c.elev,
              borderColor: isDirty ? 'rgba(77,156,245,0.55)' : c.line,
            }]} />
        )}
      </View>
    );
  };

  return (
    <View style={{ flex: 1 }}>
      <ScrollView style={{ flex: 1 }} showsVerticalScrollIndicator={false} stickyHeaderIndices={[1]}>
        <View>
          <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 3 }}>
            📄 {t('설정 파일')} <Text style={{ color: c.dim, fontSize: 10 }}>INI · {t('위험 조작')}</Text>
          </Text>
          <Text style={{ color: c.dim, fontSize: 11, marginBottom: 10 }}>
            {t('로봇 데몬의 INI 설정을 직접 편집합니다. 값의 의미를 모르면 바꾸지 마세요.')}
          </Text>
          <View style={{ flexDirection: 'row', flexWrap: 'wrap', gap: 6, marginBottom: 8 }}>
            {sources.map((s) => {
              const n = dirtyIn(s.key);
              return (
                <RBToggle key={s.key} size="sm" variant="outline" pressed={on.includes(s.key)} onPressedChange={() => toggleFile(s.key)}>
                  {s.label}{n ? ` ●${n}` : ''}
                </RBToggle>
              );
            })}
          </View>
        </View>
        <View style={{ flexDirection: 'row', alignItems: 'center', gap: 8, paddingBottom: 8, flexWrap: 'wrap', backgroundColor: c.glassHi }}>
          <View style={[styles.iniSearch, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
            <Icon name="search" size={12} color={c.dim} />
            <TextInput value={q} onChangeText={setQ} placeholder={t('키·값·섹션 검색')} placeholderTextColor={c.dim}
              autoCapitalize="none" autoCorrect={false}
              style={[{ flex: 1, color: c.text, fontSize: 12, padding: 0 }, inputVFix]} />
            {q ? (
              <Pressable onPress={() => setQ('')} hitSlop={8} accessibilityLabel={t('검색어 지우기')}>
                <Icon name="x" size={12} color={c.dim} />
              </Pressable>
            ) : null}
          </View>
          <RBToggle size="sm" variant="outline" pressed={changedOnly} onPressedChange={setChangedOnly}>{t('바꾼 것만')}</RBToggle>
          <Tappable onPress={() => setOpen(Object.fromEntries(ids.map((id) => [id, !allOpen])))}
            style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 30, opacity: ids.length ? 1 : 0.4 }]}>
            <Text style={{ color: c.text, fontSize: 11 }}>{allOpen ? t('모두 접기') : t('모두 펼치기')}</Text>
          </Tappable>
        </View>
        <View style={{ gap: 14 }}>
          {hasProfileKeys ? (
            <Text style={{ color: c.dim, fontSize: 10 }}>
              {t('흐린 키는 로봇에 내장된 플랫폼 프로필 값이고, 강조된 키는 이 로봇의 덮어쓰기입니다. 프로필과 같은 값으로 저장하면 키가 파일에서 지워집니다.')}
            </Text>
          ) : null}
          {view.map(({ src, f, secs }) => {
            const readAt = f.file?.timestamp ? f.file.timestamp.replace('T', ' ').replace('Z', '') : '';
            return (
              <View key={src.key} style={{ gap: 4 }}>
                <View style={{ flexDirection: 'row', alignItems: 'baseline', gap: 7, flexWrap: 'wrap', paddingBottom: 2 }}>
                  <Text style={{ color: c.text, fontSize: 13, fontWeight: '700' }}>{src.label}</Text>
                  <Text style={{ color: c.dim, fontSize: 10, fontFamily: fonts.mono }}>{f.file?.path ?? ''}</Text>
                  {f.file ? <Text style={{ color: c.dim, fontSize: 10 }}>· {f.file.sections.length}{t('개 섹션 · ')}{readAt}{t(' 읽음')}</Text> : null}
                </View>
                {f.err ? <Text style={{ color: c.redTx, fontSize: 11 }}>{t('오류:')} {f.err}</Text>
                  : !f.file ? <Text style={{ color: c.dim, fontSize: 11 }}>{t('불러오는 중…')}</Text>
                  : secs.length === 0 ? <Text style={{ color: c.dim, fontSize: 11 }}>{t('맞는 키가 없습니다')}</Text>
                  : (
                    <View style={{ flexDirection: 'row', gap: 9, alignItems: 'flex-start' }}>
                      {splitCols(secs.map((x) => ({ ...x, keys: x.sec.keys, shown: x.keys })), cols).map((col, ci) => (
                        <View key={ci} style={{ flex: 1, minWidth: 0, gap: 9 }}>
                          {col.map(({ sec, shown: keys, id }) => {
                            const n = keys.filter((k) => edits[ekey(src.key, sec.name, k.key)] !== undefined).length;
                            return (
                              <View key={id} style={[styles.iniSec, { backgroundColor: c.bg, borderColor: n ? 'rgba(77,156,245,0.55)' : c.line, borderRadius: radius.md }]}>
                                <RBAccordion open={isOpen(id)} onOpenChange={(v) => setOpen((o) => ({ ...o, [id]: v }))}
                                  title={<Text style={{ fontSize: 12 }}>[{sec.name}] <Text style={{ color: c.dim, fontWeight: '400' }}>{sec.keys.length}</Text>{n ? <Text style={{ color: c.accent2 }}>{`  ● ${n}`}</Text> : null}</Text>}>
                                  <View style={{ paddingHorizontal: 8, paddingBottom: 6 }}>
                                    {keys.map((k) => keyRow(src.key, sec.name, k))}
                                  </View>
                                </RBAccordion>
                              </View>
                            );
                          })}
                        </View>
                      ))}
                    </View>
                  )}
              </View>
            );
          })}
        </View>
      </ScrollView>
      <View style={[styles.iniBar, { borderTopColor: c.line }]}>
        {dirty > 0
          ? <Text style={{ color: c.accent2, fontSize: 10, fontWeight: '600' }}>● {t('변경 {n}개').replace('{n}', String(dirty))}</Text>
          : <Text style={{ color: c.dim, fontSize: 10 }}>{t('변경 없음')}</Text>}
        <Text numberOfLines={1} style={{ flex: 1, color: c.dim, fontSize: 9 }}>
          {t('저장 시 백업 없음 · 저장하면 그 파일을 읽는 프로그램을 다시 시작합니다(앉음·제어 OFF 일 때만)')}
        </Text>
        <Tappable onPress={() => setEdits({})} style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, opacity: dirty ? 1 : 0.4, height: 30 }]}>
          <Icon name="recover" size={12} color={c.muted} />
          <Text style={{ color: c.text, fontSize: 11 }}>{t('되돌리기')}</Text>
        </Tappable>
        <Tappable onPress={() => shown.forEach(load)} style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 30 }]}>
          <Icon name="download" size={12} color={c.muted} />
          <Text style={{ color: c.text, fontSize: 11 }}>{t('다시 읽기')}</Text>
        </Tappable>
        <Tappable onPress={() => { if (dirty) { setSaveErr(null); setConfirm(true); } }} style={[styles.action, {
          backgroundColor: c.accent, borderColor: c.accent, borderRadius: radius.md, opacity: dirty ? 1 : 0.4, height: 30,
        }]}>
          <Icon name="save" size={12} color={c.onAccent} />
          <Text style={{ color: c.onAccent, fontSize: 11, fontWeight: '600' }}>{t('저장')}</Text>
        </Tappable>
      </View>
      {confirm && (
        <Modal onClose={() => !saving && setConfirm(false)}>
          <View style={[styles.iniModal, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
            <Text style={{ color: c.text, fontSize: 15, fontWeight: '700', textAlign: 'center', marginBottom: 10 }}>
              {dirtySources.length === 1
                ? t('{label}에 저장하시겠습니까?').replace('{label}', dirtySources[0].label)
                : t('설정 파일 {n}개에 저장하시겠습니까?').replace('{n}', String(dirtySources.length))}
            </Text>
            <ScrollView style={{ maxHeight: 200 }}>
              {dirtySources.map((s) => (
                <View key={s.key} style={{ marginBottom: 6 }}>
                  <Text style={{ color: c.dim, fontSize: 9.5, fontWeight: '700', letterSpacing: 0.5, marginBottom: 4 }}>
                    {s.label} · {t('변경 {n}개').replace('{n}', String(dirtyIn(s.key)))}
                  </Text>
                  {diffs.filter((d) => d.src === s.key).map((d) => (
                    <View key={d.id} style={[styles.iniDiff, { borderTopColor: c.line2 }]}>
                      <Text numberOfLines={1} style={{ flex: 1, color: c.dim, fontSize: 10, fontFamily: fonts.mono }}>[{d.sec}] {d.key}</Text>
                      <Text style={{ color: c.redTx, fontSize: 10, fontFamily: fonts.mono, textDecorationLine: 'line-through' }}>{d.old}</Text>
                      <Text style={{ color: c.dim, fontSize: 10 }}>→</Text>
                      <Text style={{ color: c.greenTx, fontSize: 10, fontFamily: fonts.mono, fontWeight: '700' }}>{d.next}</Text>
                      {d.toProfile ? <Text style={{ color: c.dim, fontSize: 9 }}>{t('프로필 값 · 키 삭제')}</Text> : null}
                    </View>
                  ))}
                </View>
              ))}
            </ScrollView>
            <Text style={{ color: c.muted, fontSize: 10.5, marginTop: 10, lineHeight: 15 }}>
              {applies.length
                ? <>{t('저장한 뒤 다시 시작합니다:')} <Text style={{ color: c.amberTx, fontWeight: '700' }}>{applies.map((a) => t(a.label)).join(', ')}</Text>
                  {applies.some((a) => a.service) ? ` — ${t('재시작 동안 로봇 연결이 끊깁니다')}` : ''}</>
                : notes.length ? null : t('바뀐 섹션은 프로그램이 바로 다시 읽습니다 — 재시작하지 않습니다.')}
            </Text>
            {notes.map((m) => <Text key={m} style={{ color: c.amberTx, fontSize: 10.5, marginTop: 6, lineHeight: 15 }}>{m}</Text>)}
            {blocked ? <Text style={{ color: c.amberTx, fontSize: 11, marginTop: 8, fontWeight: '600' }}>{blocked}</Text> : null}
            {saveErr ? <Text style={{ color: c.redTx, fontSize: 11, marginTop: 8 }}>{t('오류:')} {saveErr}</Text> : null}
            <View style={{ flexDirection: 'row', gap: 9, marginTop: 14 }}>
              <Tappable onPress={() => !saving && setConfirm(false)}
                style={[styles.iniModalBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{t('취소')}</Text>
              </Tappable>
              <Tappable onPress={() => { if (!saving) void save(); }} disabled={!!blocked}
                style={[styles.iniModalBtn, { backgroundColor: 'rgba(231,51,28,0.85)', borderColor: c.dangerLine, borderRadius: radius.md, opacity: blocked ? 0.4 : 1 }]}>
                <Icon name="save" size={13} color="#fff" />
                <Text style={{ color: '#fff', fontSize: 12, fontWeight: '600' }}>{saving ? t('저장 중…') : applies.length ? t('저장하고 다시 시작') : t('저장')}</Text>
              </Tappable>
            </View>
          </View>
        </Modal>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  action: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 34, paddingHorizontal: 14, borderWidth: 1 },
  iniSearch: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 30, paddingHorizontal: 9, borderWidth: 1, flexGrow: 1, minWidth: 200, maxWidth: 420 },
  iniSec: { borderWidth: 1, overflow: 'hidden' },
  iniRow: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 3 },
  iniBool: { alignItems: 'center', justifyContent: 'center', height: 26, minWidth: 60, paddingHorizontal: 8, borderWidth: 1 },
  iniInput: { height: 26, minWidth: 60, maxWidth: 190, paddingHorizontal: 8, paddingVertical: 0, borderWidth: 1, fontSize: 11, textAlign: 'right' },
  iniBar: { flexDirection: 'row', alignItems: 'center', gap: 9, paddingTop: 10, marginTop: 8, borderTopWidth: 1 },
  iniModal: { width: 460, maxWidth: '100%', borderWidth: 1, padding: 18 },
  iniDiff: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 5, borderTopWidth: 1 },
  iniModalBtn: { flex: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 38, borderWidth: 1 },
});
