import { useCallback, useEffect, useMemo, useState } from 'react';
import { View, Text, StyleSheet, ScrollView, Pressable, TextInput } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Select, inputVFix } from '@/components/ui/controls';
import { Modal } from '@/components/ui/overlays';
import { useRobot } from '@/store/robot';
import { t } from '@/lib/i18n';
import {
  INI_TARGETS, getIniSetting, putIniSetting, isBoolValue, toggleBoolValue,
  type IniTarget, type IniFile, type IniSection,
} from '@/lib/iniSetting';

const ekey = (sec: string, key: string) => `${sec}\u0000${key}`;

function splitCols(secs: IniSection[], n = 3): IniSection[][] {
  const cols: IniSection[][] = Array.from({ length: n }, () => []);
  const h = new Array(n).fill(0);
  for (const s of secs) {
    const i = h.indexOf(Math.min(...h));
    cols[i].push(s);
    h[i] += s.keys.length + 2;
  }
  return cols;
}

export function IniEditorPanel({ initialTarget }: { initialTarget?: IniTarget } = {}) {
  const { c, fonts, radius } = useTheme();
  const ip = useRobot((s) => s.ip);
  const [target, setTarget] = useState<IniTarget>(initialTarget ?? 'motion');
  const [file, setFile] = useState<IniFile | null>(null);
  const [err, setErr] = useState<string | null>(null);
  const [loading, setLoading] = useState(false);
  const [edits, setEdits] = useState<Record<string, string>>({});
  const [q, setQ] = useState('');
  const [confirm, setConfirm] = useState(false);
  const [saving, setSaving] = useState(false);

  const load = useCallback((t: IniTarget) => {
    setLoading(true); setErr(null); setFile(null); setEdits({}); setConfirm(false);
    getIniSetting(ip, t)
      .then((f) => setFile(f))
      .catch((e) => setErr(String(e?.message ?? e)))
      .finally(() => setLoading(false));
  }, [ip]);
  useEffect(() => { load(target); }, [load, target]);

  const original = useCallback((sec: string, key: string) =>
    file?.sections.find((s) => s.name === sec)?.keys.find((k) => k.key === key)?.value ?? '', [file]);
  const current = (sec: string, key: string) => edits[ekey(sec, key)] ?? original(sec, key);
  const setVal = (sec: string, key: string, v: string) =>
    setEdits((e) => {
      const k = ekey(sec, key);
      if (v === original(sec, key)) { const { [k]: _drop, ...rest2 } = e; return rest2; }
      return { ...e, [k]: v };
    });

  const dirty = Object.keys(edits).length;
  const diffs = useMemo(() =>
    Object.entries(edits).map(([k, v]) => {
      const [sec, key] = k.split('\u0000');
      return { sec, key, old: original(sec, key), next: v };
    }), [edits, original]);

  const filtered = useMemo(() => {
    if (!file) return [];
    const s = q.trim().toLowerCase();
    if (!s) return file.sections;
    return file.sections
      .map((sec) => sec.name.toLowerCase().includes(s)
        ? sec
        : { ...sec, keys: sec.keys.filter((k) => k.key.toLowerCase().includes(s)) })
      .filter((sec) => sec.keys.length > 0);
  }, [file, q]);
  const cols = useMemo(() => splitCols(filtered), [filtered]);

  const tgt = INI_TARGETS.find((t) => t.key === target)!;
  const save = () => {
    if (!file) return;
    setSaving(true);
    const sections = file.sections.map((s) => ({
      name: s.name,
      keys: s.keys.map((k) => ({ key: k.key, value: current(s.name, k.key) })),
    }));
    putIniSetting(ip, target, sections)
      .then((f) => { setFile(f); setEdits({}); setConfirm(false); })
      .catch((e) => setErr(String(e?.message ?? e)))
      .finally(() => setSaving(false));
  };

  const readAt = file?.timestamp ? file.timestamp.replace('T', ' ').replace('Z', '') : '';
  return (
    <View style={{ flex: 1 }}>
      <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', marginBottom: 3 }}>
        📄 {t('설정 파일')} <Text style={{ color: c.dim, fontSize: 10 }}>INI · {t('위험 조작')}</Text>
      </Text>
      <Text style={{ color: c.dim, fontSize: 11, marginBottom: 10 }}>
        {t('로봇 데몬의 INI 설정을 직접 편집합니다. 값의 의미를 모르면 바꾸지 마세요.')}
      </Text>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 10, marginBottom: 8 }}>
        <View style={{ flex: 1, maxWidth: 520 }}>
          <Select options={INI_TARGETS.map((t) => ({ key: t.key, label: t.label }))}
            value={target} onChange={setTarget} />
        </View>
        <View style={[styles.iniSearch, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.sm }]}>
          <Icon name="search" size={12} color={c.dim} />
          <TextInput value={q} onChangeText={setQ} placeholder={t('키 검색')} placeholderTextColor={c.dim}
            autoCapitalize="none" autoCorrect={false}
            style={[{ flex: 1, color: c.text, fontSize: 11, padding: 0 }, inputVFix]} />
        </View>
      </View>
      <View style={{ flexDirection: 'row', alignItems: 'center', gap: 7, marginBottom: 8, flexWrap: 'wrap' }}>
        <Text style={{ color: c.dim, fontSize: 9.5, fontFamily: fonts.mono }}>{file?.path ?? '—'}</Text>
        {file ? <Text style={{ color: c.dim, fontSize: 9.5 }}>· {file.sections.length}{t('개 섹션 · ')}{readAt}{t(' 읽음')}</Text> : null}
      </View>
      {err ? (
        <Text style={{ color: c.redTx, fontSize: 11, marginBottom: 8 }}>{t('오류:')} {err}</Text>
      ) : null}
      <ScrollView style={{ flex: 1 }} showsVerticalScrollIndicator={false}>
        {loading ? (
          <Text style={{ color: c.dim, fontSize: 11 }}>{t('불러오는 중…')}</Text>
        ) : (
          <View style={{ flexDirection: 'row', gap: 9, alignItems: 'flex-start' }}>
            {cols.map((col, ci) => (
              <View key={ci} style={{ flex: 1, minWidth: 0, gap: 9 }}>
                {col.map((sec) => (
                  <View key={sec.name} style={[styles.iniSec, { backgroundColor: c.bg, borderColor: c.line, borderRadius: radius.md }]}>
                    <Text style={{ color: c.dim, fontSize: 9, fontWeight: '700', letterSpacing: 0.5, marginBottom: 3 }}>
                      [{sec.name}]
                    </Text>
                    {sec.keys.map((k) => {
                      const v = current(sec.name, k.key);
                      const isDirty = edits[ekey(sec.name, k.key)] !== undefined;
                      return (
                        <View key={k.key} style={styles.iniRow}>
                          <Text numberOfLines={1} style={{ flex: 1, minWidth: 0, color: isDirty ? c.accent2 : c.muted, fontSize: 9.5, fontFamily: fonts.mono }}>
                            {k.key}
                          </Text>
                          {isBoolValue(v) ? (
                            <Pressable onPress={() => setVal(sec.name, k.key, toggleBoolValue(v))}
                              style={[styles.iniBool, {
                                borderRadius: radius.sm,
                                backgroundColor: /^t/i.test(v) ? 'rgba(63,185,80,0.14)' : c.elev,
                                borderColor: /^t/i.test(v) ? 'rgba(63,185,80,0.55)' : c.line,
                              }]}>
                              <Text style={{ color: /^t/i.test(v) ? c.greenTx : c.dim, fontSize: 9.5, fontFamily: fonts.mono, fontWeight: '700' }}>{v}</Text>
                            </Pressable>
                          ) : (
                            <TextInput value={v} onChangeText={(nv) => setVal(sec.name, k.key, nv)}
                              autoCapitalize="none" autoCorrect={false}
                              style={[styles.iniInput, inputVFix, {
                                borderRadius: radius.sm, color: c.text, fontFamily: fonts.mono,
                                backgroundColor: isDirty ? 'rgba(77,156,245,0.10)' : c.elev,
                                borderColor: isDirty ? 'rgba(77,156,245,0.55)' : c.line,
                              }]} />
                          )}
                        </View>
                      );
                    })}
                  </View>
                ))}
              </View>
            ))}
          </View>
        )}
      </ScrollView>
      <View style={[styles.iniBar, { borderTopColor: c.line }]}>
        {dirty > 0
          ? <Text style={{ color: c.accent2, fontSize: 10, fontWeight: '600' }}>● {t('변경 {n}개').replace('{n}', String(dirty))}</Text>
          : <Text style={{ color: c.dim, fontSize: 10 }}>{t('변경 없음')}</Text>}
        <Text numberOfLines={1} style={{ flex: 1, color: c.dim, fontSize: 9 }}>
          {t('저장 시 백업 생성 · 반영엔 ')}{tgt.daemon}{t(' 데몬 재시작 필요')}
        </Text>
        <Tappable onPress={() => setEdits({})} style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, opacity: dirty ? 1 : 0.4, height: 30 }]}>
          <Icon name="recover" size={12} color={c.muted} />
          <Text style={{ color: c.text, fontSize: 11 }}>{t('되돌리기')}</Text>
        </Tappable>
        <Tappable onPress={() => load(target)} style={[styles.action, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md, height: 30 }]}>
          <Icon name="download" size={12} color={c.muted} />
          <Text style={{ color: c.text, fontSize: 11 }}>{t('다시 읽기')}</Text>
        </Tappable>
        <Tappable onPress={() => dirty && setConfirm(true)} style={[styles.action, {
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
              {t('{label}에 저장하시겠습니까?').replace('{label}', tgt.label)}
            </Text>
            <Text style={{ color: c.dim, fontSize: 9.5, fontWeight: '700', letterSpacing: 0.5, marginBottom: 4 }}>{t('변경 {n}개').replace('{n}', String(dirty))}</Text>
            <ScrollView style={{ maxHeight: 150 }}>
              {diffs.map((d) => (
                <View key={ekey(d.sec, d.key)} style={[styles.iniDiff, { borderTopColor: c.line2 }]}>
                  <Text numberOfLines={1} style={{ flex: 1, color: c.dim, fontSize: 10, fontFamily: fonts.mono }}>[{d.sec}] {d.key}</Text>
                  <Text style={{ color: c.redTx, fontSize: 10, fontFamily: fonts.mono, textDecorationLine: 'line-through' }}>{d.old}</Text>
                  <Text style={{ color: c.dim, fontSize: 10 }}>→</Text>
                  <Text style={{ color: c.greenTx, fontSize: 10, fontFamily: fonts.mono, fontWeight: '700' }}>{d.next}</Text>
                </View>
              ))}
            </ScrollView>
            <Text style={{ color: c.muted, fontSize: 10.5, marginTop: 10, lineHeight: 15 }}>
              {t('변경은 ')}<Text style={{ color: c.amberTx, fontWeight: '700' }}>{tgt.daemon} {t('데몬 재시작 후 반영')}</Text>{t('됩니다 — 자동 재시작 없음.')}
            </Text>
            <View style={{ flexDirection: 'row', gap: 9, marginTop: 14 }}>
              <Tappable onPress={() => !saving && setConfirm(false)}
                style={[styles.iniModalBtn, { backgroundColor: c.elev, borderColor: c.line, borderRadius: radius.md }]}>
                <Text style={{ color: c.text, fontSize: 12, fontWeight: '600' }}>{t('취소')}</Text>
              </Tappable>
              <Tappable onPress={() => !saving && save()}
                style={[styles.iniModalBtn, { backgroundColor: 'rgba(231,51,28,0.85)', borderColor: c.dangerLine, borderRadius: radius.md }]}>
                <Icon name="save" size={13} color="#fff" />
                <Text style={{ color: '#fff', fontSize: 12, fontWeight: '600' }}>{saving ? t('저장 중…') : t('저장')}</Text>
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
  iniSearch: { flexDirection: 'row', alignItems: 'center', gap: 6, height: 30, paddingHorizontal: 9, borderWidth: 1, width: 190 },
  iniSec: { borderWidth: 1, paddingHorizontal: 10, paddingVertical: 7 },
  iniRow: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 2.5 },
  iniBool: { alignItems: 'center', justifyContent: 'center', height: 24, minWidth: 56, paddingHorizontal: 8, borderWidth: 1 },
  iniInput: { height: 24, minWidth: 56, maxWidth: 170, paddingHorizontal: 8, paddingVertical: 0, borderWidth: 1, fontSize: 9.5, textAlign: 'right' },
  iniBar: { flexDirection: 'row', alignItems: 'center', gap: 9, paddingTop: 10, marginTop: 8, borderTopWidth: 1 },
  iniModal: { width: 440, borderWidth: 1, padding: 18 },
  iniDiff: { flexDirection: 'row', alignItems: 'center', gap: 8, paddingVertical: 5, borderTopWidth: 1 },
  iniModalBtn: { flex: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 6, height: 38, borderWidth: 1 },
});
