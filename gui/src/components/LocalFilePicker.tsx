import { useEffect, useState } from 'react';
import { View, Text, ScrollView, StyleSheet, useWindowDimensions } from 'react-native';
import { useTheme } from '@/theme';
import { Modal } from '@/components/ui/overlays';
import { Tappable } from '@/components/anim';
import { Icon } from '@/components/Icon';
import { acceptsName, finishLocalPick, useLocalPicker } from '@/lib/pickDocument';
import { t } from '@/lib/i18n';

type Entry = { name: string; dir: boolean; size: number; mtime: number };
type Listing = { dir: string; parent: string | null; roots: { label: string; path: string }[]; entries: Entry[] };

const join = (dir: string, name: string) => (dir.endsWith('/') ? dir + name : `${dir}/${name}`);
const human = (n: number) => (n >= 1 << 20 ? `${(n / (1 << 20)).toFixed(1)} MB` : `${Math.max(1, Math.round(n / 1024))} KB`);

export function LocalFilePicker() {
  const req = useLocalPicker((s) => s.req);
  const { c, radius } = useTheme();
  const { width: winW } = useWindowDimensions();
  const [dir, setDir] = useState<string | null>(null);
  const [list, setList] = useState<Listing | null>(null);
  const [err, setErr] = useState('');
  useEffect(() => { if (req) { setDir(null); setList(null); } }, [req]);
  useEffect(() => {
    if (!req) return;
    let alive = true;
    setErr('');
    fetch(`/local/list${dir ? `?dir=${encodeURIComponent(dir)}` : ''}`)
      .then(async (r) => { const j = await r.json(); if (!r.ok) throw new Error(j?.error || String(r.status)); return j as Listing; })
      .then((j) => { if (alive) setList(j); })
      .catch((e) => { if (alive) setErr(`${t('폴더를 열지 못했습니다')} (${(e as Error).message})`); });
    return () => { alive = false; };
  }, [req, dir]);
  if (!req) return null;
  const rows = (list?.entries ?? []).filter((e) => e.dir || acceptsName(req.type, e.name));
  return (
    <Modal onClose={() => finishLocalPick(null)} fit>
      <View style={[S.card, { width: Math.min(winW - 24, 1100), backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.lg }]}>
        <View style={S.head}>
          <Text style={{ color: c.text, fontSize: 18, fontWeight: '700', flex: 1 }}>{t('파일 고르기')}</Text>
          <Tappable onPress={() => finishLocalPick(null)} accessibilityLabel={t('닫기')} style={[S.close, { borderColor: c.line }]}>
            <Icon name="x" size={16} color={c.text} />
          </Tappable>
        </View>
        <View style={S.roots}>
          {(list?.roots ?? []).map((r) => (
            <Tappable key={r.path} onPress={() => setDir(r.path)}
              style={[S.root, { borderColor: list?.dir === r.path ? c.accent2 : c.line, borderRadius: radius.md }]}>
              <Text style={{ color: list?.dir === r.path ? c.accent2 : c.text, fontSize: 15, fontWeight: '600' }}>{t(r.label)}</Text>
            </Tappable>
          ))}
        </View>
        <View style={S.pathRow}>
          <Tappable onPress={() => list?.parent && setDir(list.parent)} accessibilityLabel={t('위로')}
            style={[S.up, { borderColor: c.line, borderRadius: radius.md, opacity: list?.parent ? 1 : 0.4 }]}>
            <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>↑ {t('위로')}</Text>
          </Tappable>
          <Text numberOfLines={1} style={{ color: c.muted, fontSize: 14, flex: 1 }}>{list?.dir ?? ''}</Text>
        </View>
        <ScrollView style={{ flex: 1 }} contentContainerStyle={{ paddingBottom: 8 }}>
          {err ? <Text style={{ color: c.redbright, fontSize: 12.5, padding: 12 }}>{err}</Text> : null}
          {!err && list && rows.length === 0 && (
            <Text style={{ color: c.muted, fontSize: 12.5, padding: 12 }}>{t('고를 수 있는 파일이 없습니다')}</Text>
          )}
          {rows.map((e) => (
            <Tappable key={e.name} style={[S.row, { borderBottomColor: c.line }]}
              onPress={() => (e.dir ? setDir(join(list!.dir, e.name)) : finishLocalPick({ path: join(list!.dir, e.name), name: e.name, size: e.size }))}>
              <Text style={{ color: e.dir ? c.accent2 : c.text, fontSize: 17, width: 26 }}>{e.dir ? '▸' : '•'}</Text>
              <Text numberOfLines={1} style={{ color: c.text, fontSize: 16, flex: 1 }}>{e.name}</Text>
              {!e.dir && <Text style={{ color: c.muted, fontSize: 14 }}>{human(e.size)}</Text>}
            </Tappable>
          ))}
        </ScrollView>
      </View>
    </Modal>
  );
}

const S = StyleSheet.create({
  card: { flex: 1, borderWidth: 1, padding: 18, gap: 12 },
  head: { flexDirection: 'row', alignItems: 'center', gap: 10 },
  close: { width: 48, height: 48, borderWidth: 1, borderRadius: 24, alignItems: 'center', justifyContent: 'center' },
  roots: { flexDirection: 'row', flexWrap: 'wrap', gap: 8 },
  root: { paddingHorizontal: 16, height: 46, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  pathRow: { flexDirection: 'row', alignItems: 'center', gap: 10 },
  up: { paddingHorizontal: 16, height: 46, borderWidth: 1, alignItems: 'center', justifyContent: 'center' },
  row: { flexDirection: 'row', alignItems: 'center', gap: 8, minHeight: 58, paddingHorizontal: 8, borderBottomWidth: StyleSheet.hairlineWidth },
});
