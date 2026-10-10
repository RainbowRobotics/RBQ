import { useEffect, useState } from 'react';
import { View, Text, ScrollView, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { HTabs } from '@/components/ui/HTabs';
import { CATEGORIES } from '@/gallery';
import type { WidgetCase } from '@/gallery/types';
import { Frame } from '@/gallery/Frame';

export default function WidgetsScreen() {
  const { c, name, toggle } = useTheme();
  const [key, setKey] = useState(CATEGORIES[0].key);
  const [cases, setCases] = useState<WidgetCase[] | null>(null);
  const cat = CATEGORIES.find((x) => x.key === key) ?? CATEGORIES[0];

  useEffect(() => {
    let alive = true;
    setCases(null);
    void cat.load().then((cs) => { if (alive) setCases(cs); });
    return () => { alive = false; };
  }, [cat]);
  return (
    <View style={{ flex: 1, backgroundColor: c.bg }}>
      <View style={[styles.head, { borderBottomColor: c.line, backgroundColor: c.panel }]}>
        <Icon name="palette" size={18} color={c.accent2} />
        <Text style={{ color: c.text, fontSize: 15, fontWeight: '700' }}>위젯 갤러리</Text>
        <Text style={{ color: c.dim, fontSize: 11, flex: 1 }}>신UI 위젯 · legacy 제외</Text>
        <Tappable onPress={toggle} style={[styles.btn, { backgroundColor: c.elev, borderColor: c.line }]}>
          <Text style={{ color: c.text, fontSize: 11, fontWeight: '600' }}>{name === 'dark' ? '라이트' : '다크'}로</Text>
        </Tappable>
      </View>

      <HTabs items={CATEGORIES.map(({ key: k, label, icon }) => ({ key: k, label, icon }))} value={key} onChange={setKey} />

      <ScrollView contentContainerStyle={styles.grid}>
        {cases
          ? cases.map((w) => <Frame key={w.name} c={w} />)
          : <Text style={{ color: c.dim, fontSize: 12, padding: 12 }}>불러오는 중…</Text>}
      </ScrollView>
    </View>
  );
}

const styles = StyleSheet.create({
  head: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingHorizontal: 14, paddingVertical: 10, borderBottomWidth: 1 },
  btn: { paddingHorizontal: 14, paddingVertical: 8, borderRadius: 9, borderWidth: 1 },
  grid: { flexDirection: 'row', flexWrap: 'wrap', gap: 12, padding: 12, alignItems: 'flex-start' },
});
