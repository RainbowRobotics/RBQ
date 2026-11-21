import { ScrollView, View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';

export function HTabs<K extends string>({ items, value, onChange }: {
  items: { key: K; label: string; icon?: IconName }[]; value: K; onChange: (k: K) => void;
}) {
  const { c } = useTheme();
  return (
    <ScrollView horizontal showsHorizontalScrollIndicator={false} style={{ flexGrow: 0, flexShrink: 0 }}
      contentContainerStyle={styles.row}>
      {items.map((it) => {
        const on = it.key === value;
        return (
          <Tappable key={it.key} onPress={() => onChange(it.key)}
            style={[styles.pill, {
              backgroundColor: on ? 'rgba(77,156,245,0.10)' : c.elev,
              borderColor: on ? 'rgba(77,156,245,0.5)' : c.line,
            }]}>
            {it.icon && <Icon name={it.icon} size={13} color={on ? c.accent2 : c.muted} />}
            <Text style={{ color: on ? c.accent2 : c.muted, fontSize: 12, fontWeight: '600' }}>{it.label}</Text>
          </Tappable>
        );
      })}
    </ScrollView>
  );
}

const styles = StyleSheet.create({
  row: { flexDirection: 'row', gap: 6, paddingHorizontal: 12, paddingVertical: 9 },
  pill: { flexDirection: 'row', alignItems: 'center', gap: 6, paddingHorizontal: 14, paddingVertical: 7, borderRadius: 999, borderWidth: 1 },
  cbody: { flex: 1, marginHorizontal: 12, marginBottom: 12, borderWidth: 1, borderRadius: 14, paddingHorizontal: 18, paddingVertical: 16 },
});
export const compactBodyStyle = styles.cbody;
