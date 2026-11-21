import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { useRb } from '@/rb/theme';
import { RBBadge } from '@/rb/components/RBBadge';
import { RBLabelButton } from '@/rb/components/RBLabelButton';
import { Icon, type IconName } from '@/components/Icon';
import { verdictTone, verdictWord } from '@/lib/ptzQc';

export function Card({ n, title, right, children, style }: { n?: string; title?: string; right?: React.ReactNode; children: React.ReactNode; style?: object }) {
  const { c, radius } = useTheme();
  const { c: rc } = useRb();
  return (
    <View style={[styles.card, { backgroundColor: c.panel, borderColor: c.line, borderRadius: radius.md }, style]}>
      {title ? (
        <View style={styles.cardHead}>
          {n ? (
            <View style={[styles.num, { backgroundColor: rc('fill-brand') }]}>
              <Text style={{ color: rc('fg-static-white'), fontSize: 14, fontWeight: '800' }}>{n}</Text>
            </View>
          ) : null}
          <Text style={{ color: c.text, fontSize: 16, fontWeight: '700', flex: 1 }}>{title}</Text>
          {right}
        </View>
      ) : null}
      {children}
    </View>
  );
}

export function VerdictChip({ v, big }: { v: string | null | undefined; big?: boolean }) {
  const { c, fonts, radius } = useTheme();
  const tone = verdictTone(v);
  const fg = { green: c.greenTx, orange: c.amberTx, red: c.redTx, gray: c.muted, blue: c.accent }[tone];
  const bg = { green: 'rgba(63,185,80,0.14)', orange: 'rgba(240,136,62,0.16)', red: 'rgba(231,51,28,0.13)', gray: c.elev, blue: 'rgba(77,156,245,0.15)' }[tone];
  return (
    <View style={{ alignSelf: 'flex-start', paddingHorizontal: big ? 12 : 8, paddingVertical: big ? 5 : 2, borderRadius: radius.sm, backgroundColor: bg }}>
      <Text style={{ color: fg, fontFamily: fonts.mono, fontWeight: '700', fontSize: big ? 15 : 11.5 }}>{verdictWord(v)}</Text>
    </View>
  );
}

export function KvRow({ k, v, mono, extra }: { k: string; v: React.ReactNode; mono?: boolean; extra?: React.ReactNode }) {
  const { c, fonts } = useTheme();
  return (
    <View style={styles.kv}>
      <Text style={{ color: c.muted, fontSize: 13, width: 112 }}>{k}</Text>
      <View style={[styles.row, { flex: 1, gap: 8 }]}>
        {typeof v === 'string' ? <Text style={{ color: c.text, fontSize: 13.5, fontFamily: mono ? fonts.mono : undefined, flexShrink: 1 }}>{v}</Text> : v}
        {extra}
      </View>
    </View>
  );
}

export function Bar({ pct, tone }: { pct: number; tone?: 'brand' | 'danger' }) {
  const { c: rc } = useRb();
  return (
    <View style={{ height: 8, borderRadius: 4, backgroundColor: rc('bg-alt-2'), overflow: 'hidden' }}>
      <View style={{ height: 8, width: `${Math.round(Math.max(0, Math.min(1, pct)) * 100)}%`, backgroundColor: tone === 'danger' ? rc('fg-danger') : rc('fill-brand') }} />
    </View>
  );
}

export function Note({ tone = 'info', icon, title, text, right }: { tone?: 'info' | 'warn' | 'danger'; icon?: IconName; title?: string; text?: string; right?: React.ReactNode }) {
  const { c, radius } = useTheme();
  const { c: rc } = useRb();
  const fill = { info: rc('fill-information-subtlest'), warn: rc('fill-caution-subtlest'), danger: rc('fill-danger-subtlest') }[tone];
  const line = { info: rc('border-information'), warn: rc('border-caution'), danger: rc('border-danger') }[tone];
  const fg = { info: rc('fg-information'), warn: rc('fg-warning'), danger: rc('fg-danger') }[tone];
  return (
    <View style={[styles.note, { backgroundColor: fill, borderColor: line, borderRadius: radius.sm }]}>
      <View style={{ paddingTop: 1 }}><Icon name={icon ?? (tone === 'info' ? 'ptz' : 'warn')} size={15} color={fg} /></View>
      <Text style={{ color: c.text, fontSize: 13, flex: 1, lineHeight: 19 }}>
        {title ? <Text style={{ fontWeight: '700' }}>{title}</Text> : null}
        {title && text ? ' ' : null}
        {text}
      </Text>
      {right}
    </View>
  );
}

export function BigBtn({ kind, icon, label, onPress, disabled, small }: { kind: 'primary' | 'ghost' | 'danger'; icon: IconName; label: string; onPress: () => void; disabled?: boolean; small?: boolean }) {
  const { c } = useRb();
  const variant = kind === 'primary' ? 'brand' : kind === 'danger' ? 'danger' : 'outlined';
  return (
    <RBLabelButton size={small ? 'sm' : 'md'} variant={variant as any} disabled={disabled} onPress={onPress}
      leadingIcon={<Icon name={icon} size={small ? 14 : 17} color={kind === 'ghost' ? c('fg-default') : c('fg-static-white')} />}>{label}</RBLabelButton>
  );
}

export const Tag = ({ label, color = 'gray' }: { label: string; color?: 'gray' | 'green' | 'blue' | 'orange' | 'red' }) =>
  <RBBadge type="soft" color={color}>{label}</RBBadge>;

export const bareName = (name: string) => name.replace(/^\s*\d+\s*/, '');

export const styles = StyleSheet.create({
  card: { borderWidth: 1, padding: 14 },
  cardHead: { flexDirection: 'row', alignItems: 'center', gap: 10, marginBottom: 12 },
  num: { width: 26, height: 26, borderRadius: 13, alignItems: 'center', justifyContent: 'center' },
  kv: { flexDirection: 'row', alignItems: 'center', paddingVertical: 4, gap: 8 },
  row: { flexDirection: 'row', alignItems: 'center', flexWrap: 'wrap' },
  note: { flexDirection: 'row', alignItems: 'flex-start', gap: 8, borderWidth: 1, paddingHorizontal: 12, paddingVertical: 10 },
});
