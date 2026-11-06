import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { t } from '@/lib/i18n';
import { useHubNav, type NavItem } from '@/components/hub/hubNav';

export function HubTiles({ compact }: { compact?: boolean }) {
  const { c, radius } = useTheme();
  const { control, update, pills } = useHubNav();
  const online = control.on;
  const w = compact ? 86 : 108;
  const PER_ROW = 3;
  const rowW = w * PER_ROW + 10 * (PER_ROW - 1);
  const mainH = compact ? 64 : 76;
  const pill = ({ icon: key, label, on, go, dot, dotHint }: NavItem, h = compact ? 56 : 64) => (
    <Tappable key={label} onPress={on ? go : undefined} disabled={!on} accessibilityLabel={dot && dotHint ? `${label} — ${dotHint}` : label}
      style={[styles.pill, { width: w, height: h, backgroundColor: c.glass, borderColor: c.glassLine, borderRadius: radius.md, opacity: on ? 1 : 0.45 }]}>
      <Icon name={key} size={20} color={on ? c.accent2 : c.muted} />
      {dot ? <View style={[styles.dot, { backgroundColor: c.accent }]} /> : null}
      <Text numberOfLines={1} adjustsFontSizeToFit minimumFontScale={0.8}
        style={{ color: on ? c.text : c.muted, fontSize: 11.5, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
  return (
    <View style={styles.col}>
      {pills.length > 0 && <View style={[styles.row, { width: rowW }]}>{pills.map((p) => pill(p))}</View>}
      <View style={[styles.bottom, update ? { width: rowW } : null]}>
        {update ? pill(update, mainH) : null}
        <Tappable onPress={online ? control.go : undefined} disabled={!online} accessibilityLabel={t('컨트롤')}
          style={[styles.main, { flex: update ? 1 : undefined, height: mainH, borderRadius: radius.lg, opacity: online ? 1 : 0.45,
            minWidth: pills.length === 0 ? (compact ? 320 : 380) : undefined,
            backgroundColor: online ? 'rgba(77,156,245,0.22)' : c.glass, borderColor: online ? 'rgba(77,156,245,0.65)' : c.glassLine }]}>
          <View style={styles.mainRow}>
            <Icon name="gamepad" size={28} color={online ? c.accent2 : c.muted} />
            <Text numberOfLines={1} style={{ color: online ? c.text : c.muted, fontSize: 19, fontWeight: '800', letterSpacing: 0.3 }}>{t('컨트롤')}</Text>
            {!online && !update && <Text style={{ color: c.muted, fontSize: 11 }}>{t('연결되면 열립니다')}</Text>}
          </View>
          {!online && update && <Text numberOfLines={1} style={{ color: c.muted, fontSize: 11 }}>{t('연결되면 열립니다')}</Text>}
        </Tappable>
      </View>
    </View>
  );
}
const styles = StyleSheet.create({
  col: { gap: 10 },
  row: { flexDirection: 'row', flexWrap: 'wrap', gap: 10 },
  bottom: { flexDirection: 'row', gap: 10 },
  pill: { borderWidth: 1, alignItems: 'center', justifyContent: 'center', gap: 5 },
  dot: { position: 'absolute', top: 8, right: 10, width: 9, height: 9, borderRadius: 5 },
  main: { borderWidth: 1, alignItems: 'center', justifyContent: 'center', gap: 2 },
  mainRow: { flexDirection: 'row', alignItems: 'center', gap: 12 },
});
