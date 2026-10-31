import { View, Text, StyleSheet, Platform, useWindowDimensions } from 'react-native';
import { usePathname } from 'expo-router';
import { useSafeAreaInsets } from 'react-native-safe-area-context';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { Popover } from '@/components/ui/overlays';
import { useHubNav, type NavItem } from '@/components/hub/hubNav';
import { goTop } from '@/lib/nav';
import { t } from '@/lib/i18n';
import { EDGE } from '@/lib/layout';

export function LogoMenu({ onClose, beforeNavigate }: { onClose: () => void; beforeNavigate?: (next:()=>void)=>void }) {
  const { c, radius } = useTheme();
  const path = usePathname();
  const insets = useSafeAreaInsets();
  const { height: winH } = useWindowDimensions();
  const { control, update, pills } = useHubNav();
  const hub: NavItem = { key: 'hub', icon: 'apps', label: t('허브'), on: true, go: () => goTop('/hub'), path: '/hub' };
  const row = (it: NavItem) => {
    const here = it.path === path;
    const tint = here ? c.accent2 : it.on ? c.text : c.muted;
    return (
      <Tappable key={it.key} disabled={!it.on || here} onPress={() => { onClose(); beforeNavigate ? beforeNavigate(it.go) : it.go(); }}
        accessibilityLabel={it.label}
        style={[styles.item, { borderRadius: radius.sm, opacity: it.on ? 1 : 0.45 }, here && { backgroundColor: c.elev }]}>
        <Icon name={it.icon} size={16} color={tint} />
        <Text numberOfLines={1} style={{ color: tint, fontSize: 13.5, fontWeight: here ? '700' : '600', flex: 1 }}>{it.label}</Text>
        {it.dot ? <View style={[styles.dot, { backgroundColor: c.accent }]} /> : null}
      </Tappable>
    );
  };
  return (
    <Popover onClose={onClose} style={[Platform.OS === 'android' ? { left: EDGE + insets.left, top: 56 } : { left: -1, top: 51 }, { width: 210, maxHeight: winH - 64 }]}>
      {[hub, control, ...pills, ...(update ? [update] : [])].map(row)}
    </Popover>
  );
}

const styles = StyleSheet.create({
  item: { flexDirection: 'row', alignItems: 'center', gap: 10, paddingHorizontal: 10, paddingVertical: 10 },
  dot: { width: 8, height: 8, borderRadius: 4 },
});
