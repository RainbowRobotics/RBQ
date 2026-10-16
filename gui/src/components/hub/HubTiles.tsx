import { View, Text, StyleSheet } from 'react-native';
import { openExternal } from '@/lib/openExternal';
import { useRouter, type Href } from 'expo-router';
import { useRobotKind } from '@/modules/registry';
import { useTheme } from '@/theme';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useRobots, useRobotReady, SIM_SERIAL } from '@/store/robots';
import { useNotMine } from '@/lib/spectating';
import { CHANNEL } from '@/lib/firmwareRelease';
import { t } from '@/lib/i18n';
import { goTop } from '@/lib/nav';

export const GUIDE_URL = CHANNEL === 'nightly'
  ? 'https://rainbowrobotics.github.io/RBQ/nightly/'
  : 'https://rainbowrobotics.github.io/RBQ/';

export function HubTiles({ compact }: { compact?: boolean }) {
  const { c, radius } = useTheme();
  const kind = useRobotKind();
  const router = useRouter();
  const ready = useRobotReady();
  const isSim = useRobots((s) => s.currentSerial) === SIM_SERIAL;
  const online = ready || isSim;
  const notMine = useNotMine();
  const w = compact ? 86 : 108;
  const PER_ROW = 3;
  const rowW = w * PER_ROW + 10 * (PER_ROW - 1);
  const pill = (key: IconName, label: string, on: boolean, go: () => void) => (
    <Tappable key={label} onPress={on ? go : undefined} disabled={!on} accessibilityLabel={label}
      style={[styles.pill, { width: w, height: compact ? 56 : 64, backgroundColor: c.glass, borderColor: c.glassLine, borderRadius: radius.md, opacity: on ? 1 : 0.45 }]}>
      <Icon name={key} size={20} color={on ? c.accent2 : c.muted} />
      <Text numberOfLines={1} adjustsFontSizeToFit minimumFontScale={0.8}
        style={{ color: on ? c.text : c.muted, fontSize: 11.5, fontWeight: '700' }}>{label}</Text>
    </Tappable>
  );
  return (
    <View style={styles.col}>
      {!kind && <View style={[styles.row, { width: rowW }]}>
        {pill('gauge', t('대시보드'), ready && !notMine, () => router.push('/dashboard'))}
        {pill('sliders', t('설정'), true, () => router.push('/settings'))}
        {pill('wrench', t('정비'), !notMine, () => router.push('/maintenance'))}
        {pill('log', t('로그'), true, () => router.push('/log'))}
        {pill('save', t('미디어'), ready, () => router.push('/media'))}
        {pill('book', t('가이드'), true, () => openExternal(GUIDE_URL))}
      </View>}
      <Tappable onPress={online ? () => goTop((kind?.route ?? '/') as Href) : undefined} disabled={!online} accessibilityLabel={t('컨트롤')}
        style={[styles.main, { height: compact ? 64 : 76, borderRadius: radius.lg, opacity: online ? 1 : 0.45,
          minWidth: kind ? (compact ? 320 : 380) : undefined,
          backgroundColor: online ? 'rgba(77,156,245,0.22)' : c.glass, borderColor: online ? 'rgba(77,156,245,0.65)' : c.glassLine }]}>
        <Icon name="gamepad" size={28} color={online ? c.accent2 : c.muted} />
        <Text style={{ color: online ? c.text : c.muted, fontSize: 19, fontWeight: '800', letterSpacing: 0.3 }}>{t('컨트롤')}</Text>
        {!online && <Text style={{ color: c.muted, fontSize: 11 }}>{t('연결되면 열립니다')}</Text>}
      </Tappable>
    </View>
  );
}
const styles = StyleSheet.create({
  col: { gap: 10 },
  row: { flexDirection: 'row', flexWrap: 'wrap', gap: 10 },
  pill: { borderWidth: 1, alignItems: 'center', justifyContent: 'center', gap: 5 },
  main: { borderWidth: 1, flexDirection: 'row', alignItems: 'center', justifyContent: 'center', gap: 12 },
});
