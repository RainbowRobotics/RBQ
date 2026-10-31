import { useLocalSearchParams, useRouter } from 'expo-router';
import { View, Text, StyleSheet } from 'react-native';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { FirmwarePanel } from '@/components/panels/FirmwarePanel';
import { BoardFirmwarePanel } from '@/components/panels/BoardFirmwarePanel';
import { t } from '@/lib/i18n';
import { useCompactH, EDGE } from '@/lib/layout';

type Tab = 'sw' | 'fw';
const TABS: { key: Tab; label: string; icon: IconName }[] = [
  { key: 'sw', label: '소프트웨어 업데이트', icon: 'download' },
  { key: 'fw', label: '펌웨어 업데이트', icon: 'cpu' },
];

export default function Update() {
  const { c, radius } = useTheme();
  const compact = useCompactH();
  const router = useRouter();
  const { tab: tabParam } = useLocalSearchParams<{ tab?: string }>();
  const tab: Tab = tabParam === 'fw' ? 'fw' : 'sw';
  return (
    <Screen>
      <HubHeader title={t('업데이트')} subtitle={t(TABS.find((x) => x.key === tab)!.label)} />
      <View style={styles.tabs}>
        {TABS.map((s) => {
          const on = s.key === tab;
          return (
            <Tappable key={s.key} onPress={() => router.setParams({ tab: s.key })} accessibilityLabel={t(s.label)}
              style={[styles.tab, { backgroundColor: on ? 'rgba(77,156,245,0.18)' : c.glass, borderColor: on ? 'rgba(77,156,245,0.6)' : c.glassLine, borderRadius: radius.md }]}>
              <Icon name={s.icon} size={15} color={on ? c.accent2 : c.muted} />
              <Text style={{ color: on ? c.text : c.muted, fontSize: 12.5, fontWeight: '600' }}>{t(s.label)}</Text>
            </Tappable>
          );
        })}
      </View>
      <View style={[styles.panel, { backgroundColor: c.glassHi, borderColor: c.glassLine, borderRadius: radius.lg }, compact && { paddingHorizontal: 14, paddingVertical: 10 }]}>
        {tab === 'sw' ? <FirmwarePanel /> : <BoardFirmwarePanel />}
      </View>
    </Screen>
  );
}

const styles = StyleSheet.create({
  tabs: { flexDirection: 'row', gap: 8, paddingHorizontal: EDGE, paddingBottom: 8, flexWrap: 'wrap' },
  tab: { height: 36, paddingHorizontal: 14, borderWidth: 1, flexDirection: 'row', alignItems: 'center', gap: 8 },
  panel: { flex: 1, marginHorizontal: EDGE, marginBottom: EDGE, borderWidth: 1, paddingHorizontal: 22, paddingVertical: 16, overflow: 'hidden' },
});
