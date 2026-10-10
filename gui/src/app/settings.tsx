import { useEffect, useState } from 'react';
import { View, Text, StyleSheet, ScrollView, Platform } from 'react-native';
import { useRobots, useProfiles, SIM_SERIAL, SIM_PROFILE } from '@/store/robots';
import { useNotMine } from '@/lib/spectating';
import { useSteamOS } from '@/lib/platformInfo';
import { isDesktop } from '@/lib/desktopBridge';
import { currentAppVersion } from '@/lib/appSelfUpdate';
import Animated, { FadeIn } from 'react-native-reanimated';
import { useRouter, useLocalSearchParams } from 'expo-router';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { useGamepad } from '@/store/gamepad';
import { GamepadRemapWizard } from '@/components/GamepadRemapWizard';
import { GamepadPanel } from '@/components/panels/settings/GamepadPanel';
import { PayloadPanel } from '@/components/panels/settings/PayloadPanel';
import { AppPanel } from '@/components/panels/settings/AppPanel';
import { PowerBasicPanel } from '@/components/panels/settings/PowerBasicPanel';
import { t } from '@/lib/i18n';
import { useCompactH } from '@/lib/layout';

type Sec = 'gp' | 'pwr' | 'pay' | 'app';
type Scope = 'robot' | 'app';
const SNAV: { key: Sec; label: string; icon: IconName; scope: Scope }[] = [
  { key: 'pwr', label: '전원 제어', icon: 'power', scope: 'robot' },
  { key: 'pay', label: '페이로드 설정', icon: 'box', scope: 'robot' },
  { key: 'gp', label: '게임패드', icon: 'joystick', scope: 'app' },
  { key: 'app', label: '앱 설정', icon: 'palette', scope: 'app' },
];

function Body({ sec, onWizard }: { sec: Sec; onWizard?: () => void }) {
  const router = useRouter();
  if (sec === 'gp') return <GamepadPanel onWizard={onWizard} onOpenDiag={() => router.navigate('/gamepad')} />;
  if (sec === 'pwr') return <PowerBasicPanel />;
  if (sec === 'pay') return <PayloadPanel />;
  return <AppPanel />;
}


export default function Settings() {
  const { c, radius } = useTheme();
  const { sec: secParam } = useLocalSearchParams<{ sec?: string }>();
  const serial = useRobots((s) => s.currentSerial);
  const sim = serial === SIM_SERIAL;
  const notMine = useNotMine();
  const open = (k: Sec) => (!sim && !notMine) || SNAV.find((s) => s.key === k)!.scope === 'app';
  const pick = (v?: string): Sec => {
    const k = SNAV.find((s) => s.key === v)?.key ?? 'pwr';
    return open(k) ? k : 'gp';
  };
  const [sec, setSec] = useState<Sec>(() => pick(secParam));
  useEffect(() => { if (secParam) setSec(pick(secParam)); }, [secParam]); // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => { if (!open(sec)) setSec('gp'); }, [sim, notMine]); // eslint-disable-line react-hooks/exhaustive-deps
  const [wizard, setWizard] = useState(false);
  const gpDev = useGamepad((s) => s.devices[0] ?? null);
  const compact = useCompactH();
  const profiles = useProfiles();
  const cur = serial === SIM_SERIAL ? SIM_PROFILE : profiles.find((p) => p.serial === serial);
  const steamOS = useSteamOS();
  const device = Platform.OS === 'web' ? (steamOS ? 'Steam Deck' : isDesktop() ? 'PC' : 'Web') : Platform.OS === 'ios' ? 'iOS' : 'Android';
  const subtitle = `${cur ? `${cur.name}${cur.serial !== SIM_SERIAL ? ` · ${cur.serial}` : ''}` : t('로봇 없음')} · ${device} · ${currentAppVersion()}`;
  return (
    <Screen>
      <HubHeader title={t('설정')} subtitle={subtitle} />
      <View style={[styles.tabs, compact && { paddingHorizontal: 10 }]}>
        {SNAV.map((s, i) => {
          const on = s.key === sec;
          const can = open(s.key);
          return (
            <Tappable key={s.key} onPress={can ? () => setSec(s.key) : undefined} disabled={!can} accessibilityLabel={t(s.label)}
              style={[styles.tab, { backgroundColor: on ? 'rgba(77,156,245,0.18)' : c.glass, borderColor: on ? 'rgba(77,156,245,0.6)' : c.glassLine, borderRadius: radius.md, opacity: can ? 1 : 0.45 },
                i > 0 && SNAV[i - 1].scope !== s.scope && styles.groupGap]}>
              <Icon name={s.icon} size={15} color={on ? c.accent2 : c.muted} />
              {!compact && <Text style={{ color: on ? c.text : c.muted, fontSize: 12.5, fontWeight: '600' }}>{t(s.label)}</Text>}
            </Tappable>
          );
        })}
      </View>
      <View style={[styles.panel, { backgroundColor: c.glassHi, borderColor: c.glassLine, borderRadius: radius.lg }, compact && { marginHorizontal: 10, marginBottom: 10, paddingHorizontal: 14, paddingVertical: 10 }]}>
        <Animated.View key={sec} entering={FadeIn.duration(160)} style={{ flex: 1 }}>
          <ScrollView showsVerticalScrollIndicator={false}><Body sec={sec} onWizard={() => setWizard(true)} /></ScrollView>
        </Animated.View>
      </View>
      {wizard && gpDev && <GamepadRemapWizard dev={gpDev} onClose={() => setWizard(false)} />}
    </Screen>
  );
}

const styles = StyleSheet.create({
  tabs: { flexDirection: 'row', gap: 8, paddingHorizontal: 16, paddingTop: 4, paddingBottom: 10, flexWrap: 'wrap' },
  groupGap: { marginLeft: 12 },
  tab: { height: 36, paddingHorizontal: 14, borderWidth: 1, flexDirection: 'row', alignItems: 'center', gap: 8 },
  panel: { flex: 1, marginHorizontal: 16, marginBottom: 16, borderWidth: 1, paddingHorizontal: 22, paddingVertical: 16, overflow: 'hidden' },
  wrap: { flex: 1, flexDirection: 'row', padding: 15, gap: 14 },
  snav: { width: 190, borderWidth: 1, borderRadius: 14, padding: 8, gap: 3 },
  snavItem: { flexDirection: 'row', alignItems: 'center', gap: 11, padding: 11, borderRadius: 9 },
  sbody: { flex: 1, borderWidth: 1, borderRadius: 14, paddingHorizontal: 22, paddingVertical: 18 },
});
