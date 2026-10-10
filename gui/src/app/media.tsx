import { useEffect, useState } from 'react';
import { View, Text, StyleSheet, ScrollView } from 'react-native';
import Animated, { FadeIn } from 'react-native-reanimated';
import { useLocalSearchParams } from 'expo-router';
import { useTheme } from '@/theme';
import { Screen } from '@/components/Screen';
import { HubHeader } from '@/components/hub/HubHeader';
import { Icon, type IconName } from '@/components/Icon';
import { Tappable } from '@/components/anim';
import { HTabs, compactBodyStyle } from '@/components/ui/HTabs';
import { MediaPhotos, MediaClips, MediaArchive, MediaAudio, MediaSounds } from '@/components/panels/MediaPanel';
import { media, useMedia, groupCaptures, type MediaKind } from '@/lib/media';
import { useRobot } from '@/store/robot';
import { useCompactH } from '@/lib/layout';
import { t } from '@/lib/i18n';

type Sec = MediaKind | 'sound';
const SNAV: { key: Sec; label: string; icon: IconName }[] = [
  { key: 'photo', label: '사진', icon: 'camera' },
  { key: 'clip', label: '녹화', icon: 'play2' },
  { key: 'archive', label: '블랙박스', icon: 'calendar' },
  { key: 'audio', label: '녹음', icon: 'mic' },
  { key: 'sound', label: '사운드', icon: 'speaker' },
];

function useCount(sec: Sec): number | null {
  return useMedia((s) => {
    const kind: MediaKind = sec === 'sound' ? 'audio' : sec;
    const l = s.lists[kind];
    if (!l.loaded) return null;
    if (sec === 'sound') return l.files.filter((f) => f.path.startsWith('library/my-')).length;
    if (kind === 'audio') return l.files.filter((f) => !f.path.startsWith('library/')).length;
    return kind === 'photo' ? groupCaptures(l.files).length : l.files.length;
  });
}

function NavItem({ k, on, onPress }: { k: (typeof SNAV)[number]; on: boolean; onPress: () => void }) {
  const { c, fonts } = useTheme();
  const n = useCount(k.key);
  const rec = useMedia((s) => s.recordingPath != null);
  const vid = useMedia((s) => s.videoSince !== 0);
  return (
    <Tappable onPress={onPress} style={[styles.snavItem, on && { backgroundColor: 'rgba(77,156,245,0.12)' }]}>
      <Icon name={k.icon} size={16} color={on ? c.accent2 : c.muted} />
      <Text style={{ color: on ? c.text : c.muted, fontSize: 12.5, flex: 1 }}>{t(k.label)}</Text>
      {(k.key === 'audio' && rec) || (k.key === 'clip' && vid)
        ? <View style={[styles.recDot, { backgroundColor: c.red }]} />
        : n != null && <Text style={{ color: c.dim, fontSize: 10.5, fontFamily: fonts.mono }}>{n}</Text>}
    </Tappable>
  );
}

function Body({ sec }: { sec: Sec }) {
  if (sec === 'sound') return <MediaSounds />;
  if (sec === 'photo') return <MediaPhotos />;
  if (sec === 'clip') return <MediaClips />;
  if (sec === 'archive') return <MediaArchive />;
  return <MediaAudio />;
}

export default function Media() {
  const { c } = useTheme();
  const ip = useRobot((s) => s.ip);
  const { sec: secParam } = useLocalSearchParams<{ sec?: string }>();
  const valid = (v?: string): v is Sec => !!v && SNAV.some((s) => s.key === v);
  const [sec, setSec] = useState<Sec>(valid(secParam) ? secParam : 'photo');
  useEffect(() => { if (valid(secParam)) setSec(secParam); }, [secParam]); // eslint-disable-line react-hooks/exhaustive-deps
  useEffect(() => { for (const s of SNAV) if (s.key !== 'sound') void media.refresh(s.key); }, [ip]);
  const compact = useCompactH();
  const title = t(SNAV.find((s) => s.key === sec)!.label);
  const body = (
    <Animated.View key={sec} entering={FadeIn.duration(160)} style={{ flex: 1 }}>
      <ScrollView showsVerticalScrollIndicator={false} contentContainerStyle={{ paddingBottom: 12 }}>
        <Body sec={sec} />
      </ScrollView>
    </Animated.View>
  );
  if (compact) {
    return (
      <Screen>
        <HubHeader title={t('미디어')} subtitle={title} />
        <HTabs items={SNAV.map((s) => ({ key: s.key, label: t(s.label), icon: s.icon }))} value={sec} onChange={setSec} />
        <View style={[compactBodyStyle, { backgroundColor: c.panel, borderColor: c.line }]}>{body}</View>
      </Screen>
    );
  }
  return (
    <Screen>
      <HubHeader title={t('미디어')} subtitle={title} />
      <View style={styles.wrap}>
        <View style={[styles.snav, { backgroundColor: c.panel, borderColor: c.line }]}>
          {SNAV.map((s) => <NavItem key={s.key} k={s} on={s.key === sec} onPress={() => setSec(s.key)} />)}
        </View>
        <View style={[styles.sbody, { backgroundColor: c.panel, borderColor: c.line }]}>{body}</View>
      </View>
    </Screen>
  );
}

const styles = StyleSheet.create({
  wrap: { flex: 1, flexDirection: 'row', padding: 15, gap: 14 },
  snav: { width: 190, borderWidth: 1, borderRadius: 14, padding: 8, gap: 3 },
  snavItem: { flexDirection: 'row', alignItems: 'center', gap: 11, padding: 11, borderRadius: 9 },
  sbody: { flex: 1, borderWidth: 1, borderRadius: 14, paddingHorizontal: 22, paddingVertical: 18 },
  recDot: { width: 8, height: 8, borderRadius: 4 },
});
