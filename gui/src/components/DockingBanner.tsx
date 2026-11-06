import { useEffect, useState } from 'react';
import { View, StyleSheet, Text, Pressable } from 'react-native';
import { useTheme } from '@/theme';
import { Icon } from '@/components/Icon';
import { useRobot } from '@/store/robot';
import { useDockView } from '@/store/dockView';
import { t } from '@/lib/i18n';

const IN_PROGRESS = '#FBBA16';
const COMPLETE = '#00C853';

export function useDockingBannerShown() {
  const status = useRobot((s) => s.robot?.docking_status);
  const dockViewOpen = useDockView((s) => s.open);
  const [completeHidden, setCompleteHidden] = useState(false);

  const inProgress = status != null && status > 0 && status < 5;
  const complete = status != null && status >= 5 && status <= 7;

  useEffect(() => {
    if (!complete) { setCompleteHidden(false); return; }
    const id = setTimeout(() => setCompleteHidden(true), 4000);
    return () => clearTimeout(id);
  }, [complete]);

  return { show: !dockViewOpen && (inProgress || (complete && !completeHidden)), inProgress };
}

export function DockingBanner({ openable = false, inline }: { openable?: boolean; inline?: boolean } = {}) {
  const { radius } = useTheme();
  const { show, inProgress } = useDockingBannerShown();
  if (!show) return null;

  const tone = inProgress ? IN_PROGRESS : COMPLETE;
  const label = inProgress ? t('도킹 진행 중') : t('도킹 완료');
  const inner = (
    <>
      <Icon name="anchor" size={14} color={tone} />
      <Text style={{ color: tone, fontSize: 11.5, fontWeight: '700' }}>{label}</Text>
      {openable && <Text style={{ color: tone, fontSize: 10.5, opacity: 0.75 }}>{t('눌러서 보기')}</Text>}
    </>
  );
  return (
    <View pointerEvents="box-none" style={inline ? undefined : styles.bannerWrap}>
      {openable ? (
        <Pressable onPress={() => useDockView.getState().show()}
          style={({ pressed }) => [styles.banner, { borderColor: tone, borderRadius: radius.md },
            pressed && { backgroundColor: 'rgba(0,0,0,0.75)' }]}>
          {inner}
        </Pressable>
      ) : (
        <View style={[styles.banner, { borderColor: tone, borderRadius: radius.md }]}>{inner}</View>
      )}
    </View>
  );
}

const styles = StyleSheet.create({
  bannerWrap: { position: 'absolute', bottom: 70, left: 0, right: 0, alignItems: 'center', zIndex: 30 },
  banner: {
    flexDirection: 'row', alignItems: 'center', gap: 8,
    paddingHorizontal: 14, paddingVertical: 8, borderWidth: 1,
    backgroundColor: 'rgba(0,0,0,0.55)',
  },
});
