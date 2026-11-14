import { View, StyleSheet, type StyleProp, type ViewStyle } from 'react-native';
import type { IconName } from '@/components/Icon';
import { RailButton } from '@/components/control/RailButton';
import { railAnchor } from '@/components/control/ToolRail';
import { useDockButton } from '@/components/control/overlays';
import { t } from '@/lib/i18n';
import type { MotionName } from '@/types/robot';

type Fixed = 'sit' | 'stand' | 'walk';
const FIXED: { key: Fixed; icon: IconName; label: string }[] = [
  { key: 'sit', icon: 'sit', label: 'SIT' },
  { key: 'stand', icon: 'stand', label: 'STAND' },
  { key: 'walk', icon: 'walk', label: 'WALK' },
];

function MBtn({ icon, label, active, dashed, onPress, style }: {
  icon: IconName; label: string; active?: boolean; dashed?: boolean; onPress?: () => void;
  style?: StyleProp<ViewStyle>;
}) {
  return (
    <View style={style}>
      <RailButton icon={icon} label={label} active={active} dashed={dashed}
        onPress={onPress} style={StyleSheet.absoluteFill} />
    </View>
  );
}

export function MotionButtons({ active, onSelect, onMore, extra = [] }: {
  active?: MotionName | null; onSelect?: (m: MotionName) => void; onMore?: () => void;
  extra?: { key: MotionName; icon: IconName; label: string }[];
}) {
  return (
    <>
      {[...FIXED, ...extra].map((b) => (
        <MBtn key={b.key} icon={b.icon} label={b.label} active={active === b.key}
          onPress={() => onSelect?.(b.key)} style={styles.colBtn} />
      ))}
      <MBtn icon="plus" label={t('명령')} dashed onPress={onMore} style={styles.colBtn} />
    </>
  );
}

export function usePadExtra(): { key: MotionName; icon: IconName; label: string }[] {
  const dock = useDockButton();
  return [
    { key: 'stairs' as MotionName, icon: 'stairs' as IconName, label: 'STAIRS' },
    ...(dock ? [{ key: 'dock' as MotionName, icon: 'dock' as IconName, label: 'DOCKING' }] : []),
  ];
}

const styles = StyleSheet.create({
  colBtn: { height: 50, alignSelf: 'stretch' },
});
