import { useWindowDimensions } from 'react-native';
import { MorphCard } from '@/components/control/MorphCard';
import type { IconName } from '@/components/Icon';
import { RailButton } from '@/components/control/RailButton';
import { t } from '@/lib/i18n';

export const RIGHT_X = 4;
export const RIGHT_W = 92;
export const RIGHT_TOP = 12;

export type ViewItem = {
  key: string;
  icon: IconName;
  label: string;
  open?: boolean;
  onPress: () => void;
};

export function ViewRow({ label, icon, on, dot, onPress }: {
  label: string; icon: IconName; on?: boolean;
  dot?: boolean;
  onPress: () => void;
}) {
  return <RailButton icon={icon} label={label} active={on} dot={dot} onPress={onPress} />;
}

export function RightPanel({ children, bottomReserve = 12, inset = 0, overVideo = true, topInset = 0, width = RIGHT_W, title = '관측' }: {
  children: React.ReactNode;
  bottomReserve?: number;
  inset?: number;
  title?: string;
  overVideo?: boolean;
  topInset?: number;
  width?: number;
}) {
  const { height: winH } = useWindowDimensions();
  const maxH = Math.max(140, winH - topInset - RIGHT_TOP - bottomReserve);
  return (
    <MorphCard side="right" x={RIGHT_X + inset} top={RIGHT_TOP + topInset} width={width} maxHeight={maxH}
      icon="apps" label={t(title)} overVideo={overVideo}>
      {children}
    </MorphCard>
  );
}
