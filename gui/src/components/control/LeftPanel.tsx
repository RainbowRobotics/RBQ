import { useWindowDimensions } from 'react-native';
import type { IconName } from '@/components/Icon';
import { MorphCard } from '@/components/control/MorphCard';
import { t } from '@/lib/i18n';

export const LEFT_X = 4;
export const LEFT_W = 92;
export const LEFT_TOP = 12;
export const LEFT_HEAD = 34;

export type ToolItem = {
  key: string;
  icon: IconName;
  label: string;
  open?: boolean;
  dot?: boolean;
  onPress: () => void;
};

export function LeftPanel({ bottomReserve = 12, inset = 0, overVideo = true, topInset = 0, width = LEFT_W, children }: {
  bottomReserve?: number;
  inset?: number;
  overVideo?: boolean;
  topInset?: number;
  width?: number;
  children: React.ReactNode;
}) {
  const { height: winH } = useWindowDimensions();
  const maxH = Math.max(140, winH - topInset - LEFT_TOP - bottomReserve);
  return (
    <MorphCard side="left" x={LEFT_X + inset} top={LEFT_TOP + topInset} width={width} maxHeight={maxH}
      icon="person" label={t('모션')} overVideo={overVideo}>
      {children}
    </MorphCard>
  );
}
