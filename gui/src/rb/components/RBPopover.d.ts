import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type RBPopoverPlacement } from '../vendor/popover.variants';
export type PopoverPlace = RBPopoverPlacement;
export declare function RBPopover({ trigger, triggerStyle, place, arrow, children, style }: {
    trigger: (toggle: () => void, open: boolean) => ReactNode;
    triggerStyle?: StyleProp<ViewStyle>;
    place?: PopoverPlace;
    arrow?: boolean;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
