import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type BadgeColor, type BadgeNumberVariant, type BadgeType } from '../vendor/badge.variants';
export type { BadgeType, BadgeNumberVariant, BadgeColor };
export type RBBadgeProps = {
    type?: BadgeType;
    numberVariant?: BadgeNumberVariant;
    color?: BadgeColor;
    leadingIcon?: (p: {
        size: number;
        color: string;
    }) => ReactNode;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function RBBadge({ type, numberVariant, color, leadingIcon, children, style }: RBBadgeProps): import("react").JSX.Element;
