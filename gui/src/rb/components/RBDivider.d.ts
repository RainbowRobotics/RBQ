import type { ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type DividerThickness } from '../vendor/divider.variants';
export type RBDividerProps = {
    type?: 'horizontal' | 'vertical';
    variant?: 'solid' | 'dashed' | 'dotted' | 'soft';
    color?: string;
    size?: number;
    thickness?: DividerThickness;
    spaceSize?: number;
    spaceDirection?: 'top' | 'bottom' | 'left' | 'right' | 'both';
    contentSpace?: number;
    contentAlign?: 'left' | 'right' | 'center';
    contentPosition?: number;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function RBDivider({ type, variant, color, size, thickness, spaceSize, spaceDirection, contentSpace, contentAlign, contentPosition, children, style }: RBDividerProps): import("react").JSX.Element;
