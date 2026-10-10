import type { ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type DividerProps = {
    type?: 'horizontal' | 'vertical';
    variant?: 'solid' | 'dashed' | 'dotted';
    color?: string;
    size?: number;
    spaceSize?: number;
    spaceDirection?: 'top' | 'bottom' | 'left' | 'right' | 'both';
    contentSpace?: number;
    contentAlign?: 'left' | 'right' | 'center';
    contentPosition?: number;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function Divider({ type, variant, color, size, spaceSize, spaceDirection, contentSpace, contentAlign, contentPosition, children, style }: DividerProps): import("react").JSX.Element;
