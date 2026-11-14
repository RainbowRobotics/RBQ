import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type ResizableDirection = 'horizontal' | 'vertical';
export declare function RBResizable({ direction, defaultRatio, minRatio, hideThumb, first, second, style }: {
    direction?: ResizableDirection;
    defaultRatio?: number;
    minRatio?: number;
    hideThumb?: boolean;
    first: ReactNode;
    second: ReactNode;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
