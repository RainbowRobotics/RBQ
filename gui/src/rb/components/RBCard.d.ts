import type { ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type CardSize } from '../vendor/card.variants';
export type { CardSize };
export declare function RBCard({ size, children, style }: {
    size?: CardSize;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function RBCardHeader({ children, action, size }: {
    children?: ReactNode;
    action?: ReactNode;
    size?: CardSize;
}): import("react").JSX.Element;
export declare function RBCardTitle({ children, size }: {
    children?: ReactNode;
    size?: CardSize;
}): import("react").JSX.Element;
export declare function RBCardDescription({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element;
export declare function RBCardContent({ children, size }: {
    children?: ReactNode;
    size?: CardSize;
}): import("react").JSX.Element;
export declare function RBCardFooter({ children, size }: {
    children?: ReactNode;
    size?: CardSize;
}): import("react").JSX.Element;
