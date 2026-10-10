import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type DrawerSide = 'top' | 'bottom' | 'left' | 'right';
export declare function RBDrawer({ trigger, defaultOpen, side, showHandle, showCloseButton, children, style }: {
    trigger?: (open: () => void) => ReactNode;
    defaultOpen?: boolean;
    side?: DrawerSide;
    showHandle?: boolean;
    showCloseButton?: boolean;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function RBDrawerHeader({ children, center }: {
    children?: ReactNode;
    center?: boolean;
}): import("react").JSX.Element;
export declare function RBDrawerTitle({ children, center }: {
    children?: ReactNode;
    center?: boolean;
}): import("react").JSX.Element;
export declare function RBDrawerDescription({ children, center }: {
    children?: ReactNode;
    center?: boolean;
}): import("react").JSX.Element;
export declare function RBDrawerBody({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element;
export declare function RBDrawerFooter({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element;
