import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type SheetSide = 'top' | 'bottom' | 'left' | 'right';
export declare function pxAnim(ds: {
    anim: string;
} | undefined, size: {
    w: number;
    h: number;
} | null): {
    anim: string;
} | undefined;
export declare function holdExit(ds: {
    anim: string;
} | undefined, closed: boolean): {
    anim: string;
} | undefined;
export declare function RBSheet({ trigger, defaultOpen, side, showCloseButton, children, style }: {
    trigger?: (open: () => void) => ReactNode;
    defaultOpen?: boolean;
    side?: SheetSide;
    showCloseButton?: boolean;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function RBSheetHeader({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element;
export declare function RBSheetTitle({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element;
export declare function RBSheetDescription({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element;
export declare function RBSheetFooter({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element;
