import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type LabelButtonVariant } from './RBLabelButton';
export type DialogSize = 'sm' | 'md' | 'lg';
export type RBDialogProps = {
    open: boolean;
    title?: ReactNode;
    description?: ReactNode;
    footer?: ReactNode;
    children?: ReactNode;
    size?: DialogSize;
    width?: number;
    minWidth?: number;
    style?: StyleProp<ViewStyle>;
    onOverlayPress?: () => void;
    animated?: boolean;
    state?: 'open' | 'closed';
    overlayRef?: (n: unknown) => void;
    contentRef?: (n: unknown) => void;
    onEscape?: () => void;
};
export declare function RBDialog({ open, title, description, footer, children, size, width, minWidth, style, onOverlayPress, animated, onEscape, state, overlayRef, contentRef }: RBDialogProps): import("react").JSX.Element | null;
export declare function DialogBodyText({ children }: {
    children: ReactNode;
}): import("react").JSX.Element;
type BaseProps = {
    open: boolean;
    title?: ReactNode;
    description?: ReactNode;
    content?: ReactNode;
    onOpenChange?: (open: boolean) => void;
};
export declare function RBAlert({ open, title, description, content, buttonText, onConfirm, onOpenChange }: BaseProps & {
    buttonText?: ReactNode;
    onConfirm?: () => void;
}): import("react").JSX.Element;
export declare function RBConfirm({ open, title, description, content, buttonText, yesVariant, onConfirm, onCancel, onOpenChange }: BaseProps & {
    buttonText?: {
        yes: ReactNode;
        no: ReactNode;
    };
    yesVariant?: LabelButtonVariant;
    onConfirm?: () => void;
    onCancel?: () => void;
}): import("react").JSX.Element;
export declare function RBPrompt({ open, title, content, placeholder, value, buttonText, onResolve, onOpenChange }: BaseProps & {
    placeholder?: string;
    value?: string;
    buttonText?: {
        yes: ReactNode;
        no: ReactNode;
    };
    onResolve?: (v: string | null) => void;
}): import("react").JSX.Element;
export {};
