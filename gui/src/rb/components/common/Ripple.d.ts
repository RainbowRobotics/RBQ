import { type ReactNode } from 'react';
import { type PressableProps, type StyleProp, type ViewStyle } from 'react-native';
export type RippleOption = {
    color?: string;
    duration?: number;
    maxSize?: number;
    disabled?: boolean;
};
export type RippleProps = RippleOption & Omit<PressableProps, 'style' | 'disabled'> & {
    children: ReactNode;
    style?: StyleProp<ViewStyle>;
    pressDisabled?: boolean;
};
export declare function Ripple({ children, color, duration, maxSize, disabled, pressDisabled, onPressIn, onHoverIn, onHoverOut, style, ...rest }: RippleProps): import("react").JSX.Element;
