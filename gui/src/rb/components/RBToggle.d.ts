import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type ToggleSize = 'default' | 'sm' | 'lg';
export type RBToggleProps = {
    pressed?: boolean;
    defaultPressed?: boolean;
    disabled?: boolean;
    size?: ToggleSize;
    variant?: 'default' | 'outline';
    icon?: (p: {
        size: number;
        color: string;
    }) => ReactNode;
    children?: ReactNode;
    onPressedChange?: (v: boolean) => void;
    style?: StyleProp<ViewStyle>;
};
export declare const nativeFixedH: (v: Record<string, unknown>) => {
    paddingTop: number;
    paddingBottom: number;
} | null;
export declare function RBToggle({ pressed, defaultPressed, disabled, size, variant, icon, children, onPressedChange, style }: RBToggleProps): import("react").JSX.Element;
