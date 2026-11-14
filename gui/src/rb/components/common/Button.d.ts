import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type RippleOption } from './Ripple';
export type ButtonSize = 'large' | 'medium' | 'small' | 'tiny';
export type ButtonProps = {
    children: ReactNode;
    size?: ButtonSize;
    disabled?: boolean;
    loading?: boolean;
    fullsize?: boolean;
    ripple?: RippleOption;
    onPress?: () => void;
    style?: StyleProp<ViewStyle>;
    white?: boolean;
};
export declare function Button({ children, size, disabled, loading, fullsize, ripple, onPress, style, white }: ButtonProps): import("react").JSX.Element;
export declare function WhiteButton(p: Omit<ButtonProps, 'white'>): import("react").JSX.Element;
