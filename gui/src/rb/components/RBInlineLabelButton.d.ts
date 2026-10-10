import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { inlinelabelButtonSizes } from '../vendor/inline-label-button.variants';
export type InlineLabelButtonSize = (typeof inlinelabelButtonSizes)[number];
export type InlineLabelButtonVariant = 'default' | 'brand';
export type RBInlineLabelButtonProps = {
    children: ReactNode;
    size?: InlineLabelButtonSize;
    variant?: InlineLabelButtonVariant;
    disabled?: boolean;
    leadingIcon?: (p: {
        size: number;
        color: string;
    }) => ReactNode;
    trailingIcon?: (p: {
        size: number;
        color: string;
    }) => ReactNode;
    onPress?: () => void;
    style?: StyleProp<ViewStyle>;
};
export declare function RBInlineLabelButton({ children, size, variant, disabled, leadingIcon, trailingIcon, onPress, style }: RBInlineLabelButtonProps): import("react").JSX.Element;
