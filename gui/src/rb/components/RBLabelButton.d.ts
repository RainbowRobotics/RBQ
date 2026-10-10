import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { labelButtonSizes } from '../vendor/label-button.variants';
export { labelButtonSizes };
export type LabelButtonSize = (typeof labelButtonSizes)[number];
export declare const labelButtonVariantNames: readonly ["transparent", "outlined", "soft", "solid", "inverse", "inverseSoft", "brand", "brandSoft", "danger", "dangerSoft", "success", "successSoft"];
export type LabelButtonVariant = (typeof labelButtonVariantNames)[number];
export type RBLabelButtonProps = {
    children: ReactNode;
    size?: LabelButtonSize;
    variant?: LabelButtonVariant;
    disabled?: boolean;
    loading?: boolean;
    leadingIcon?: ReactNode;
    trailingIcon?: ReactNode;
    onPress?: () => void;
    style?: StyleProp<ViewStyle>;
};
export declare function RBLabelButton({ children, size, variant, disabled, loading, leadingIcon, trailingIcon, onPress, style }: RBLabelButtonProps): import("react").JSX.Element;
