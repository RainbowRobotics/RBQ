import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { iconButtonSizes } from '../vendor/icon-button.variants';
export { iconButtonSizes };
export type IconButtonSize = (typeof iconButtonSizes)[number];
export declare const iconButtonVariantNames: readonly ["transparent", "outlined", "soft", "solid", "inverse", "inverseSoft", "brand", "circle"];
export type IconButtonVariant = (typeof iconButtonVariantNames)[number];
export type RBIconButtonProps = {
    icon: (p: {
        size: number;
        color: string;
    }) => ReactNode;
    size?: IconButtonSize;
    variant?: IconButtonVariant;
    disabled?: boolean;
    loading?: boolean;
    onPress?: () => void;
    style?: StyleProp<ViewStyle>;
    slotName?: string;
};
export declare function RBIconButton({ icon, size, variant, disabled, loading, onPress, style, slotName }: RBIconButtonProps): import("react").JSX.Element;
