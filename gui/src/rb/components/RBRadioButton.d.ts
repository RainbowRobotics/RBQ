import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type RadioSize } from '../vendor/radio.variants';
export type RadioButtonSize = RadioSize;
export type RBRadioButtonProps = {
    children: ReactNode;
    size?: RadioButtonSize;
    checked?: boolean;
    disabled?: boolean;
    readOnly?: boolean;
    onPress?: () => void;
    style?: StyleProp<ViewStyle>;
};
export declare function RBRadioButton({ children, size, checked, disabled, readOnly, onPress, style }: RBRadioButtonProps): import("react").JSX.Element;
