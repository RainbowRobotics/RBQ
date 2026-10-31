import type { ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type RadioSize } from '../vendor/radio.variants';
export type RadioBoxSize = RadioSize;
export type RBRadioBoxProps = {
    size?: RadioBoxSize;
    checked?: boolean;
    disabled?: boolean;
    readOnly?: boolean;
    onPress?: () => void;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function RBRadioBox({ size, checked, disabled, readOnly, onPress, children, style }: RBRadioBoxProps): import("react").JSX.Element;
