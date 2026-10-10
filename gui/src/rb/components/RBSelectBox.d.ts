import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type SelectSize = 'xs2' | 'sm' | 'md' | 'lg';
export type SelectOption = {
    value: string;
    label: string;
    info?: string;
    disabled?: boolean;
};
export type RBSelectBoxProps = {
    size?: SelectSize;
    placeholder?: string;
    valueLabel?: string;
    error?: boolean;
    disabled?: boolean;
    readOnly?: boolean;
    open?: boolean;
    onPress?: () => void;
    width?: number;
    valueFontWeight?: '400' | '500';
    style?: StyleProp<ViewStyle>;
    arrow?: (p: {
        size: number;
        color: string;
    }) => ReactNode;
    options?: SelectOption[];
    value?: string;
    onChange?: (value: string) => void;
    enableModal?: boolean;
};
export declare function RBSelectBox({ size, placeholder, valueLabel, error, disabled, readOnly, open, onPress, width, valueFontWeight, style, arrow, options, value, onChange, enableModal }: RBSelectBoxProps): import("react").JSX.Element;
