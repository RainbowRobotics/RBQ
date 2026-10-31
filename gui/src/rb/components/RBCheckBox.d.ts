import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type CheckBoxSize } from '../vendor/checkbox.variants';
export type { CheckBoxSize };
export type RBCheckBoxProps = {
    size?: CheckBoxSize;
    checked?: boolean;
    partial?: boolean;
    defaultChecked?: boolean;
    disabled?: boolean;
    readOnly?: boolean;
    onChange?: (checked: boolean) => void;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function RBCheckBox({ size, checked, partial, defaultChecked, disabled, readOnly, onChange, children, style }: RBCheckBoxProps): import("react").JSX.Element;
