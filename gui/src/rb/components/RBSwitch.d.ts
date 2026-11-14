import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type SwitchSize = 'default' | 'sm';
export type RBSwitchProps = {
    checked?: boolean;
    defaultChecked?: boolean;
    disabled?: boolean;
    invalid?: boolean;
    size?: SwitchSize;
    onCheckedChange?: (v: boolean) => void;
    style?: StyleProp<ViewStyle>;
};
export declare function RBSwitch({ checked, defaultChecked, disabled, invalid, size, onCheckedChange, style }: RBSwitchProps): import("react").JSX.Element;
export type RBSwitchFieldProps = RBSwitchProps & {
    label?: ReactNode;
    description?: ReactNode;
    controlPlacement?: 'start' | 'end';
    type?: 'default' | 'box';
    fieldStyle?: StyleProp<ViewStyle>;
};
export declare function RBSwitchField({ label, description, controlPlacement, type, fieldStyle, ...sw }: RBSwitchFieldProps): import("react").JSX.Element;
