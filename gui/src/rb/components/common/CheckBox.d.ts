import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
type Variant = 'default' | 'single' | 'round';
export type CheckBoxProps = {
    children?: ReactNode;
    checked?: boolean;
    defaultChecked?: boolean;
    disabled?: boolean;
    onChange?: (checked: boolean) => void;
    style?: StyleProp<ViewStyle>;
    variant?: Variant;
};
export declare function CheckBox({ children, disabled, style, variant, ...p }: CheckBoxProps): import("react").JSX.Element;
export declare function Check(p: Omit<CheckBoxProps, 'variant'>): import("react").JSX.Element;
export declare function RoundCheckBox(p: Omit<CheckBoxProps, 'variant'>): import("react").JSX.Element;
export declare function CheckButton({ children, disabled, style, ...p }: Omit<CheckBoxProps, 'variant'>): import("react").JSX.Element;
export {};
