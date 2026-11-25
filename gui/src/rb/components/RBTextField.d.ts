import type { ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type HelperTone = 'default' | 'error' | 'success';
export type RBTextFieldProps = {
    label?: ReactNode;
    description?: ReactNode;
    required?: boolean;
    helperText?: ReactNode;
    helperTone?: HelperTone;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function RBTextField({ label, description, required, helperText, helperTone, children, style }: RBTextFieldProps): import("react").JSX.Element | null;
