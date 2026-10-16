import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type RadioProps<T> = {
    children?: ReactNode;
    checkValue: T;
    value?: T;
    onChange?: (v: T) => void;
    disabled?: boolean;
    style?: StyleProp<ViewStyle>;
};
export declare function RadioBox<T>({ children, checkValue, value, onChange, disabled, style }: RadioProps<T>): import("react").JSX.Element;
export declare function RadioButton<T>({ children, checkValue, value, onChange, disabled, style }: RadioProps<T>): import("react").JSX.Element;
