import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type SelectOption<T> = {
    value: T;
    label: ReactNode;
    disabled?: boolean;
};
export type SelectBoxProps<T> = {
    options?: SelectOption<T>[];
    value?: T;
    placeholder?: string;
    disabled?: boolean;
    readOnly?: boolean;
    noDataMsg?: ReactNode;
    onChange?: (v: T) => void;
    style?: StyleProp<ViewStyle>;
};
export declare function SelectBox<T>({ options, value, placeholder, disabled, readOnly, noDataMsg, onChange, style }: SelectBoxProps<T>): import("react").JSX.Element;
