import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export declare function NxButton({ children, onPress, disabled, style }: {
    children: ReactNode;
    onPress?: () => void;
    disabled?: boolean;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function NxNumberInput({ value, step, disabled, onChange, style }: {
    value?: number;
    step?: number;
    disabled?: boolean;
    onChange?: (v: number) => void;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function NxSelectBox({ value, options, onChange, style }: {
    value?: string;
    options: {
        value: string;
        label: string;
        disabled?: boolean;
    }[];
    onChange?: (v: string) => void;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function NxTextInput({ value, onChangeText, placeholder, style }: {
    value?: string;
    onChangeText?: (v: string) => void;
    placeholder?: string;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function NxTextArea({ value, onChangeText, placeholder, style }: {
    value?: string;
    onChangeText?: (v: string) => void;
    placeholder?: string;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function NxSwitch({ checked, disabled, onCheckedChange, style }: {
    checked?: boolean;
    disabled?: boolean;
    onCheckedChange?: (v: boolean) => void;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function NxPagination({ currPage, totalPage, onPageChange, style }: {
    currPage?: number;
    totalPage?: number;
    onPageChange?: (p: number) => void;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
