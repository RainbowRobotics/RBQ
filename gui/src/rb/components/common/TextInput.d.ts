import { type StyleProp, type TextStyle, type ViewStyle } from 'react-native';
type Common = {
    value?: string;
    defaultValue?: string;
    placeholder?: string;
    disabled?: boolean;
    readOnly?: boolean;
    error?: boolean;
    onChangeText?: (v: string) => void;
    onSubmit?: (v: string) => void;
    style?: StyleProp<ViewStyle>;
    inputStyle?: StyleProp<TextStyle>;
};
export declare function TextInput({ width, hideBtnReset, ...p }: Common & {
    width?: number;
    hideBtnReset?: boolean;
}): import("react").JSX.Element;
export declare function TextArea({ rows, ...p }: Common & {
    rows?: number;
}): import("react").JSX.Element;
export {};
