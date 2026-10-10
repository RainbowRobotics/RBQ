import { type StyleProp, type TextStyle, type ViewStyle, type TextInputProps } from 'react-native';
export type TextInputSize = 'xs2' | 'sm' | 'md' | 'lg';
export type TextInputVariant = 'default' | 'success';
type Common = {
    value?: string;
    defaultValue?: string;
    placeholder?: string;
    variant?: TextInputVariant;
    error?: boolean;
    disabled?: boolean;
    readOnly?: boolean;
    secureTextEntry?: boolean;
    autoFocus?: boolean;
    onChangeText?: (v: string) => void;
    onEnter?: (v: string) => void;
    onReset?: (v: string) => void;
    hideBtnReset?: boolean;
    style?: StyleProp<ViewStyle>;
    inputStyle?: StyleProp<TextStyle>;
    keyboardType?: TextInputProps['keyboardType'];
    autoCapitalize?: TextInputProps['autoCapitalize'];
};
export declare function RBTextInput({ size, ...p }: Common & {
    size?: TextInputSize;
}): import("react").JSX.Element;
export type TextAreaSize = 'sm' | 'md' | 'lg';
export declare function RBTextArea({ size, ...p }: Common & {
    size?: TextAreaSize;
}): import("react").JSX.Element;
export {};
