import { type StyleProp, type ViewStyle } from 'react-native';
import { type RippleOption } from './Ripple';
export type SideNumberInputProps = {
    value?: string;
    handleChange?: (v: string, name?: string, opt?: {
        type: 'up' | 'down';
    }) => void;
    step?: number;
    min?: number;
    max?: number;
    disabled?: boolean;
    placeholder?: string;
    rippleOption?: RippleOption;
    style?: StyleProp<ViewStyle>;
};
export declare function SideNumberInput({ value, handleChange, step, min, max, disabled, placeholder, rippleOption, style }: SideNumberInputProps): import("react").JSX.Element;
