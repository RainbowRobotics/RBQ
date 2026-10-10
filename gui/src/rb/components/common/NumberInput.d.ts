import { type StyleProp, type ViewStyle } from 'react-native';
import { type RippleOption } from './Ripple';
export type NumberInputProps = {
    value?: string;
    defaultValue?: string;
    step?: number;
    min?: number;
    max?: number;
    placeholder?: string;
    disabled?: boolean;
    error?: boolean;
    rippleOption?: RippleOption;
    onChange?: (v: string, opt?: {
        type: 'up' | 'down';
    }) => void;
    style?: StyleProp<ViewStyle>;
};
export declare function useRepeat(fire: (type: 'up' | 'down') => void, firstDelay: number): {
    down: (type: "up" | "down") => void;
    stop: () => void;
};
export declare function NumberInput({ value, defaultValue, step, min, max, placeholder, disabled, error, rippleOption, onChange, style }: NumberInputProps): import("react").JSX.Element;
