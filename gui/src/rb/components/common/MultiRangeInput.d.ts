import { type StyleProp, type ViewStyle } from 'react-native';
export type MultiRangeInputProps = {
    min?: number;
    max?: number;
    step?: number;
    value?: [number, number];
    defaultValue?: [number, number];
    disabled?: boolean;
    thumbSize?: number;
    sliderHeight?: number;
    onChange?: (v: [number, number]) => void;
    handleChange?: (v: [number, number]) => void;
    style?: StyleProp<ViewStyle>;
};
export declare function MultiRangeInput({ min, max, step, value, defaultValue, disabled: disabledProp, thumbSize, sliderHeight, onChange, handleChange, style }: MultiRangeInputProps): import("react").JSX.Element;
