import { type StyleProp, type ViewStyle } from 'react-native';
export type RBMultiRangeInputProps = {
    min?: number;
    max?: number;
    step?: number;
    value?: [number, number];
    defaultValue?: [number, number];
    disabled?: boolean;
    showTextInput?: boolean;
    thumbSize?: number;
    sliderHeight?: number;
    onChange?: (v: [number, number]) => void;
    style?: StyleProp<ViewStyle>;
};
export declare function RBMultiRangeInput({ min, max, step, value, defaultValue, disabled, showTextInput, thumbSize, sliderHeight, onChange, style }: RBMultiRangeInputProps): import("react").JSX.Element;
