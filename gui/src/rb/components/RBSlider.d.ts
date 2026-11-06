import { type StyleProp, type ViewStyle } from 'react-native';
export type RBSliderProps = {
    value?: number;
    defaultValue?: number;
    min?: number;
    max?: number;
    step?: number;
    disabled?: boolean;
    onValueChange?: (v: number) => void;
    onValueCommit?: (v: number) => void;
    style?: StyleProp<ViewStyle>;
};
export declare function RBSlider({ value, defaultValue, min, max, step, disabled, onValueChange, onValueCommit, style }: RBSliderProps): import("react").JSX.Element;
