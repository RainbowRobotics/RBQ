import { type StyleProp, type ViewStyle } from 'react-native';
export type DateInputSize = 'xs2' | 'sm' | 'md' | 'lg';
export type RBDateInputProps = {
    size?: DateInputSize;
    valueLabel?: string;
    placeholder?: string;
    width?: number;
    onPress?: () => void;
    style?: StyleProp<ViewStyle>;
    mode?: 'single' | 'range';
    onChange?: (from?: Date, to?: Date) => void;
    disabled?: boolean;
    readOnly?: boolean;
    error?: boolean;
};
export declare function RBDateInput({ size, valueLabel, placeholder, width, onPress, style, mode, onChange, disabled, readOnly, error }: RBDateInputProps): import("react").JSX.Element;
export declare const RBRangeDateInput: (p: RBDateInputProps) => import("react").JSX.Element;
