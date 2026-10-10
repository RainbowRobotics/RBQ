import { type StyleProp, type ViewStyle } from 'react-native';
import { type DateRange } from './Calendar';
export declare function DateInput({ value, onChange, disabled, style }: {
    value?: Date;
    onChange?: (d: Date) => void;
    disabled?: boolean;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function RangeDateInput({ value, onChange, disabled, style }: {
    value?: DateRange;
    onChange?: (r: DateRange) => void;
    disabled?: boolean;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
