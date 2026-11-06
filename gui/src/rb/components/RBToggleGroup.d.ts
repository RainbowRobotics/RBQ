import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type ToggleGroupItem = {
    value: string;
    icon?: (p: {
        size: number;
        color: string;
    }) => ReactNode;
    label?: ReactNode;
    disabled?: boolean;
    ariaLabel?: string;
};
export type RBToggleGroupProps = {
    items: ToggleGroupItem[];
    value?: string;
    defaultValue?: string;
    onValueChange?: (v: string) => void;
    orientation?: 'horizontal' | 'vertical';
    separated?: boolean;
    stretched?: boolean;
    style?: StyleProp<ViewStyle>;
};
export declare function RBToggleGroup({ items, value, defaultValue, onValueChange, orientation, separated, stretched, style }: RBToggleGroupProps): import("react").JSX.Element;
