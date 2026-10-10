import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export declare function ShadButton({ children, disabled, onPress, style }: {
    children: ReactNode;
    disabled?: boolean;
    onPress?: () => void;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
