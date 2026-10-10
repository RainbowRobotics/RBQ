import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export declare const matches: (value: string, query: string) => boolean;
export declare function RBCommand({ children, defaultSelected, style }: {
    children?: ReactNode;
    defaultSelected?: string | null;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function RBCommandInput({ placeholder, value, onChangeText }: {
    placeholder?: string;
    value?: string;
    onChangeText?: (v: string) => void;
}): import("react").JSX.Element;
export declare function RBCommandList({ children, style }: {
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function RBCommandEmpty({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element | null;
export declare function RBCommandGroup({ heading, children }: {
    heading?: ReactNode;
    children?: ReactNode;
}): import("react").JSX.Element | null;
export declare function RBCommandItem({ value, icon, children, shortcut, selectedMark, disabled, onSelect }: {
    value: string;
    icon?: ReactNode;
    children?: ReactNode;
    shortcut?: ReactNode;
    selectedMark?: ReactNode;
    disabled?: boolean;
    onSelect?: () => void;
}): import("react").JSX.Element | null;
export declare function RBCommandSeparator(): import("react").JSX.Element | null;
