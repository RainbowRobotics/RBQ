import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
type Common = {
    checked?: boolean;
    defaultChecked?: boolean;
    disabled?: boolean;
    readOnly?: boolean;
    onChange?: (checked: boolean) => void;
    style?: StyleProp<ViewStyle>;
};
export type CheckIconButtonSize = 'xs' | 'xs2' | 'sm' | 'md';
export declare function RBCheckIconButton({ icon, size, ...p }: Common & {
    size?: CheckIconButtonSize;
    icon: (a: {
        size: number;
        color: string;
    }) => ReactNode;
}): import("react").JSX.Element;
export type CheckLabelButtonSize = 'xs' | 'xs2' | 'sm' | 'md';
export declare function RBCheckLabelButton({ children, size, icon, ...p }: Common & {
    children: ReactNode;
    size?: CheckLabelButtonSize;
    icon?: (a: {
        size: number;
        color: string;
    }) => ReactNode;
}): import("react").JSX.Element;
export {};
