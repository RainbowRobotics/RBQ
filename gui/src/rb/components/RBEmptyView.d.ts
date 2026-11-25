import type { ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type EmptyViewType } from '../vendor/empty-view.variants';
export type RBEmptyViewProps = {
    type?: EmptyViewType;
    message?: ReactNode;
    description?: ReactNode;
    buttonLabel?: ReactNode;
    onButtonPress?: () => void;
    icon?: (p: {
        size: number;
        color: string;
    }) => ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function RBEmptyView({ type, message, description, buttonLabel, onButtonPress, icon, style }: RBEmptyViewProps): import("react").JSX.Element;
