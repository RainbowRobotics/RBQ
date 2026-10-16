import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type ChipSize, type TabSize } from '../vendor/tab.variants';
export type TabItem = {
    key: string | number;
    label: ReactNode;
    children?: ReactNode;
    disabled?: boolean;
};
export type { TabSize };
type Common = {
    tabItems: TabItem[];
    activeKey?: string | number;
    defaultActiveKey?: string | number;
    onChange?: (key: string | number) => void;
    disabled?: boolean;
    style?: StyleProp<ViewStyle>;
};
export declare function RBTab({ size, slider, ...p }: Common & {
    size?: TabSize;
    slider?: boolean;
}): import("react").JSX.Element;
export declare function RBChipTab({ size, ...p }: Common & {
    size?: ChipSize;
}): import("react").JSX.Element;
