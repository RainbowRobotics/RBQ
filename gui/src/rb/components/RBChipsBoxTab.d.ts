import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type ChipBoxSize } from '../vendor/tab.variants';
export type ChipsBoxItem = {
    key: string | number;
    label: ReactNode;
    disabled?: boolean;
};
export type ChipsBoxSize = ChipBoxSize;
export type RBChipsBoxTabProps = {
    tabItems: ChipsBoxItem[];
    size?: ChipsBoxSize;
    activeKey?: string | number;
    defaultActiveKey?: string | number;
    onChange?: (k: string | number) => void;
    disabled?: boolean;
    enableOverflowExpand?: boolean;
    showBottomBorder?: boolean;
    style?: StyleProp<ViewStyle>;
};
export declare function RBChipsBoxTab({ tabItems, size, activeKey, defaultActiveKey, onChange, disabled, enableOverflowExpand, showBottomBorder, style }: RBChipsBoxTabProps): import("react").JSX.Element;
