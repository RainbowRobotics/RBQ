import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type TabsSize, type TabsVariant } from '../vendor/tabs.variants';
export type TabsItem = {
    value: string;
    label: ReactNode;
    icon?: ReactNode;
    badge?: ReactNode;
    disabled?: boolean;
    content?: ReactNode;
};
export declare function RBTabs({ items, defaultValue, value, onChange, size, variant, orientation, style }: {
    items: TabsItem[];
    defaultValue?: string;
    value?: string;
    onChange?: (v: string) => void;
    size?: TabsSize;
    variant?: TabsVariant;
    orientation?: 'horizontal' | 'vertical';
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
