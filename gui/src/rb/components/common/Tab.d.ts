import { type ReactNode } from 'react';
import { type StyleProp, type TextStyle, type ViewStyle } from 'react-native';
export type TabItem = {
    key: string;
    label: ReactNode;
    children?: ReactNode;
    disabled?: boolean;
    isHide?: boolean;
};
export type TabProps = {
    tabItems: TabItem[];
    activeKey?: string;
    onChange?: (key: string) => void;
    contentPaddingTop?: number;
    style?: StyleProp<ViewStyle>;
};
export declare const TabLabelCtx: import("react").Context<TextStyle>;
export declare const useTabLabelStyle: () => TextStyle;
export declare const Tab: (p: TabProps) => import("react").JSX.Element;
export declare const ChipTab: (p: TabProps) => import("react").JSX.Element;
export declare const InlineBoxTab: (p: TabProps) => import("react").JSX.Element;
