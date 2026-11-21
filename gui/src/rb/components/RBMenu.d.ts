import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export declare function RBMenuContent({ children, width, side, style }: {
    children?: ReactNode;
    width?: number;
    side?: 'bottom' | 'right';
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function RBMenuLabel({ children }: {
    children?: ReactNode;
}): import("react").JSX.Element;
export declare function RBMenuSeparator(): import("react").JSX.Element;
export declare function RBMenuShortcut({ children, tone }: {
    children?: ReactNode;
    tone?: 'default' | 'destructive' | 'disabled';
}): import("react").JSX.Element;
type ItemProps = {
    children?: ReactNode;
    icon?: ReactNode;
    shortcut?: ReactNode;
    variant?: 'default' | 'destructive';
    disabled?: boolean;
    onSelect?: () => void;
    highlighted?: boolean;
    trailing?: ReactNode;
    cls?: string;
};
export declare function RBMenuItem({ children, icon, shortcut, variant, disabled, onSelect, highlighted, trailing, cls }: ItemProps): import("react").JSX.Element;
export declare function RBMenuCheckboxItem({ checked, onCheckedChange, ...p }: Omit<ItemProps, 'trailing' | 'cls'> & {
    checked?: boolean;
    onCheckedChange?: (v: boolean) => void;
}): import("react").JSX.Element;
export declare function RBMenuSub({ label, disabled, children, width }: {
    label: ReactNode;
    disabled?: boolean;
    children?: ReactNode;
    width?: number;
}): import("react").JSX.Element;
export declare function RBDropdownMenu({ trigger, defaultOpen, width, children }: {
    trigger: (toggle: () => void, open: boolean) => ReactNode;
    defaultOpen?: boolean;
    width?: number;
    children?: ReactNode;
}): import("react").JSX.Element;
export declare function RBMenubar({ menus, style }: {
    menus: {
        label: ReactNode;
        content: ReactNode;
    }[];
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export {};
