import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type AccordionProps = {
    title: ReactNode;
    open?: boolean;
    defaultOpen?: boolean;
    disabled?: boolean;
    onOpenChange?: (open: boolean) => void;
    borderColor?: string;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
    noBottom?: boolean;
};
export declare function Accordion({ title, open, defaultOpen, disabled, onOpenChange, borderColor, children, style, noBottom }: AccordionProps): import("react").JSX.Element;
export declare function AccordionGroup({ list, style }: {
    list: Array<Omit<AccordionProps, 'noBottom'>>;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
