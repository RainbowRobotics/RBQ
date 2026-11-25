import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
import { type AccordionType } from '../vendor/accordion.variants';
export type { AccordionType };
export type RBAccordionProps = {
    type?: AccordionType;
    title: ReactNode;
    open?: boolean;
    initialOpen?: boolean;
    disabled?: boolean;
    onOpenChange?: (open: boolean) => void;
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function RBAccordion({ type, title, open, initialOpen, disabled, onOpenChange, children, style }: RBAccordionProps): import("react").JSX.Element;
export declare function RBAccordionGroup({ list, type, initialOpen, style }: {
    list: Array<Omit<RBAccordionProps, 'type'> & {
        key?: string | number;
    }>;
    type?: AccordionType;
    initialOpen?: boolean;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
