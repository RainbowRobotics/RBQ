import { type ReactNode } from 'react';
import { type Side } from './popper';
export type RBTooltipProps = {
    anchor: ReactNode;
    children?: ReactNode;
    place?: Side;
    events?: ('click' | 'hover')[];
    width?: number;
    tooltipColor?: string;
    tooltipTextColor?: string;
    open?: boolean;
    space?: number;
};
export declare function RBTooltip({ anchor, children, place, events, width, tooltipColor, tooltipTextColor, open: forced, space }: RBTooltipProps): import("react").JSX.Element;
