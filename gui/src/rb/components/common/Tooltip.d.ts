import { type ReactNode } from 'react';
export type TooltipProps = {
    anchor: ReactNode;
    children?: ReactNode;
    place?: 'top' | 'bottom' | 'left' | 'right';
    events?: ('click' | 'hover')[];
    tooltipColor?: string;
    tooltipTextColor?: string;
    open?: boolean;
    width?: number;
};
export declare function Tooltip({ anchor, children, place, events, tooltipColor, tooltipTextColor, open: forced, width }: TooltipProps): import("react").JSX.Element;
