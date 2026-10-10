import { type RefObject } from 'react';
import { type LayoutChangeEvent, type View } from 'react-native';
import { type Align, type PlaceResult, type Placement, type Rect, type Side } from './placement';
export type { Align, Side, Placement, PlaceResult };
export type PopperOptions = {
    anchorRef: RefObject<View | null>;
    open: boolean;
    side: Side;
    align?: Align;
    sideOffset?: number | ((side: Side) => number);
    alignOffset?: number;
    collisionPadding?: number;
    shiftPadding?: number;
    avoidCollisions?: boolean;
    fallbacks?: Placement[];
    sticky?: 'partial' | 'always';
    pickSide?: (a: {
        anchor: Rect;
        size: {
            width: number;
            height: number;
        };
        boundary: Rect;
        side: Side;
    }) => Side;
    fitHeight?: (availableHeight: number) => number;
    knownSize?: {
        width: number;
        height: number;
    } | null;
};
export type Popper = PlaceResult & {
    ready: boolean;
    anchor: Rect | null;
    height: number;
    onContentLayout: (e: LayoutChangeEvent) => void;
    style: object;
    rel: {
        left: number;
        top: number;
    };
};
export declare function usePopperPlacement(o: PopperOptions): Popper;
export declare const tooltipSide: (arrow: number, space: number) => NonNullable<PopperOptions["pickSide"]>;
export declare function usePresence(open: boolean, exitMs: number, opts?: {
    timer?: boolean;
}): {
    mounted: boolean;
    state: "open" | "closed";
    exiting: boolean;
    ref: (n: unknown) => void;
};
export declare function useFocusReturn(mounted: boolean, target: RefObject<unknown>): void;
export declare function focusWhenShown(get: () => unknown): () => void;
