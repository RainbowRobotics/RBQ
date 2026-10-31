import { type ReactNode } from 'react';
import { type GestureResponderEvent, type LayoutChangeEvent } from 'react-native';
export declare const rbUiHoverCursor: (hover: boolean) => object | null;
export declare function useRipple(disabled?: boolean, color?: string, duration?: number): {
    onLayout: (e: LayoutChangeEvent) => void;
    onPointerDown: (e: {
        nativeEvent: {
            clientX: number;
            clientY: number;
        };
    }) => void;
    onTouchStart: (e: GestureResponderEvent) => void;
    render: (radius: number) => ReactNode;
};
