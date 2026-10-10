import { type ViewStyle } from 'react-native';
export declare const isWeb: boolean;
export declare const web: (s: object) => object | null;
export declare const androidInput: {
    readonly textAlignVertical: "center";
    readonly includeFontPadding: false;
} | null;
export declare const EASE: {
    readonly default: "cubic-bezier(0.4, 0, 0.2, 1)";
    readonly out: "cubic-bezier(0, 0, 0.2, 1)";
    readonly inOut: "cubic-bezier(0.4, 0, 0.2, 1)";
    readonly ease: "ease";
};
export declare const transition: (property: string, ms?: number, ease?: string) => object;
export declare const COLORS = "color, background-color, border-color, outline-color";
export declare const measureFree: {
    readonly position: "absolute";
    readonly top: 0;
    readonly left: 0;
    readonly right: 0;
} | null;
export declare const collapse: (open: boolean, h: number | undefined, unmeasured?: "auto" | "zero") => {
    maxHeight: number | undefined;
    height?: undefined;
} | {
    height: number;
    maxHeight?: undefined;
};
export type Cursor = 'default' | 'pointer' | 'not-allowed' | 'ew-resize' | 'ns-resize' | 'grab' | 'grabbing';
export declare const cursor: (k: Cursor) => object | null;
export declare const cursorOf: (enabled: Cursor, disabled?: boolean) => object | null;
export declare function useFocusVisible(): {
    focused: boolean;
    onFocus: (e: {
        target?: unknown;
    }) => void;
    onBlur: () => void;
};
export declare function useInteraction(): {
    hover: boolean;
    pressed: boolean;
    handlers: {
        onHoverIn: () => void;
        onHoverOut: () => void;
        onPressIn: () => void;
        onPressOut: () => void;
    };
};
export declare const shadow: (css: string) => {
    boxShadow: string;
};
export declare const ring: (color: string, width?: number) => {
    boxShadow: string;
};
export declare const enter: Record<"fade" | "fadeZoom" | "fadeZoomFromTop" | "fadeZoomFromLeft" | "zoom500" | "fromRight500" | "fromLeft500" | "fromBottom500" | "fromTop500" | "ripple", ViewStyle>;
export declare function useDismiss(ref: {
    current: unknown;
}, open: boolean, onClose: () => void): void;
export declare const slot: (name: string) => object;
