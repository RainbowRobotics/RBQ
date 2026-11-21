import { type ReactNode, type RefObject } from 'react';
import { type TextStyle, type View } from 'react-native';
import { type RbThemeName, type RbColorKey, type RbTypoKey, type RbShadowKey } from './tokens';
type RbTheme = {
    name: RbThemeName;
    c: (k: RbColorKey) => string;
    t: (k: RbTypoKey) => TextStyle;
    sh: (k: RbShadowKey) => object;
    inline: boolean;
    toFrame: (x: number, y: number) => Promise<{
        x: number;
        y: number;
    }>;
    portal?: {
        host: HTMLElement | null;
        set: (key: string, node: ReactNode | null) => void;
        lock: (key: string, on: boolean) => void;
    };
    viewport?: {
        width: number;
        height: number;
    };
};
export declare function RbPortalHost({ store, hostRef }: {
    store: PortalStore;
    hostRef?: (n: unknown) => void;
}): import("react").JSX.Element;
type PortalStore = {
    get: () => Record<string, ReactNode>;
    subscribe: (l: () => void) => () => void;
    set: (key: string, node: ReactNode | null) => void;
};
export declare function usePortalState(): {
    store: PortalStore;
    api: {
        host: HTMLElement | null;
        set: (key: string, node: ReactNode | null) => void;
        lock: (key: string, on: boolean) => void;
    };
    locked: boolean;
    hostRef: (n: unknown) => void;
};
export declare function usePortalSlot(visible: boolean, node: ReactNode, lock?: boolean): {
    portaled: boolean;
    element: ReactNode;
};
export declare function RbProvider({ theme, inline, frameRef, portal, viewport, children }: {
    theme?: RbThemeName;
    inline?: boolean;
    frameRef?: RefObject<View | null>;
    portal?: RbTheme['portal'];
    viewport?: RbTheme['viewport'];
    children: ReactNode;
}): import("react").JSX.Element;
export declare function useOutsidePress(refs: RefObject<View | null>[], onOutside: () => void, active: boolean): void;
export declare function useViewport(): {
    width: number;
    height: number;
};
export declare function useRb(): RbTheme;
export {};
