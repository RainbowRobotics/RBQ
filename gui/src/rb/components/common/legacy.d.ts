import { type PressableProps } from 'react-native';
declare const PALETTE: {
    readonly light: {
        readonly 'label-normal': "#171719";
        readonly 'label-strong': "#000000";
        readonly 'label-neutral': "rgba(46,47,51,0.88)";
        readonly 'label-alternative': "rgba(55,56,60,0.61)";
        readonly 'label-assistive': "rgba(55,56,60,0.28)";
        readonly 'label-disable': "rgba(55,56,60,0.16)";
        readonly 'label-inverse-normal': "#f7f7f8";
        readonly 'line-solid-neutral': "#eaebec";
        readonly 'line-solid-normal': "#e1e2e4";
        readonly 'line-normal-normal': "rgba(112,115,124,0.22)";
        readonly 'line-normal-neutral': "rgba(112,115,124,0.16)";
        readonly 'background-normal-alternative': "#f7f7f8";
        readonly 'background-normal-normal': "#ffffff";
        readonly 'interaction-disable': "#f4f4f5";
        readonly 'static-white': "#ffffff";
        readonly 'static-black': "#000000";
        readonly 'static-gray85': "#313337";
        readonly 'static-gray05': "#f7f7f8";
        readonly 'primary-normal': "#ff0066";
        readonly 'body-fg': "#0a0a0a";
        readonly 'fill-alternative': "rgba(112,115,124,0.05)";
        readonly 'fill-normal': "rgba(112,115,124,0.08)";
        readonly 'fill-strong': "rgba(112,115,124,0.16)";
    };
    readonly dark: {
        readonly 'label-normal': "#f7f7f8";
        readonly 'label-strong': "#ffffff";
        readonly 'label-neutral': "rgba(194,196,200,0.88)";
        readonly 'label-alternative': "rgba(174,176,182,0.61)";
        readonly 'label-assistive': "rgba(174,176,182,0.28)";
        readonly 'label-disable': "rgba(152,155,162,0.16)";
        readonly 'label-inverse-normal': "#171719";
        readonly 'line-solid-neutral': "#333438";
        readonly 'line-solid-normal': "#37383c";
        readonly 'line-normal-normal': "rgba(112,115,124,0.32)";
        readonly 'line-normal-neutral': "rgba(112,115,124,0.28)";
        readonly 'background-normal-alternative': "#0f0f10";
        readonly 'background-normal-normal': "#1b1c1e";
        readonly 'interaction-disable': "#2e2f33";
        readonly 'static-white': "#ffffff";
        readonly 'static-black': "#000000";
        readonly 'static-gray85': "#313337";
        readonly 'static-gray05': "#f7f7f8";
        readonly 'primary-normal': "#ff0066";
        readonly 'body-fg': "#0a0a0a";
        readonly 'fill-alternative': "rgba(112,115,124,0.12)";
        readonly 'fill-normal': "rgba(112,115,124,0.22)";
        readonly 'fill-strong': "rgba(112,115,124,0.28)";
    };
};
export type LegacyColorKey = keyof (typeof PALETTE)['light'];
export declare const LegacyThemeCtx: import("react").Context<"light" | "dark" | null>;
export declare function useLegacy(): {
    theme: import("../../tokens").RbThemeName;
    lc: (k: LegacyColorKey) => string;
};
export declare const PRETENDARD = "Pretendard Variable";
export declare const legacyText: (size: number, lineHeight: number, weight: "400" | "500" | "600" | "700" | "900", letterSpacing?: number | boolean) => {
    readonly fontFamily: "Pretendard Variable";
    readonly fontSize: number;
    readonly lineHeight: number;
    readonly fontWeight: "400" | "500" | "700" | "600" | "900";
    readonly letterSpacing: number;
};
export declare const web: (s: Record<string, unknown>) => object | null;
export declare const shadow: (css: string) => {
    boxShadow: string;
};
export declare const tr: (property: string, ms: number, easing?: string) => object;
export declare function cssAnim(name: string, keyframes: string, animation: string, extra?: string): {
    anim: string;
} | undefined;
export declare function hoverStyle(fn: (s: {
    hovered?: boolean;
    pressed: boolean;
}) => unknown): PressableProps['style'];
export {};
