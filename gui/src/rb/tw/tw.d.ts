import { type TextStyle, type ViewStyle } from 'react-native';
import { type RbThemeName } from '../tokens';
export type Style = ViewStyle & TextStyle & {
    [k: string]: unknown;
};
export type TwOutline = {
    width?: number;
    color?: string;
    offset?: number;
};
export type TwRing = {
    width?: number;
    color?: string;
    offset?: number;
    offsetColor?: string;
    inset?: boolean;
};
export type TwTransition = {
    property: string[];
    duration: number;
    easing: string;
    delay?: number;
};
export type TwAnimation = {
    name: string;
    keyframes: Record<string, Style>;
    duration: number;
    easing: string;
    fill: string;
    iterationCount?: number | 'infinite';
    delay?: number;
    css: {
        keyframes: string;
        animation: string;
    };
};
export type TwGradient = {
    to: string;
    stops: {
        color?: string;
        pos?: string;
    }[];
};
export type TwGrid = {
    cols?: number | string;
    rows?: string;
    autoRows?: string;
    colStart?: number;
    rowStart?: number;
    rowSpan?: number;
};
export type TwLayer = {
    style: Style;
    states: Record<string, TwLayer>;
    slots: Record<string, TwLayer>;
    pseudo: {
        before?: TwLayer;
        after?: TwLayer;
    };
    outline?: TwOutline;
    ring?: TwRing;
    transition?: TwTransition;
    animation?: TwAnimation;
    gradient?: TwGradient;
    grid?: TwGrid;
    cursor?: string;
    userSelect?: 'none' | 'text' | 'auto' | 'all';
    anim?: AnimMeta;
    hints: Record<string, unknown>;
};
export type TwResult = TwLayer & {
    media: Record<string, TwLayer & {
        minWidth: number;
    }>;
    theme: {
        dark?: TwLayer;
        light?: TwLayer;
    };
    unknown: string[];
    unsupported: {
        cls: string;
        reason: string;
    }[];
};
export declare const EASE_DEFAULT = "cubic-bezier(0.4, 0, 0.2, 1)";
export type Enter = {
    opacity?: number;
    scale?: number;
    x?: number | string;
    y?: number | string;
    rotate?: number;
};
export type AnimMeta = {
    duration?: number;
    ease?: string;
    delay?: number;
    enter?: Enter;
    exit?: Enter;
    animName?: string;
    animDur?: number;
    animEase?: string;
    animIter?: number | 'infinite';
};
export declare function mergeAnim(a: AnimMeta | undefined, b: AnimMeta | undefined): AnimMeta | undefined;
export declare function buildAnimation(m: AnimMeta): TwAnimation;
export declare function tw(classes: string, ctx: {
    theme: RbThemeName;
}): TwResult;
export declare const TEXT_KEYS: Set<string>;
export declare function splitText(style: Style): {
    view: Style;
    text: Style;
};
