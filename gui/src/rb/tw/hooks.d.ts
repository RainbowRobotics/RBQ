import { type ReactNode } from 'react';
import { type PressableProps, type TextProps, type ViewProps } from 'react-native';
import { type Style, type TwResult } from './tw';
import { resolve, type Resolved } from './resolve';
export { resolve, type Resolved };
export declare function webStyle(r: Resolved, extra?: {
    cursor?: string;
}): object | null;
export declare function scrollProps(r: Resolved): {
    horizontal: boolean;
    showsHorizontalScrollIndicator: boolean;
    showsVerticalScrollIndicator: boolean;
    style: (Style | object | null)[];
};
export declare function animDataSet(r: Resolved): {
    anim: string;
} | undefined;
export declare function useTwState(opts?: {
    disabled?: boolean;
    extra?: readonly string[];
}): {
    states: Set<string>;
    handlers: {
        onHoverIn: () => void;
        onHoverOut: () => void;
        onPressIn: () => void;
        onPressOut: () => void;
        onFocus: (e: {
            target?: unknown;
        }) => void;
        onBlur: () => void;
    };
    hover: boolean;
    pressed: boolean;
    focused: boolean;
    focusVisible: boolean;
};
export declare function useTw(cls: string, states?: Iterable<string>): {
    result: TwResult;
    r: Resolved;
};
export declare function TwOutline({ r }: {
    r: Resolved;
}): import("react").JSX.Element | null;
type TwProps = {
    cls: string;
    states?: Iterable<string>;
};
export declare function TwView({ cls, states, style, children, ...rest }: ViewProps & TwProps): import("react").JSX.Element;
export declare function TwText({ cls, states, style, ...rest }: TextProps & TwProps): import("react").JSX.Element;
export declare function TwPressable({ cls, extra, disabled, style, children, onHoverIn, onHoverOut, onPressIn, onPressOut, onFocus, onBlur, ...rest }: Omit<PressableProps, 'style' | 'children'> & {
    cls: string;
    extra?: readonly string[];
    style?: ViewProps['style'];
    children?: ReactNode;
}): import("react").JSX.Element;
