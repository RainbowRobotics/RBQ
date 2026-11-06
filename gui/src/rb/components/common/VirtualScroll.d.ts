import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
type Base<T> = {
    list: T[] | null | undefined;
    noDataMsg?: ReactNode;
    element: (p: {
        item: T;
        index: number;
    }) => ReactNode;
    itemHeight: number;
    style?: StyleProp<ViewStyle>;
    overscan?: number;
};
export declare const ListVirtualScroll: <T>(p: Base<T>) => import("react").JSX.Element;
export declare const GridVirtualScroll: <T>(p: Base<T> & {
    itemWidth: (containerWidth: number) => number;
    gapX?: number;
    gapY?: number;
}) => import("react").JSX.Element;
export {};
