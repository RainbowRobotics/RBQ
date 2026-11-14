import type { ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type InfiniteScrollProps = {
    list: unknown[];
    totalCount: number;
    loading?: boolean;
    loadingElement?: ReactNode;
    handleLoadMore: () => void;
    children: ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function InfiniteScroll({ list, totalCount, loading, loadingElement, handleLoadMore, children, style }: InfiniteScrollProps): import("react").JSX.Element;
