import { type ReactNode } from 'react';
export type ListMapProps<T> = {
    list: T[] | null | undefined;
    loading?: boolean;
    noDataMsg?: ReactNode;
    skeletonElement?: ReactNode;
    skeletonCnt?: number;
    children: (p: {
        item: T;
        index: number;
    }) => ReactNode;
};
export declare function ListMap<T>({ list, loading, noDataMsg, skeletonElement, skeletonCnt, children }: ListMapProps<T>): import("react").JSX.Element;
