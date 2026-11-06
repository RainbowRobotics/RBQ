import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type TdColumnProps = {
    fieldId: string;
    label: ReactNode;
    size?: number;
    sortValue?: string | number;
    expend?: boolean;
    children?: ReactNode;
};
export declare function TdColumn({ children }: TdColumnProps): import("react").JSX.Element;
export declare function TdExpend({ children, style }: {
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export declare function TableRow({ children, style }: {
    children?: ReactNode;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
export type TableProps<T> = {
    list: T[] | null | undefined;
    noDataMsg?: ReactNode;
    loadingElement?: ReactNode;
    children: (p: {
        item: T;
        index: number;
    }) => ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function Table<T>({ list, noDataMsg, children, style }: TableProps<T>): import("react").JSX.Element;
