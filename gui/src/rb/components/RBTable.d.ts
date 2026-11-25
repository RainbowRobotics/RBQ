import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type TableColumn<T> = {
    fieldId: string;
    label: ReactNode;
    width: number;
    align?: 'left' | 'center' | 'right';
    sortable?: boolean;
    sortValue?: (item: T) => string | number;
    expandable?: boolean;
    render: (item: T, index: number) => ReactNode;
};
export type RBTableProps<T> = {
    list: T[];
    columns: TableColumn<T>[];
    hideThead?: boolean;
    expand?: (item: T, index: number) => ReactNode;
    style?: StyleProp<ViewStyle>;
};
export declare function RBTable<T>({ list, columns, hideThead, expand, style }: RBTableProps<T>): import("react").JSX.Element;
