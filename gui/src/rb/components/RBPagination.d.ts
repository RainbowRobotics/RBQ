import { type StyleProp, type ViewStyle } from 'react-native';
export type RBPaginationProps = {
    currPage: number;
    totalCount: number;
    pageSize?: number;
    listSize?: number;
    onPageChange?: (page: number) => void;
    style?: StyleProp<ViewStyle>;
};
export declare function RBPagination({ currPage, totalCount, pageSize, listSize, onPageChange, style }: RBPaginationProps): import("react").JSX.Element;
