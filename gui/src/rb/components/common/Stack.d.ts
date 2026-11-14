import type { ReactNode } from 'react';
import { type FlexStyle, type StyleProp, type ViewStyle } from 'react-native';
export type StackProps = {
    children?: ReactNode;
    justifyContent?: FlexStyle['justifyContent'];
    alignItems?: FlexStyle['alignItems'];
    alignSelf?: FlexStyle['alignSelf'];
    columnGap?: number;
    rowGap?: number;
    style?: StyleProp<ViewStyle>;
};
export declare const HStack: (p: StackProps) => import("react").JSX.Element;
export declare const VStack: (p: StackProps) => import("react").JSX.Element;
