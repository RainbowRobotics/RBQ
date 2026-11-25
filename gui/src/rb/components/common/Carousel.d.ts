import { type ReactNode } from 'react';
import { type StyleProp, type ViewStyle } from 'react-native';
export type CarouselProps = {
    children: ReactNode;
    spaceBetween?: number;
    initialSlide?: number;
    speed?: number;
    loop?: boolean;
    navigation?: boolean;
    threshold?: number;
    longSwipesMs?: number;
    onSlideChange?: (index: number) => void;
    style?: StyleProp<ViewStyle>;
};
export declare function Carousel({ children, spaceBetween, initialSlide, speed, loop, navigation, threshold, longSwipesMs, onSlideChange, style }: CarouselProps): import("react").JSX.Element;
