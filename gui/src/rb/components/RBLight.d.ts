import { type StyleProp, type ViewStyle } from 'react-native';
import { type LightColor, type LightSize } from '../vendor/light.variants';
export type { LightColor, LightSize };
export declare function RBLight({ color, size, style }: {
    color?: LightColor;
    size?: LightSize;
    style?: StyleProp<ViewStyle>;
}): import("react").JSX.Element;
