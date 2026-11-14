import { type StyleProp, type ViewStyle } from 'react-native';
export type RBJoyStickProps = {
    size?: number;
    disabled?: boolean;
    onMove?: (x: number, y: number) => void;
    onEnd?: () => void;
    style?: StyleProp<ViewStyle>;
    testID?: string;
};
export declare function RBJoyStick({ size, disabled, onMove, onEnd, style, testID }: RBJoyStickProps): import("react").JSX.Element;
