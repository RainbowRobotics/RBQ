import { type StyleProp, type ViewStyle } from 'react-native';
export type BatteryStatus = 'empty' | 'critical' | 'low' | 'normal' | 'full';
export type BatterySize = 'xs' | 'sm' | 'md' | 'lg';
export type RBBatteryProps = {
    size?: BatterySize;
    status?: BatteryStatus;
    percent?: number;
    charging?: boolean;
    borderColor?: string;
    showPercentLabel?: boolean;
    style?: StyleProp<ViewStyle>;
};
export declare function RBBattery({ size, status, percent, charging, borderColor, showPercentLabel, style }: RBBatteryProps): import("react").JSX.Element;
