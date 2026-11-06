export type JoyStickProps = {
    size?: number;
    paddingSize?: number;
    stickAreaColor?: string;
    arcColor?: string;
    disabled?: boolean;
    onStart?: () => void;
    onMove?: (v: {
        x: number;
        y: number;
        direction: {
            left: boolean;
            right: boolean;
            top: boolean;
            bottom: boolean;
        };
    }) => void;
    onEnd?: () => void;
};
export declare function JoyStick({ size, paddingSize, stickAreaColor, arcColor, disabled, onStart, onMove, onEnd }: JoyStickProps): import("react").JSX.Element;
