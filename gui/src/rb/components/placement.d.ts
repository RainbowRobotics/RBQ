export type Side = 'top' | 'right' | 'bottom' | 'left';
export type Align = 'start' | 'center' | 'end';
export type Rect = {
    x: number;
    y: number;
    width: number;
    height: number;
};
export type Placement = {
    side: Side;
    align: Align;
};
export type PlaceOptions = {
    anchor: Rect;
    size: {
        width: number;
        height: number;
    };
    boundary: Rect;
    side: Side;
    align?: Align;
    sideOffset?: number;
    alignOffset?: number;
    collisionPadding?: number;
    shiftPadding?: number;
    avoidCollisions?: boolean;
    sticky?: 'partial' | 'always';
    fallbacks?: Placement[];
};
export type PlaceResult = Placement & {
    x: number;
    y: number;
    availableWidth: number;
    availableHeight: number;
    transformOrigin: string;
};
export declare function place(o: PlaceOptions): PlaceResult;
