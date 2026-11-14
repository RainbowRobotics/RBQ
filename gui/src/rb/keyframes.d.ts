export type Channel = 'opacity' | 'translateX' | 'translateY' | 'scaleX' | 'scaleY' | 'rotate';
export declare const CHANNELS: Channel[];
export type Stop = {
    at: number;
    v: Partial<Record<Channel, number>>;
};
export type Anim = {
    stops: Stop[];
    duration: number;
    delay: number;
    easing: string;
    iterations: number;
    direction: 'normal' | 'reverse' | 'alternate' | 'alternate-reverse';
    fill: 'none' | 'forwards' | 'backwards' | 'both';
};
export declare function parseTransform(v: string): Partial<Record<Channel, number>>;
export declare function parseKeyframes(css: string): Stop[];
export declare function parseAnimation(shorthand: string): Omit<Anim, 'stops'>;
export declare function toAnim(kfCss: string, animation: string): Anim | null;
export declare function tracks(stops: Stop[], base: Record<Channel, number>): Partial<Record<Channel, {
    at: number[];
    v: number[];
}>>;
