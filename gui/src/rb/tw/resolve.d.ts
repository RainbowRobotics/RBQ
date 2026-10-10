import { type Style, type TwLayer } from './tw';
export type Resolved = {
    style: Style;
    slots: Record<string, Resolved>;
    pseudo: {
        before?: Resolved;
        after?: Resolved;
    };
    outline?: TwLayer['outline'];
    ring?: TwLayer['ring'];
    transition?: TwLayer['transition'];
    animation?: TwLayer['animation'];
    gradient?: TwLayer['gradient'];
    grid?: TwLayer['grid'];
    cursor?: string;
    userSelect?: TwLayer['userSelect'];
    hints: Record<string, unknown>;
};
export declare function resolve(layer: TwLayer, states?: Iterable<string>): Resolved;
