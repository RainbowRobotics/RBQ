type ClassValue = string | number | boolean | null | undefined | ClassValue[] | Record<string, boolean | null | undefined>;
export declare function cn(...inputs: ClassValue[]): string;
export declare const clsx: typeof cn;
type VariantShape = Record<string, Record<string, ClassValue>>;
type StringToBoolean<T> = T extends 'true' | 'false' ? boolean : T;
type ConfigVariants<V extends VariantShape> = {
    [K in keyof V]?: StringToBoolean<keyof V[K]> | null | undefined;
};
type ConfigVariantsMulti<V extends VariantShape> = {
    [K in keyof V]?: StringToBoolean<keyof V[K]> | StringToBoolean<keyof V[K]>[] | undefined;
};
type Config<V extends VariantShape> = {
    variants?: V;
    defaultVariants?: ConfigVariants<V>;
    compoundVariants?: ((ConfigVariants<V> | ConfigVariantsMulti<V>) & {
        class?: ClassValue;
        className?: ClassValue;
    })[];
};
export type VariantProps<C extends (...args: never[]) => string> = C extends (props?: infer P) => string ? Omit<NonNullable<P>, 'class' | 'className'> : never;
export declare function cva<V extends VariantShape>(base?: ClassValue | Config<V>, config?: Config<V>): (props?: ConfigVariants<V> & {
    class?: ClassValue;
    className?: ClassValue;
}) => string;
export {};
