export type DateRange = {
    from?: Date;
    to?: Date;
    timeEnabled: boolean;
};
export type RBCalendarProps = {
    mode: 'single' | 'range';
    value: DateRange;
    onChange: (v: DateRange) => void;
    onReset?: () => void;
    onCancel?: () => void;
    onConfirm?: () => void;
    showPresets?: boolean;
    showFooter?: boolean;
};
export declare const formatDate: (d?: Date, time?: boolean) => string;
export declare function RBCalendar({ mode, value, onChange, onReset, onCancel, onConfirm, showPresets, showFooter, epoch }: RBCalendarProps & {
    epoch?: number;
}): import("react").JSX.Element;
