export type DateRange = {
    from?: Date;
    to?: Date;
};
export declare const calendarBorder: {
    readonly borderWidth: 1;
    readonly borderColor: "#e5e5e5";
    readonly borderBottomColor: "#d1d5dc";
    readonly borderRadius: 4;
};
type Props = {
    numberOfMonths?: number;
    defaultMonth?: Date;
    selected?: Date;
    range?: DateRange;
    onSelectDay?: (d: Date) => void;
};
export declare function Calendar({ numberOfMonths, defaultMonth, selected, range, onSelectDay }: Props): import("react").JSX.Element;
export declare function RangeCalendar({ value, onCancel, onConfirm }: {
    value?: DateRange;
    onCancel?: () => void;
    onConfirm?: (r: DateRange) => void;
}): import("react").JSX.Element;
export {};
