import { type ReactNode } from 'react';
export type ToastItem = {
    id: number;
    title?: ReactNode;
    content?: ReactNode;
    type?: 'success' | 'fail' | 'info';
};
export declare const toast: {
    openToast(t: Omit<ToastItem, "id">): number;
};
export type ToastPosition = 'top' | 'top-left' | 'top-right' | 'bottom' | 'bottom-left' | 'bottom-right';
export declare function SimpleToast({ item, position, open, onClose, pauseTimer, resumeTimer }: {
    item: ToastItem;
    position: ToastPosition;
    open: boolean;
    onClose: () => void;
    pauseTimer?: () => void;
    resumeTimer?: () => void;
}): import("react").JSX.Element;
export declare function ToastContainer({ position, duration }: {
    position?: ToastPosition;
    duration?: number;
    toastComponent?: unknown;
}): import("react").JSX.Element | null;
