import { type ReactNode } from 'react';
import { type ToastPosition, type ToastType, type ToastVariant } from '../vendor/toast.variants';
export type { ToastType, ToastVariant, ToastPosition };
export type ToastItem = {
    id: number;
    title?: string;
    content?: string;
    type?: ToastType;
    variant?: ToastVariant;
    icon?: ReactNode;
    duration?: number;
};
export declare function RBSimpleToast({ title, content, type, variant, icon, pressed, stacked }: Omit<ToastItem, 'id'> & {
    pressed?: boolean;
    stacked?: boolean;
}): import("react").JSX.Element;
export declare function RBToastContainer({ position, toasts, onClose }: {
    position?: ToastPosition;
    toasts: ToastItem[];
    onClose?: (id: number) => void;
}): import("react").JSX.Element;
