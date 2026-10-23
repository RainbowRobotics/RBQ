import { type ReactNode } from 'react';
export type ModalProps = {
    open: boolean;
    onOpenChange?: (open: boolean) => void;
    hideCloseButton?: boolean;
    enableDimClose?: boolean;
    children?: ReactNode;
    width?: number;
};
export declare function Modal({ open, onOpenChange, hideCloseButton, enableDimClose, children, width }: ModalProps): import("react").JSX.Element;
