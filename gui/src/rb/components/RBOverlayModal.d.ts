import { type ReactNode } from 'react';
export declare function RbModal({ visible, onRequestClose, lockScroll, children }: {
    visible: boolean;
    onRequestClose?: () => void;
    lockScroll?: boolean;
    children: ReactNode;
}): import("react").JSX.Element | null;
export declare function RBOverlayModal({ open, onClose, overlayAnim, nativeOverlay, keepMounted, children }: {
    open: boolean;
    onClose?: () => void;
    overlayAnim?: object | null;
    nativeOverlay?: object;
    keepMounted?: boolean;
    children: ReactNode;
}): import("react").JSX.Element;
