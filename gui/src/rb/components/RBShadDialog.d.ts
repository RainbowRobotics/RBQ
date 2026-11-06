import { type ReactNode } from 'react';
import { type DialogSize } from './RBDialog';
export type ShadDialogSize = DialogSize;
export declare function RBShadDialog({ trigger, defaultOpen, size, showCloseButton, title, description, footer, children }: {
    trigger: (open: () => void) => ReactNode;
    defaultOpen?: boolean;
    size?: ShadDialogSize;
    showCloseButton?: boolean;
    title?: ReactNode;
    description?: ReactNode;
    footer?: ReactNode;
    children?: ReactNode;
}): import("react").JSX.Element;
