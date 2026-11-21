import { type ComponentType } from 'react';
export type GalleryIcon = ComponentType<{
    size?: number;
    color?: string;
}>;
export declare function RBIconGallery({ title, icons }: {
    title: string;
    icons: Record<string, GalleryIcon>;
}): import("react").JSX.Element;
