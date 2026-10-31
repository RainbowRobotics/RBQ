import { MARKER_CAM } from '@/lib/arucoState';

export const DOCK_PANES = [MARKER_CAM.front, MARKER_CAM.rear, MARKER_CAM.bottom] as const;
export const DOCK_PANE_LABELS = ['Front', 'Rear', 'Bottom2'] as const;
export const DOCK_PANE_KINDS = ['COLOR', 'COLOR', 'IR'] as const;
export const DOCK_BAND_TOP = 1 / 4;
export const DOCK_BAND_H = 1 / 3;

export function paneIndexOf(cam: number): number {
  return DOCK_PANES.indexOf(cam as (typeof DOCK_PANES)[number]);
}

export function paneU(paneIdx: number, u: number): number {
  return (paneIdx + u) / DOCK_PANES.length;
}
