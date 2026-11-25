import { useCallback, useEffect } from 'react';
import { create } from 'zustand';

export type ViewMode = { key: string; exit: () => void; leave: () => void };

export const spectateTop = (ddBelow: boolean) => (ddBelow ? 104 : 60);
export const SPECTATE_PILL = { mt: 8, h: 40 };
export function homeColumn(o: {
  ddBelow: boolean; spectating: boolean; statusH: number; bannerH: number;
  winH?: number; bottomH?: number; toolH?: number; need?: number;
}) {
  const limit = o.winH == null ? Infinity : o.winH - (o.bottomH ?? 0) - 8;
  const need = o.need ?? 0;
  const first = (spect: boolean) => (spect ? spectateTop(o.ddBelow) + SPECTATE_PILL.mt + SPECTATE_PILL.h + 8 : o.ddBelow ? 108 : 76);
  const bannerAt = (spect: boolean, status: boolean) => (status ? first(spect) + o.statusH + 6 : spect ? first(spect) : 104);
  let showSpectate = o.spectating;
  let showStatus = true;
  const short = () => need > 0 && bannerAt(showSpectate, showStatus && o.statusH > 0) + need > limit;
  if (short() && o.statusH > 0) showStatus = false;
  if (short() && showSpectate) showSpectate = false;
  const banner = bannerAt(showSpectate, showStatus && o.statusH > 0);
  const tool = banner + (o.bannerH > 0 ? o.bannerH + 8 : 0);
  return {
    status: first(showSpectate), showStatus, showSpectate, banner, tool,
    toolFits: tool + (o.toolH ?? 0) <= limit,
    toastRoom: limit - banner,
  };
}

export function useHomeColumn(spectating: boolean, winH: number, toolH = 0) {
  const ddBelow = useViewport((s) => s.ddBelow);
  const statusH = useViewport((s) => s.statusH);
  const bannerH = useViewport((s) => s.bannerH);
  const need = useViewport((s) => s.toastNeed);
  const bottomH = useViewport((s) => bottomUsed(s.bottomUse));
  return homeColumn({ ddBelow, spectating, statusH, bannerH, winH, bottomH, toolH, need });
}

type ViewportState = {
  key: string;
  p2g: boolean;
  ptzSens: number;
  ddOpen: boolean;
  setDdOpen: (v: boolean) => void;
  barPopOpen: boolean;
  setBarPopOpen: (v: boolean) => void;
  ddBelow: boolean;
  setDdBelow: (v: boolean) => void;
  statusH: number;
  setStatusH: (h: number) => void;
  bannerH: number;
  setBannerH: (h: number) => void;
  bottomUse: Record<string, number>;
  setBottomUse: (key: string, h: number) => void;
  toastNeed: number;
  setToastNeed: (h: number) => void;
  viewMode: ViewMode | null;
  setViewMode: (v: ViewMode | null) => void;
  setKey: (k: string) => void;
  setP2g: (v: boolean) => void;
  setPtzSens: (v: number) => void;
};

export function useBottomUse(key: string, bottom: number, on = true) {
  useEffect(() => { if (!on) useViewport.getState().setBottomUse(key, 0); }, [key, on]);
  useEffect(() => () => useViewport.getState().setBottomUse(key, 0), [key]);
  return useCallback((e: { nativeEvent: { layout: { height: number } } }) =>
    useViewport.getState().setBottomUse(key, e.nativeEvent.layout.height > 0 ? bottom + e.nativeEvent.layout.height : 0), [key, bottom]);
}
export const bottomUsed = (u: Record<string, number>) => Math.max(0, ...Object.values(u));

export const useViewport = create<ViewportState>((set) => ({
  key: 'pose3d',
  maximized: true,
  p2g: false,
  ptzSens: 0.6,
  ddOpen: false,
  setDdOpen: (ddOpen) => set({ ddOpen }),
  barPopOpen: false,
  setBarPopOpen: (barPopOpen) => set({ barPopOpen }),
  ddBelow: false,
  setDdBelow: (ddBelow) => set({ ddBelow }),
  statusH: 0,
  setStatusH: (statusH) => set({ statusH: Math.round(statusH) }),
  bannerH: 0,
  setBannerH: (bannerH) => set({ bannerH: Math.round(bannerH) }),
  bottomUse: {},
  toastNeed: 0,
  setToastNeed: (h) => set({ toastNeed: Math.round(h) }),
  setBottomUse: (key, h) => set((s) => (s.bottomUse[key] === Math.round(h) ? s : { bottomUse: { ...s.bottomUse, [key]: Math.round(h) } })),
  viewMode: null,
  setViewMode: (viewMode) => set({ viewMode }),
  setKey: (k) => {
    const m = useViewport.getState().viewMode;
    const leaving = m && m.key !== k ? m : null;
    set({ key: k, ...(leaving ? { viewMode: null } : {}) });
    leaving?.leave();
  },
  setP2g: (v) => set({ p2g: v }),
  setPtzSens: (ptzSens) => set({ ptzSens }),
}));
