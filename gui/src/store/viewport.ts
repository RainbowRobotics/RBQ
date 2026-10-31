import { create } from 'zustand';

export type ViewMode = { key: string; exit: () => void; leave: () => void };

type ViewportState = {
  key: string;
  p2g: boolean;
  ptzSens: number;
  ddOpen: boolean;
  setDdOpen: (v: boolean) => void;
  barPopOpen: boolean;
  setBarPopOpen: (v: boolean) => void;
  viewMode: ViewMode | null;
  setViewMode: (v: ViewMode | null) => void;
  setKey: (k: string) => void;
  setP2g: (v: boolean) => void;
  setPtzSens: (v: number) => void;
};

export const useViewport = create<ViewportState>((set) => ({
  key: 'pose3d',
  maximized: true,
  p2g: false,
  ptzSens: 0.6,
  ddOpen: false,
  setDdOpen: (ddOpen) => set({ ddOpen }),
  barPopOpen: false,
  setBarPopOpen: (barPopOpen) => set({ barPopOpen }),
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
