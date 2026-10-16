import { create } from 'zustand';

type ViewportState = {
  key: string;
  p2g: boolean;
  ptzSens: number;
  ddOpen: boolean;
  setDdOpen: (v: boolean) => void;
  barPopOpen: boolean;
  setBarPopOpen: (v: boolean) => void;
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
  setKey: (k) => set({ key: k }),
  setP2g: (v) => set({ p2g: v }),
  setPtzSens: (ptzSens) => set({ ptzSens }),
}));
