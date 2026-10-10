import { create } from 'zustand';

type DockViewState = {
  open: boolean;
  paused: boolean;
  dockedAt: number | null;
  closedAt: number | null;
  show: () => void;
  hide: () => void;
  setPaused: (paused: boolean) => void;
  setDockedAt: (dockedAt: number | null) => void;
};

export const useDockView = create<DockViewState>((set) => ({
  open: false,
  paused: false,
  dockedAt: null,
  closedAt: null,
  show: () => set({ open: true }),
  hide: () => set((s) => (s.open ? { open: false, paused: false, closedAt: Date.now() } : { paused: false })),
  setPaused: (paused) => set({ paused }),
  setDockedAt: (dockedAt) => set({ dockedAt }),
}));
