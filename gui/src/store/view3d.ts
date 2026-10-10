import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';

export type LidarMode = 'off' | 'on';

const clamp = (v: number, lo: number, hi: number) => Math.max(lo, Math.min(hi, v));

type View3dStore = {
  gridVisible: boolean;
  gridSize: number;
  themeIndex: number;
  viewMode: number;
  lidarMode: LidarMode;
  hmGrid: boolean; hmStair: boolean; hmEdge: boolean; hmFoot: boolean;
  setGridVisible: (v: boolean) => void;
  setGridSize: (v: number) => void;
  stepGridSize: (d: number) => void;
  setThemeIndex: (i: number) => void;
  setViewMode: (i: number) => void;
  setLidarMode: (m: LidarMode) => void;
  setHmLayer: (k: 'hmGrid' | 'hmStair' | 'hmEdge' | 'hmFoot', v: boolean) => void;
};

export const useView3d = create<View3dStore>()(
  persist(
    (set) => ({
      gridVisible: true,
      gridSize: 25,
      themeIndex: 0,
      viewMode: 2,
      lidarMode: 'off',
      hmGrid: false, hmStair: false, hmEdge: false, hmFoot: false,
      setGridVisible: (v) => set({ gridVisible: v }),
      setGridSize: (v) => set({ gridSize: clamp(Math.round(v), 1, 500) }),
      stepGridSize: (d) => set((s) => ({ gridSize: clamp(Math.round(s.gridSize + d), 1, 500) })),
      setThemeIndex: (i) => set({ themeIndex: clamp(i, 0, 3) }),
      setViewMode: (i) => set({ viewMode: clamp(i, 0, 2) }),
      setLidarMode: (m) => set({ lidarMode: m }),
      setHmLayer: (k, v) => set({ [k]: v } as Partial<View3dStore>),
    }),
    {
      name: 'rbq-view3d',
      storage: createJSONStorage(() => AsyncStorage),
      version: 2,
      migrate: (persisted: any, version) => {
        if (version < 1 && persisted && typeof persisted === 'object') {
          persisted.lidarMode = persisted.lidarMode && persisted.lidarMode !== 'off' ? 'on' : 'off';
        }
        if (version < 2 && persisted && typeof persisted === 'object') persisted.hmFoot = false;
        return persisted;
      },
      partialize: (s) => ({
        gridVisible: s.gridVisible, gridSize: s.gridSize,
        themeIndex: s.themeIndex, viewMode: s.viewMode, lidarMode: s.lidarMode,
        hmGrid: s.hmGrid, hmStair: s.hmStair, hmEdge: s.hmEdge, hmFoot: s.hmFoot,
      }),
    },
  ),
);
