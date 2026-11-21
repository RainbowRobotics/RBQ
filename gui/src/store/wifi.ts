import { create } from 'zustand';
import { wifiScan, wifiConnect, wifiCurrent, type WifiNetwork } from '@/lib/desktopBridge';

type WifiState = {
  current: WifiNetwork | null;
  networks: WifiNetwork[];
  scanning: boolean;
  connecting: boolean;
  error: string | null;
  refreshCurrent: () => Promise<void>;
  scan: () => Promise<void>;
  connect: (ssid: string, password?: string) => Promise<boolean>;
};

export const useWifi = create<WifiState>((set, get) => ({
  current: null,
  networks: [],
  scanning: false,
  connecting: false,
  error: null,
  refreshCurrent: async () => {
    try {
      set({ current: await wifiCurrent() });
    } catch {
    }
  },
  scan: async () => {
    set({ scanning: true, error: null });
    try {
      set({ networks: await wifiScan(), scanning: false });
    } catch (e) {
      set({ scanning: false, error: e instanceof Error ? e.message : 'WiFi 스캔 실패' });
    }
  },
  connect: async (ssid, password) => {
    set({ connecting: true, error: null });
    try {
      await wifiConnect(ssid, password);
      set({ connecting: false });
      await get().refreshCurrent();
      setTimeout(() => { void import('@/store/robots').then((m) => m.useRobots.getState().adoptLanRobot()); }, 2500);
      return true;
    } catch (e) {
      set({ connecting: false, error: e instanceof Error ? e.message : 'WiFi 연결 실패' });
      return false;
    }
  },
}));
