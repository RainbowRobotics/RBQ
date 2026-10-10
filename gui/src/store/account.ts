import { create } from 'zustand';
import type { RemoteAccount } from '@/lib/remoteLogin';

type AccountState = {
  pin: string;
  account: RemoteAccount | null;
  robots: RemoteAccount[];
  selected: RemoteAccount | null;
  setSelected: (a: RemoteAccount | null) => void;
  setSession: (pin: string, account: RemoteAccount, robots: RemoteAccount[]) => void;
  refresh: (account: RemoteAccount, robots: RemoteAccount[]) => void;
  setPin: (pin: string) => void;
  restored: boolean;
  markRestored: () => void;
  clear: () => void;
};

export const useAccount = create<AccountState>((set) => ({
  pin: '',
  account: null,
  robots: [],
  selected: null,
  setSelected: (selected) => set({ selected }),
  setSession: (pin, account, robots) =>
    set({ pin, account, robots, selected: robots.find((r) => r.robotSerial) ?? null }),
  setPin: (pin) => set({ pin }),
  restored: false,
  markRestored: () => set({ restored: true }),
  refresh: (account, robots) => set((s) => ({
    account, robots,
    selected: robots.find((r) => r.robotSerial && r.robotSerial === s.selected?.robotSerial)
              ?? robots.find((r) => r.robotSerial) ?? null,
  })),
  clear: () => set({ pin: '', account: null, robots: [], selected: null }),
}));

export const MIN_REGISTER_LEVEL = 3;
