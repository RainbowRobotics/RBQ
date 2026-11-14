import { useEffect } from 'react';
import { create } from 'zustand';
import { useRobot } from '@/store/robot';
import { useRobotKind } from '@/modules/registry';

function useNotMineRaw(): boolean {
  const notMine = useRobot((s) => s.conn === 'connected' && !s.isMine);
  const kind = useRobotKind();
  return notMine && !kind;
}

const useSpectatingStore = create<{ on: boolean }>(() => ({ on: false }));

type SpectateMenu = null | 'menu' | 'auto';
const useSpectateMenuStore = create<{ menu: SpectateMenu }>(() => ({ menu: null }));
export const useSpectateMenu = (): SpectateMenu => useSpectateMenuStore((s) => s.menu);
export const setSpectateMenu = (menu: SpectateMenu): void => useSpectateMenuStore.setState({ menu });

export function useSpectating(): boolean {
  return useSpectatingStore((s) => s.on);
}

export function useNotMine(): boolean {
  return useNotMineRaw();
}

export function useSpectatingDriver(): void {
  const on = useNotMineRaw();
  useEffect(() => {
    if (!on) { useSpectatingStore.setState({ on: false }); return; }
    const t = setTimeout(() => useSpectatingStore.setState({ on: true }), 1200);
    return () => clearTimeout(t);
  }, [on]);
}

export async function waitRoleKnown(maxMs = 2000): Promise<void> {
  const t0 = Date.now();
  while (!useRobot.getState().myIp && Date.now() - t0 < maxMs) await new Promise((r) => setTimeout(r, 100));
}
