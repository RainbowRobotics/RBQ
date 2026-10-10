import { create } from 'zustand';
import { bindRobotCache } from '@/store/robotSettings';
import { normalizeLed, sameLed, type LedSetting, type LedSide } from '@/lib/ledBottom';

export type LedKnown = { s: LedSetting; from: 'robot' | 'sent'; at: number } | null;

type LedBottomStore = {
  right: LedKnown;
  left: LedKnown;
  remember: (side: LedSide, s: LedSetting, from: 'robot' | 'sent', at: number) => void;
};

export const useLedBottom = create<LedBottomStore>((set, get) => ({
  right: null,
  left: null,
  remember: (side, s, from, at) => {
    const cur = get()[side];
    if (cur && at < cur.at) return;
    if (cur && cur.from === from && sameLed(cur.s, s)) return;
    set({ [side]: { s: normalizeLed(s), from, at } } as Pick<LedBottomStore, LedSide>);
  },
}));

bindRobotCache(useLedBottom, ['right', 'left'], 'ledBottom');
