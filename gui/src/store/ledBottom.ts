import { create } from 'zustand';
import { bindRobotCache } from '@/store/robotSettings';
import { normalizeLed, sameLed, ledTrackNext, ledUsable, ledReportedAt, LED_SIDES, type LedSetting, type LedSide, type LedRead, type LedTrack } from '@/lib/ledBottom';

export type LedKnown = { s: LedSetting; from: 'robot' | 'sent'; at: number } | null;

type LedBottomStore = {
  right: LedKnown;
  left: LedKnown;
  remember: (side: LedSide, s: LedSetting, from: 'robot' | 'sent', at: number) => void;
  live: LedRead | null;
  track: Record<LedSide, LedTrack | null>;
  setLive: (r: LedRead | null) => void;
  liveIp: string | null;
  bindIp: (ip: string) => void;
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
  live: null,
  track: { right: null, left: null },
  setLive: (r) => {
    if (!r || !ledUsable(r, Date.now())) { set({ live: r }); return; }
    const st = get();
    const track = { ...st.track };
    for (const sd of LED_SIDES) {
      const k = st[sd];
      const sentAt = k && k.from === 'sent' && sameLed(k.s, r.st[sd]) ? k.at : null;
      track[sd] = ledTrackNext(st.track[sd], r.st[sd], ledReportedAt(r), sentAt);
    }
    set({ live: r, track });
  },
  liveIp: null,
  bindIp: (ip) => { if (get().liveIp !== ip) set({ liveIp: ip, live: null, track: { right: null, left: null } }); },
}));

bindRobotCache(useLedBottom, ['right', 'left'], 'ledBottom');
