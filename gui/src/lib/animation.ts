import { create } from 'zustand';
import { sendUserCommand } from '@/lib/userCommand';
import { PROGRAM } from '@/lib/robotState';

export const ANIMATION_CMD = {
  RECORD_START: 600,
  RECORD_STOP: 601,
  PLAY_START: 610,
  PLAY_STOP: 611,
} as const;

export const ANIMATION_SLOTS = 16;

export type AnimMode = 'idle' | 'rec' | 'play';

type AnimState = {
  mode: AnimMode;
  slot: number;
  setSlot: (slot: number) => void;
  recordStart: () => void;
  playStart: () => void;
  stop: () => void;
};

export const useAnimation = create<AnimState>((set, get) => ({
  mode: 'idle',
  slot: 0,
  setSlot: (slot) => { if (get().mode === 'idle') set({ slot }); },
  recordStart: () => {
    const { mode, slot } = get();
    if (mode !== 'idle') return;
    sendUserCommand(PROGRAM.QuadWalk, ANIMATION_CMD.RECORD_START, [slot]);
    set({ mode: 'rec' });
  },
  playStart: () => {
    const { mode, slot } = get();
    if (mode !== 'idle') return;
    sendUserCommand(PROGRAM.QuadWalk, ANIMATION_CMD.PLAY_START, [slot]);
    set({ mode: 'play' });
  },
  stop: () => {
    const { mode } = get();
    if (mode === 'rec') sendUserCommand(PROGRAM.QuadWalk, ANIMATION_CMD.RECORD_STOP);
    else if (mode === 'play') sendUserCommand(PROGRAM.QuadWalk, ANIMATION_CMD.PLAY_STOP);
    set({ mode: 'idle' });
  },
}));
