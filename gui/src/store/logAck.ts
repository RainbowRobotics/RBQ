import { create } from 'zustand';

type LogAckState = { ackLen: number; setAck: (n: number) => void };

export const useLogAck = create<LogAckState>((set) => ({
  ackLen: 0,
  setAck: (n) => set({ ackLen: n }),
}));
