export type SeatMemory = { has(key: string): boolean; set(key: string, held: boolean): void };

const runSet = new Set<string>();
const runSeats: SeatMemory = { has: (k) => runSet.has(k), set: (k, on) => { if (on) runSet.add(k); else runSet.delete(k); } };
let savedSeats: SeatMemory = runSeats;

export function setSavedSeats(m: SeatMemory) { savedSeats = m; }
export const seatsFor = (key: string): SeatMemory => (key.startsWith('ip:') ? runSeats : savedSeats);
