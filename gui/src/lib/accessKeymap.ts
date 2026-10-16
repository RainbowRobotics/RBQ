export type AccessLevel = 1 | 2 | 3;

export const KEYMAP_MIN_LEVEL: AccessLevel = 2;

export function nextKeymap(prev: AccessLevel, next: AccessLevel, current: boolean): boolean {
  return prev === next ? current : next >= KEYMAP_MIN_LEVEL;
}
