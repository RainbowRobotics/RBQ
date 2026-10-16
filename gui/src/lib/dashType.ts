import { useWindowDimensions } from 'react-native';

const BASE = {
  title: 13,
  body: 13,
  label: 11,
  caption: 11.5,
  button: 12,
  badge: 10,
  hero: 34,
  heroSub: 15,
} as const;

export type DashType = { -readonly [K in keyof typeof BASE]: number } & { k: number };

const half = (n: number) => Math.round(n * 2) / 2;

export function dashType(windowWidth: number): DashType {
  const k = Math.min(1.15, Math.max(1, windowWidth / 1200));
  const out = { k } as DashType;
  for (const key of Object.keys(BASE) as (keyof typeof BASE)[]) out[key] = half(BASE[key] * k);
  return out;
}

export function useDashType(): DashType {
  return dashType(useWindowDimensions().width);
}
