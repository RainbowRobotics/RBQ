export type View3dTheme = { bg: string; grid: string; label: string };

export const LEGACY_VIEW3D_THEMES: View3dTheme[] = [
  { bg: '#d8dce8', grid: '#9099aa', label: 'Default' },
  { bg: '', grid: '', label: 'System' },
  { bg: '#2d3142', grid: '#555566', label: 'Dark' },
  { bg: '#1a1a2e', grid: '#333348', label: 'Night' },
];

export function resolveLegacyView3dTheme(index: number, appBg: string, appDark: boolean): View3dTheme {
  const t = LEGACY_VIEW3D_THEMES[index] ?? LEGACY_VIEW3D_THEMES[0];
  if (index === 1) return { bg: appBg, grid: appDark ? '#444455' : '#9099aa', label: 'System' };
  return t;
}

export type ModernView3dTheme = { bg?: string; grid: string; gridCenter: string; label: string };
export const MODERN_VIEW3D_THEMES: ModernView3dTheme[] = [
  { bg: undefined, grid: '#4d9cf5', gridCenter: '#2a3646', label: 'Default' },
  { bg: '#0E0F12', grid: '#3a4d6b', gridCenter: '#232a36', label: 'System' },
  { bg: '#1b1f2a', grid: '#3f4a5e', gridCenter: '#2a3140', label: 'Dark' },
  { bg: '#12122a', grid: '#33385a', gridCenter: '#22243f', label: 'Night' },
];
