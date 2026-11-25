import { createContext, useContext, useEffect, useMemo, useState, type ReactNode } from 'react';
import { Platform } from 'react-native';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { Fonts, Radius, Spacing, Themes, type Palette, type ThemeName } from '@/constants/theme';

const THEME_KEY = 'rbq-theme';

type ThemeContextValue = {
  name: ThemeName;
  c: Palette;
  radius: typeof Radius;
  spacing: typeof Spacing;
  fonts: typeof Fonts;
  setTheme: (n: ThemeName) => void;
  toggle: () => void;
};

export const ThemeContext = createContext<ThemeContextValue | null>(null);
export type { ThemeContextValue };

export function ThemeProvider({ children, initial = 'light' }: { children: ReactNode; initial?: ThemeName }) {
  const [name, setName] = useState<ThemeName>(initial);
  useEffect(() => {
    const q = Platform.OS === 'web' && typeof location !== 'undefined'
      ? new URLSearchParams(location.search).get('theme') : null;
    if (q === 'dark' || q === 'light') { setName(q); return; }
    AsyncStorage.getItem(THEME_KEY).then((v) => { if (v === 'dark' || v === 'light') setName(v); }).catch(() => {});
  }, []);
  const value = useMemo<ThemeContextValue>(
    () => {
      const setTheme = (n: ThemeName) => { setName(n); AsyncStorage.setItem(THEME_KEY, n).catch(() => {}); };
      return {
        name,
        c: Themes[name],
        radius: Radius,
        spacing: Spacing,
        fonts: Fonts,
        setTheme,
        toggle: () => setTheme(name === 'dark' ? 'light' : 'dark'),
      };
    },
    [name],
  );
  return <ThemeContext.Provider value={value}>{children}</ThemeContext.Provider>;
}

export function useTheme(): ThemeContextValue {
  const ctx = useContext(ThemeContext);
  if (!ctx) throw new Error('useTheme must be used within ThemeProvider');
  return ctx;
}
