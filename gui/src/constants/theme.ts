import { Platform } from 'react-native';
import { colors as rbColors, rounded, type RbThemeName } from '@/rb/tokens';

export type Palette = {
  bg: string; panel: string; panel2: string; elev: string; elev2: string;
  line: string; line2: string;
  text: string; muted: string; dim: string;
  brand: string; accent: string; accent2: string;
  green: string; amber: string; red: string; redbright: string; cyan: string; purple: string;
  greenTx: string; greenTx2: string; amberTx: string; cyanTx: string;
  purpleTx: string; redTx: string; fatalTx: string;
  bodyBg: string; deviceGlow: string;
  topbarA: string; topbarB: string;
  joyA: string; joyB: string; joyC: string;
  knobA: string; knobB: string; knobLine: string;
  mbtnA: string; mbtnB: string; mbtnActA: string; mbtnActB: string;
  sheetA: string; sheetB: string; modalA: string; modalB: string;
  cardA: string; cardB: string; scrollbar: string; fade: string;
  scrim: string;
  glass: string; glassLine: string; glassHi: string;
  thumb: string; onAccent: string;
  dangerA: string; dangerB: string; dangerLine: string;
  legacyBarBg: string; legacyPanelBg: string; legacyAccent: string;
  legacyGaitBg: string; legacyOk: string; legacyWarn: string;
  legacyBtnBg: string; legacyIcon: string;
  legacyWinBg: string; legacyFieldBg: string;
  legacyJoyA: string; legacyJoyB: string; legacyJoyC: string;
  legacyKnobA: string; legacyKnobB: string; legacyKnobLine: string;
  legacyHorizonSky: string; legacyHorizonGround: string; legacyHorizonLine: string;
};

function fromRb(name: RbThemeName): Omit<Palette, LegacyKey> {
  const rb = rbColors[name];
  const dark = name === 'dark';
  return {
    bg: rb['bg-default'], panel: dark ? rb['bg-alt-1'] : rb['bg-card'], panel2: dark ? rb['bg-default'] : rb['bg-raised'],
    elev: dark ? rb['bg-alt-2'] : rb['bg-alt-1'], elev2: dark ? rb['bg-raised'] : rb['bg-alt-2'],
    line: rb['border-subtle'], line2: rb['border-subtler'],
    text: rb['fg-default'], muted: rb['fg-subtle'], dim: rb['fg-subtlest'],
    brand: rb['fill-rb'], accent: rb['fill-brand'], accent2: rb['fill-information'],
    green: rb['fill-success'], amber: rb['fill-warning'], red: rb['fill-danger'], redbright: rb['fg-danger'], cyan: rb['fill-information'], purple: rb['fill-debug'],
    greenTx: rb['fg-success'], greenTx2: rb['fg-success'], amberTx: rb['fg-warning'], cyanTx: rb['fg-information'],
    purpleTx: rb['fg-debug'], redTx: rb['fg-danger'], fatalTx: rb['fg-danger'],
    bodyBg: rb['bg-default'], deviceGlow: dark ? rb['bg-alt-1'] : rb['bg-card'],
    topbarA: dark ? rb['bg-alt-1'] : rb['bg-card'], topbarB: dark ? rb['bg-default'] : rb['bg-raised'],
    joyA: dark ? rb['bg-alt-2'] : rb['bg-raised'], joyB: dark ? rb['bg-alt-1'] : rb['bg-alt-1'], joyC: dark ? rb['bg-default'] : rb['bg-alt-2'],
    knobA: dark ? rb['bg-raised'] : rb['bg-card'], knobB: dark ? rb['bg-alt-2'] : rb['bg-raised'], knobLine: rb['border-default'],
    mbtnA: dark ? rb['bg-alt-2'] : rb['bg-card'], mbtnB: dark ? rb['bg-alt-1'] : rb['bg-raised'], mbtnActA: rb['fill-brand-subtler'], mbtnActB: rb['fill-brand-subtlest'],
    sheetA: dark ? rb['bg-alt-1'] : rb['bg-card'], sheetB: dark ? rb['bg-alt-1'] : rb['bg-card'],
    modalA: dark ? rb['bg-alt-1'] : rb['bg-card'], modalB: dark ? rb['bg-alt-1'] : rb['bg-card'],
    cardA: dark ? rb['bg-alt-1'] : rb['bg-card'], cardB: dark ? rb['bg-alt-1'] : rb['bg-card'],
    scrollbar: rb['fill-subtler'], fade: dark ? rb['bg-alt-1'] : rb['bg-raised'],
    scrim: rb['bg-dim'],
    glass: rb['bg-translucent'], glassLine: rb['border-subtle'], glassHi: dark ? rb['bg-alt-1'] : rb['bg-card'],
    thumb: rb['fg-static-white'], onAccent: rb['fg-static-white'],
    dangerA: rb['fill-danger'], dangerB: rb['fill-danger-hovered'], dangerLine: rb['border-danger-stronger'],
  };
}

type LegacyKey = {
  [K in keyof Palette]: K extends `legacy${string}` ? K : never;
}[keyof Palette];

const LEGACY: Record<ThemeName, Pick<Palette, LegacyKey>> = {
  dark: {
    legacyBarBg: '#1B1D21', legacyPanelBg: '#34383F', legacyAccent: '#FFA500',
    legacyGaitBg: '#1B1D21', legacyOk: '#5FD07A', legacyWarn: '#E5C55A',
    legacyBtnBg: '#3B4048', legacyIcon: '#E8EAED',
    legacyWinBg: '#26282C', legacyFieldBg: '#2E3136',
    legacyJoyA: '#3A3E45', legacyJoyB: '#2C2F35', legacyJoyC: '#232529',
    legacyKnobA: '#4A4F57', legacyKnobB: '#33373D', legacyKnobLine: '#5A616B',
    legacyHorizonSky: '#3E5D82', legacyHorizonGround: '#5C4630', legacyHorizonLine: '#C8CCD2',
  },
  light: {
    legacyBarBg: '#808080', legacyPanelBg: '#C6C6C6', legacyAccent: '#FFA500',
    legacyGaitBg: '#808080', legacyOk: '#90EE90', legacyWarn: '#FFFF00',
    legacyBtnBg: '#FFFFFF', legacyIcon: '#000000',
    legacyWinBg: '#A8A8A8', legacyFieldBg: '#ADADAD',
    legacyJoyA: '#E9E9E9', legacyJoyB: '#CFCFCF', legacyJoyC: '#B2B2B2',
    legacyKnobA: '#FFFFFF', legacyKnobB: '#D8D8D8', legacyKnobLine: '#9A9A9A',
    legacyHorizonSky: '#5A8DC8', legacyHorizonGround: '#8A6A43', legacyHorizonLine: '#FFFFFF',
  },
};

export const Themes: { dark: Palette; light: Palette } = {
  dark: { ...fromRb('dark'), ...LEGACY.dark },
  light: { ...fromRb('light'), ...LEGACY.light },
};

export type ThemeName = RbThemeName;

export const Radius = { sm: rounded.lg, md: rounded.xl, lg: rounded['2xl'] } as const;

export const Spacing = {
  half: 2, one: 4, two: 8, three: 16, four: 24, five: 32, six: 64,
} as const;

export const Fonts = Platform.select({
  ios: { ui: 'system-ui', mono: 'ui-monospace' },
  android: { ui: 'sans-serif', mono: 'monospace' },
  default: { ui: 'System', mono: 'monospace' },
  web: { ui: 'var(--font-display, system-ui)', mono: 'var(--font-mono, monospace)' },
})!;
