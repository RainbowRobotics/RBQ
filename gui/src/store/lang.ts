import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import AsyncStorage from '@react-native-async-storage/async-storage';

export type Lang = 'ko' | 'en';

const QUERY_LANG: Lang | null = (() => {
  if (typeof location === 'undefined' || typeof location.search !== 'string') return null;
  const q = new URLSearchParams(location.search).get('lang');
  return q === 'en' || q === 'ko' ? q : null;
})();
const NO_STORAGE = { getItem: async () => null, setItem: async () => {}, removeItem: async () => {} };

type LangState = {
  lang: Lang;
  setLang: (l: Lang) => void;
  toggle: () => void;
};

export const useLang = create<LangState>()(
  persist(
    (set, get) => ({
      lang: QUERY_LANG ?? 'ko',
      setLang: (lang) => set({ lang }),
      toggle: () => set({ lang: get().lang === 'ko' ? 'en' : 'ko' }),
    }),
    {
      name: 'rbq-lang',
      storage: createJSONStorage(() => (QUERY_LANG ? NO_STORAGE : AsyncStorage)),
      onRehydrateStorage: () => (s) => { if (s?.lang) _lang = s.lang; },
    },
  ),
);

let _lang: Lang = QUERY_LANG ?? 'ko';
export function getLang(): Lang { return _lang; }
useLang.subscribe((s) => { _lang = s.lang; });
