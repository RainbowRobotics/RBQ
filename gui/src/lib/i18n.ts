import { getLang } from '@/store/lang';
import { EN } from './i18n.en';

export function t(ko: string): string {
  if (getLang() !== 'en') return ko;
  return EN[ko] ?? ko;
}
