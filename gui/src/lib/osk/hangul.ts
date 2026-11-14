
const CHO = ['ㄱ', 'ㄲ', 'ㄴ', 'ㄷ', 'ㄸ', 'ㄹ', 'ㅁ', 'ㅂ', 'ㅃ', 'ㅅ', 'ㅆ', 'ㅇ', 'ㅈ', 'ㅉ', 'ㅊ', 'ㅋ', 'ㅌ', 'ㅍ', 'ㅎ'];
const JUNG = ['ㅏ', 'ㅐ', 'ㅑ', 'ㅒ', 'ㅓ', 'ㅔ', 'ㅕ', 'ㅖ', 'ㅗ', 'ㅘ', 'ㅙ', 'ㅚ', 'ㅛ', 'ㅜ', 'ㅝ', 'ㅞ', 'ㅟ', 'ㅠ', 'ㅡ', 'ㅢ', 'ㅣ'];
const JONG = ['', 'ㄱ', 'ㄲ', 'ㄳ', 'ㄴ', 'ㄵ', 'ㄶ', 'ㄷ', 'ㄹ', 'ㄺ', 'ㄻ', 'ㄼ', 'ㄽ', 'ㄾ', 'ㄿ', 'ㅀ', 'ㅁ', 'ㅂ', 'ㅄ', 'ㅅ', 'ㅆ', 'ㅇ', 'ㅈ', 'ㅊ', 'ㅋ', 'ㅌ', 'ㅍ', 'ㅎ'];

const JUNG_PAIR: Record<string, string> = {
  'ㅗㅏ': 'ㅘ', 'ㅗㅐ': 'ㅙ', 'ㅗㅣ': 'ㅚ',
  'ㅜㅓ': 'ㅝ', 'ㅜㅔ': 'ㅞ', 'ㅜㅣ': 'ㅟ',
  'ㅡㅣ': 'ㅢ',
};
const JONG_PAIR: Record<string, string> = {
  'ㄱㅅ': 'ㄳ', 'ㄴㅈ': 'ㄵ', 'ㄴㅎ': 'ㄶ', 'ㄹㄱ': 'ㄺ', 'ㄹㅁ': 'ㄻ', 'ㄹㅂ': 'ㄼ',
  'ㄹㅅ': 'ㄽ', 'ㄹㅌ': 'ㄾ', 'ㄹㅍ': 'ㄿ', 'ㄹㅎ': 'ㅀ', 'ㅂㅅ': 'ㅄ',
};
const JONG_SPLIT: Record<string, [string, string]> = Object.fromEntries(
  Object.entries(JONG_PAIR).map(([two, one]) => [one, [two[0], two[1]] as [string, string]]),
);

export type Composing = { cho: string; jung: string; jong: string } | null;

export const isJamo = (ch: string) => CHO.includes(ch) || JUNG.includes(ch);
const isCons = (ch: string) => CHO.includes(ch);
const isVowel = (ch: string) => JUNG.includes(ch);

export function syllable(c: Composing): string {
  if (!c) return '';
  if (!c.jung) return c.cho;
  if (!c.cho) return c.jung;
  const ci = CHO.indexOf(c.cho), ji = JUNG.indexOf(c.jung), ti = JONG.indexOf(c.jong || '');
  if (ci < 0 || ji < 0 || ti < 0) return c.cho + c.jung + c.jong;
  return String.fromCharCode(0xac00 + (ci * 21 + ji) * 28 + ti);
}

export function push(c: Composing, jamo: string): { done: string; next: Composing } {
  if (!isJamo(jamo)) return { done: syllable(c) + jamo, next: null };

  if (!c) {
    return isCons(jamo) ? { done: '', next: { cho: jamo, jung: '', jong: '' } }
                        : { done: '', next: { cho: '', jung: jamo, jong: '' } };
  }

  if (!c.cho) {
    const pair = JUNG_PAIR[c.jung + jamo];
    if (isVowel(jamo) && pair) return { done: '', next: { cho: '', jung: pair, jong: '' } };
    if (isVowel(jamo)) return { done: c.jung, next: { cho: '', jung: jamo, jong: '' } };
    return { done: c.jung, next: { cho: jamo, jung: '', jong: '' } };
  }

  if (!c.jung) {
    if (isVowel(jamo)) return { done: '', next: { cho: c.cho, jung: jamo, jong: '' } };
    return { done: c.cho, next: { cho: jamo, jung: '', jong: '' } };
  }

  if (isVowel(jamo)) {
    if (!c.jong) {
      const pair = JUNG_PAIR[c.jung + jamo];
      if (pair) return { done: '', next: { cho: c.cho, jung: pair, jong: '' } };
      return { done: syllable(c), next: { cho: '', jung: jamo, jong: '' } };
    }
    const split = JONG_SPLIT[c.jong];
    const keep = split ? split[0] : '';
    const move = split ? split[1] : c.jong;
    return {
      done: syllable({ cho: c.cho, jung: c.jung, jong: keep }),
      next: { cho: move, jung: jamo, jong: '' },
    };
  }

  if (!c.jong) {
    if (JONG.includes(jamo)) return { done: '', next: { ...c, jong: jamo } };
    return { done: syllable(c), next: { cho: jamo, jung: '', jong: '' } };
  }
  const pair = JONG_PAIR[c.jong + jamo];
  if (pair) return { done: '', next: { ...c, jong: pair } };
  return { done: syllable(c), next: { cho: jamo, jung: '', jong: '' } };
}

export function backspace(c: Composing): { next: Composing; removed: number } {
  if (!c) return { next: null, removed: 1 };
  if (c.jong) {
    const split = JONG_SPLIT[c.jong];
    return { next: { ...c, jong: split ? split[0] : '' }, removed: 0 };
  }
  if (c.jung) {
    const split = Object.entries(JUNG_PAIR).find(([, one]) => one === c.jung);
    if (split) return { next: { ...c, jung: split[0][0] }, removed: 0 };
    return { next: c.cho ? { ...c, jung: '' } : null, removed: c.cho ? 0 : 1 };
  }
  return { next: null, removed: 1 };
}
