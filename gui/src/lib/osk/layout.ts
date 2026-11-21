export type KeyDef =
  | { k: 'char'; lo: string; hi?: string; w?: number }
  | { k: 'shift' | 'back' | 'space' | 'lang' | 'sym' | 'left' | 'right' | 'done'; w?: number };

export type Mode = 'ko' | 'en' | 'sym';

const chars = (s: string, hi?: string): KeyDef[] =>
  [...s].map((c, i) => ({ k: 'char', lo: c, hi: hi ? [...hi][i] : undefined }));

const KO: KeyDef[][] = [
  chars('ㅂㅈㄷㄱㅅㅛㅕㅑㅐㅔ', 'ㅃㅉㄸㄲㅆㅛㅕㅑㅒㅖ'),
  chars('ㅁㄴㅇㄹㅎㅗㅓㅏㅣ'),
  [{ k: 'shift', w: 1.5 }, ...chars('ㅋㅌㅊㅍㅠㅜㅡ'), { k: 'back', w: 1.5 }],
];

const EN: KeyDef[][] = [
  chars('qwertyuiop', 'QWERTYUIOP'),
  chars('asdfghjkl', 'ASDFGHJKL'),
  [{ k: 'shift', w: 1.5 }, ...chars('zxcvbnm', 'ZXCVBNM'), { k: 'back', w: 1.5 }],
];

const SYM: KeyDef[][] = [
  chars('1234567890'),
  chars('.:/-_@#*+='),
  [{ k: 'shift', w: 1.5 }, ...chars('!?,%()\'"'), { k: 'back', w: 1.5 }],
];

const SYM_HI: KeyDef[][] = [
  chars('~`|\\^&$₩€£¥'),
  chars('[]{}<>;·※…'),
  [{ k: 'shift', w: 1.5 }, ...chars('①②③°㎜㎝㎞'), { k: 'back', w: 1.5 }],
];

const BOTTOM: KeyDef[][] = [[
  { k: 'sym', w: 1.4 }, { k: 'lang', w: 1.4 }, { k: 'left' }, { k: 'space', w: 4 }, { k: 'right' }, { k: 'done', w: 1.6 },
]];

export function rows(mode: Mode, shift: boolean): KeyDef[][] {
  const main = mode === 'ko' ? KO : mode === 'en' ? EN : shift ? SYM_HI : SYM;
  return [...main, ...BOTTOM];
}

export function charOf(def: Extract<KeyDef, { k: 'char' }>, shift: boolean): string {
  return shift && def.hi ? def.hi : def.lo;
}

export function labelOf(def: KeyDef, mode: Mode, shift: boolean, letters: 'ko' | 'en'): string {
  switch (def.k) {
    case 'char': return charOf(def, shift);
    case 'shift': return '⇧';
    case 'back': return '⌫';
    case 'space': return '␣';
    case 'lang': return letters === 'ko' ? 'A' : '한';
    case 'sym': return mode === 'sym' ? (letters === 'ko' ? '한글' : 'ABC') : '123';
    case 'left': return '◀';
    case 'right': return '▶';
    case 'done': return '✓';
  }
}
