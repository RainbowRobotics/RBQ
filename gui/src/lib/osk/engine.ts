import { push, backspace, syllable, isJamo, type Composing } from '@/lib/osk/hangul';

export type Field = { value: string; caret: number };
export type Ime = Composing;

export type KeyAction =
  | { type: 'char'; char: string }
  | { type: 'backspace' }
  | { type: 'left' }
  | { type: 'right' };

const clamp = (n: number, max: number) => Math.max(0, Math.min(n, max));

export function apply(f: Field, ime: Ime, key: KeyAction): { field: Field; ime: Ime } {
  const caret = clamp(f.caret, f.value.length);
  const comp = syllable(ime);
  const base = comp ? f.value.slice(0, caret - comp.length) : f.value.slice(0, caret);
  const tail = f.value.slice(caret);

  switch (key.type) {
    case 'char': {
      if (isJamo(key.char)) {
        const { done, next } = push(ime, key.char);
        const head = base + done + syllable(next);
        return { field: { value: head + tail, caret: head.length }, ime: next };
      }
      const head = base + comp + key.char;
      return { field: { value: head + tail, caret: head.length }, ime: null };
    }
    case 'backspace': {
      if (ime) {
        const { next, removed } = backspace(ime);
        const head = removed ? base : base + syllable(next);
        return { field: { value: head + tail, caret: head.length }, ime: removed ? null : next };
      }
      const head = base.slice(0, Math.max(0, base.length - 1));
      return { field: { value: head + tail, caret: head.length }, ime: null };
    }
    case 'left':
      return { field: { value: f.value, caret: clamp(caret - 1, f.value.length) }, ime: null };
    case 'right':
      return { field: { value: f.value, caret: clamp(caret + 1, f.value.length) }, ime: null };
  }
}
