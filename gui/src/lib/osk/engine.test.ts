import { describe, it, expect } from 'vitest';
import { apply, type Field, type Ime } from './engine';
import { push, backspace, syllable } from './hangul';

function type(keys: string[], start: Field = { value: '', caret: 0 }) {
  let f = start, ime: Ime = null;
  for (const k of keys) {
    const r: { field: Field; ime: Ime } = k === '⌫' ? apply(f, ime, { type: 'backspace' })
      : k === '◀' ? apply(f, ime, { type: 'left' })
      : k === '▶' ? apply(f, ime, { type: 'right' })
      : apply(f, ime, { type: 'char', char: k });
    f = r.field; ime = r.ime;
  }
  return f;
}

describe('두벌식 조합', () => {
  it('초성·중성·종성을 합친다', () => {
    expect(type(['ㄱ']).value).toBe('ㄱ');
    expect(type(['ㄱ', 'ㅏ']).value).toBe('가');
    expect(type(['ㄱ', 'ㅏ', 'ㅁ']).value).toBe('감');
    expect(type(['ㅎ', 'ㅏ', 'ㄴ', 'ㄱ', 'ㅡ', 'ㄹ']).value).toBe('한글');
  });

  it('겹모음·겹받침을 만든다', () => {
    expect(type(['ㄱ', 'ㅗ', 'ㅏ']).value).toBe('과');
    expect(type(['ㄱ', 'ㅏ', 'ㅂ', 'ㅅ']).value).toBe('값');
    expect(type(['ㅇ', 'ㅢ']).value).toBe('의');
  });

  it('종성 뒤에 모음이 오면 다음 글자 초성으로 넘긴다', () => {
    expect(type(['ㄱ', 'ㅏ', 'ㄹ', 'ㅏ']).value).toBe('가라');
    expect(type(['ㅇ', 'ㅏ', 'ㄴ', 'ㅈ', 'ㅏ']).value).toBe('안자');
  });

  it('지우기는 자모 하나씩 되돌린다', () => {
    expect(type(['ㄱ', 'ㅏ', 'ㅂ', 'ㅅ', '⌫']).value).toBe('갑');
    expect(type(['ㄱ', 'ㅏ', 'ㅂ', 'ㅅ', '⌫', '⌫']).value).toBe('가');
    expect(type(['ㄱ', 'ㅏ', 'ㅂ', 'ㅅ', '⌫', '⌫', '⌫']).value).toBe('ㄱ');
    expect(type(['ㄱ', 'ㅏ', 'ㅂ', 'ㅅ', '⌫', '⌫', '⌫', '⌫']).value).toBe('');
    expect(type(['ㄱ', 'ㅗ', 'ㅏ', '⌫']).value).toBe('고');
  });

  it('자음을 연달아 치면 앞 글자가 확정된다', () => {
    expect(type(['ㄱ', 'ㄴ']).value).toBe('ㄱㄴ');
    expect(type(['ㄱ', 'ㅏ', 'ㄴ', 'ㄴ', 'ㅏ']).value).toBe('간나');
  });
});

describe('앞에 글자가 있을 때 지우기', () => {
  it('조합 중 마지막 자모를 지워도 앞 글자는 남는다', () => {
    const at = (v: string) => ({ value: v, caret: v.length });
    expect(type(['ㄷ', 'ㅏ', '⌫', '⌫'], at('가나')).value).toBe('가나');
    expect(type(['ㄷ', 'ㅏ', '⌫', '⌫', '⌫'], at('가나')).value).toBe('가');
    expect(type(['ㄱ', '⌫'], at('로봇')).value).toBe('로봇');
    expect(type(['ㄴ', '⌫'], at('가 ')).value).toBe('가 ');
  });
});

describe('영문·숫자·기호', () => {
  it('그대로 들어가고 조합을 끊는다', () => {
    expect(type(['R', 'B', 'Q', '1', '0']).value).toBe('RBQ10');
    expect(type(['1', '9', '2', '.', '1', '6', '8', '.', '0', '.', '1', '0']).value).toBe('192.168.0.10');
    expect(type(['ㄱ', 'ㅏ', 'A']).value).toBe('가A');
  });

  it('공백·지우기가 섞여도 값이 어긋나지 않는다', () => {
    expect(type(['ㄱ', 'ㅏ', ' ', 'ㄴ', 'ㅏ']).value).toBe('가 나');
    expect(type(['a', 'b', '⌫', 'c']).value).toBe('ac');
  });
});

describe('캐럿', () => {
  it('중간에 끼워 넣는다 — 조합은 캐럿을 옮기는 순간 끊긴다', () => {
    const f = type(['ㄱ', 'ㅏ', '◀', 'ㄴ', 'ㅏ']);
    expect(f.value).toBe('나가');
    expect(f.caret).toBe(1);
  });

  it('기존 값 가운데에서 지운다', () => {
    const f = type(['⌫'], { value: 'abcd', caret: 3 });
    expect(f).toEqual({ value: 'abd', caret: 2 });
  });

  it('맨 앞에서 지우면 아무 일도 없다', () => {
    expect(type(['⌫'], { value: 'ab', caret: 0 })).toEqual({ value: 'ab', caret: 0 });
  });
});

describe('조합기 자체', () => {
  it('미완성 음절도 화면에 보여 준다', () => {
    expect(syllable(push(null, 'ㄱ').next)).toBe('ㄱ');
    expect(syllable(push(push(null, 'ㄱ').next, 'ㅏ').next)).toBe('가');
  });

  it('조합 중이 아니면 지우기는 글자 하나를 지운다', () => {
    expect(backspace(null)).toEqual({ next: null, removed: 1 });
  });
});
