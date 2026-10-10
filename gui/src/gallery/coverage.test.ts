import { describe, it, expect } from 'vitest';
import { readFileSync, readdirSync } from 'node:fs';
import { join } from 'node:path';

const SRC = join(__dirname, '..');

const SHARED = [
  ...readdirSync(join(SRC, 'components/ui')).map((f) => join('components/ui', f)),
  'components/anim.tsx',
  'components/panels/settings/common.tsx',
];


const NOT_A_WIDGET = new Set(['inputVFix', 'MODAL_ORIENTATIONS', 'compactBodyStyle', 'S', 'confirmSkipped', 'LegibleText']);

const exportsOf = (rel: string) =>
  [...readFileSync(join(SRC, rel), 'utf8').matchAll(/^export (?:function|const) (\w+)/gm)]
    .map((m) => m[1])
    .filter((n) => /^[A-Z]/.test(n) && !NOT_A_WIDGET.has(n));

const registered = readdirSync(join(__dirname, 'cases'))
  .flatMap((f) => [...readFileSync(join(__dirname, 'cases', f), 'utf8').matchAll(/name: '([^']+)'/g)])
  .flatMap((m) => m[1].split('/').map((s) => s.trim()));

describe('위젯 갤러리 커버리지', () => {
  it.each(SHARED)('%s 의 공용 위젯이 전부 갤러리에 있다', (rel) => {
    const missing = exportsOf(rel).filter((n) => !registered.includes(n));
    expect(missing, `갤러리 미등록: ${missing.join(', ')} — src/gallery/cases/ 에 케이스를 추가하거나 NOT_A_WIDGET 에 사유와 함께 넣을 것`).toEqual([]);
  });

  it('등록된 이름에 중복이 없다', () => {
    const dup = registered.filter((n, i) => registered.indexOf(n) !== i);
    expect(dup).toEqual([]);
  });

  it('모든 케이스에 복붙용 code 가 있다 — 여기가 다른 사람·AI 가 베끼는 레퍼런스다', () => {
    const missing: string[] = [];
    for (const f of readdirSync(join(__dirname, 'cases'))) {
      const src = readFileSync(join(__dirname, 'cases', f), 'utf8');
      for (const b of src.split(/\n  \{/).slice(1)) {
        const name = b.match(/name: '([^']+)'/)?.[1];
        if (!name) continue;
        if (!/code: `/.test(b)) missing.push(`${f}:${name}`);
      }
    }
    expect(missing, `code 없는 케이스: ${missing.join(', ')}`).toEqual([]);
  });

});

describe('갤러리 전역 사용 금지', () => {
  for (const f of readdirSync(join(__dirname, 'cases'))) {
    const src = readFileSync(join(__dirname, 'cases', f), 'utf8');
    it(`${f} — 앱 스토어·싱글턴을 import 하지 않는다`, () => {
      const bad = [
        ...src.matchAll(/^import .* from '(@\/store\/[^']+)'/gm),
        ...src.matchAll(/^import \{[^}]*\buse\w*Store\b[^}]*\} from '([^']+)'/gm),
      ].map((m) => m[1]);
      expect(bad,
        `${f} 가 전역 상태를 import 한다: ${bad.join(', ')}. 전시하려고 덮으면 그 값이 앱 전역에 ` +
        '남는다 — Demo 를 빼고 unavailable 로 코드만 전시할 것(gallery/cases/live.tsx 참조).',
      ).toEqual([]);
    });
  }
});
