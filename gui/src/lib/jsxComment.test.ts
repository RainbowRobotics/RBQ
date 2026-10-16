import { describe, it, expect } from 'vitest';
import { readdirSync, readFileSync } from 'node:fs';

describe('JSX 주석 자리', () => {
  it('요소와 같은 줄에 주석을 붙이지 않는다(공백이 문자열 자식으로 남는다)', () => {
    const files = (readdirSync('src', { recursive: true }) as string[])
      .filter((f) => f.endsWith('.tsx')).map((f) => `src/${f}`);
    expect(files.length).toBeGreaterThan(100);
    const bad: string[] = [];
    for (const f of files) {
      readFileSync(f, 'utf8').split('\n').forEach((ln, i) => {
        if (/>[ \t]+\{\/\*/.test(ln)) bad.push(`${f}:${i + 1}`);
      });
    }
    expect(bad).toEqual([]);
  });
});
