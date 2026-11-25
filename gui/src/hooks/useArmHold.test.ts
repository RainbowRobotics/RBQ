import { describe, it, expect } from 'vitest';
import { readFileSync } from 'node:fs';
import { join } from 'node:path';

const src = readFileSync(join(__dirname, 'useArmHold.ts'), 'utf8');

describe('useArmHold platform gate', () => {
  it('does not use typeof window/document as a platform check', () => {
    expect(src).not.toMatch(/typeof\s+(window|document)\s*!==?\s*['"]undefined['"]/);
  });
  it('keeps every DOM listener call on a Platform.OS === web line', () => {
    const lines = src.split('\n').filter((l) => /\b(window|document)\.(add|remove)EventListener\(/.test(l));
    expect(lines.length).toBeGreaterThan(0);
    for (const line of lines) expect(line).toMatch(/Platform\.OS\s*===\s*'web'/);
  });
});
