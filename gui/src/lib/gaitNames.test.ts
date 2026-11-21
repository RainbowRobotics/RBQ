import { describe, it, expect } from 'vitest';
import { existsSync, readFileSync } from 'fs';
import { resolve } from 'path';
import { GAIT_NAMES, gaitIdName } from './gaitNames';

describe('gaitIdName', () => {
  const enums = resolve(__dirname, '../../../src/rcl/Enums.h');
  it.skipIf(!existsSync(enums))('matches every GAIT_STATE entry in rcl/Enums.h', () => {
    const src = readFileSync(enums, 'utf8');
    const body = src.slice(src.indexOf('enum GAIT_STATE'), src.indexOf('RL_END'));
    const pairs = [...body.matchAll(/^\s+([A-Z_0-9]+)\s+=\s+(-?\d+)/gm)].map((m) => [Number(m[2]), m[1]] as const);
    expect(pairs.length).toBeGreaterThan(30);
    for (const [id, name] of pairs) expect(GAIT_NAMES[id], `${id}`).toBe(name);
  });
  it('falls back to #id', () => {
    expect(gaitIdName(48)).toBe('RL_WALK_VISION');
    expect(gaitIdName(77)).toBe('#77');
  });
});
