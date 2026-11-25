import { describe, it, expect } from 'vitest';
import { minMaxBuckets, niceRange, presetCharts, jointLabel } from './bbSeries';
import type { BlackboxSession } from './blackbox';

function sess(vals: number[]): BlackboxSession {
  return {
    tickMs: 10, frameCount: vals.length, startEpochMs: 0, cols: new Map([['x', 0]]), numCols: 1,
    data: Float32Array.from(vals), logs: [], sync: { warn: false, reason: '' }, video: {},
  };
}

describe('minMaxBuckets', () => {
  it('keeps a one-frame spike inside its bucket', () => {
    const v = new Array(1000).fill(0); v[537] = 9;
    const mm = minMaxBuckets(sess(v), 'x', 0, 999, 10)!;
    expect(Math.max(...mm.max)).toBe(9);
    expect(mm.max[5]).toBe(9);
    expect(Math.min(...mm.min)).toBe(0);
  });
  it('fills every bucket in time order when buckets > frames (frames repeat as steps)', () => {
    const mm = minMaxBuckets(sess([1, 2, 3]), 'x', 0, 2, 8)!;
    const v = [...mm.max];
    expect(v.some(Number.isNaN)).toBe(false);
    expect([...new Set(v)]).toEqual([1, 2, 3]);
  });
  it('returns null for a missing channel and applies scale', () => {
    expect(minMaxBuckets(sess([1]), 'nope', 0, 0, 4)).toBeNull();
    expect(minMaxBuckets(sess([2, 2]), 'x', 0, 1, 1, 10)!.max[0]).toBe(20);
  });
});

describe('niceRange', () => {
  it('spans the data with 1-2-5 steps', () => {
    const r = niceRange(-13, 27);
    expect(r.lo).toBeLessThanOrEqual(-13);
    expect(r.hi).toBeGreaterThanOrEqual(27);
    expect(r.ticks).toContain(0);
  });
  it('widens a flat line', () => {
    const r = niceRange(5, 5);
    expect(r.hi).toBeGreaterThan(r.lo);
  });
});

describe('presets', () => {
  it('maps legs to recorder joint order (HR 0-2, HL 3-5, FR 6-8, FL 9-11)', () => {
    expect(jointLabel(0)).toBe('HRR0');
    expect(jointLabel(11)).toBe('FLK11');
    const fl = presetCharts('torque', 'FL', []);
    expect(fl.map((c) => c.lines[0].name)).toEqual(['joint.torque[9]', 'joint.torque[10]', 'joint.torque[11]']);
    expect(fl[0].lines[1].name).toBe('ref.joint.torque[9]');
  });
  it('preset channels all exist in the recorder schema', async () => {
    const { demoBlackboxChannels } = await import('./blackboxSchema');
    const have = new Set(demoBlackboxChannels());
    for (const key of ['overview', 'torque', 'current', 'pos', 'temp', 'imu', 'cpu', 'power'] as const) {
      for (const leg of ['ALL', 'FL', 'FR', 'HL', 'HR'] as const) {
        for (const ch of presetCharts(key, leg, [])) for (const ln of ch.lines) expect(have.has(ln.name), ln.name).toBe(true);
      }
    }
  });
  it('ALL lays out four legs front-to-back (rows) by R·P·K (columns)', () => {
    const all = presetCharts('torque', 'ALL', []);
    expect(all.map((c) => c.title)).toEqual([
      'FLR9', 'FLP10', 'FLK11', 'FRR6', 'FRP7', 'FRK8', 'HLR3', 'HLP4', 'HLK5', 'HRR0', 'HRP1', 'HRK2']);
    const temp = presetCharts('temp', 'ALL', []);
    expect(temp).toHaveLength(4);
    expect(temp[0].lines).toHaveLength(6);
  });
});
