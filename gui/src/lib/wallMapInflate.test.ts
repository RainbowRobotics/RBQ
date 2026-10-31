import { describe, it, expect } from 'vitest';
import { inflateWallMap } from './wallMapInflate';

function grid(rows: number, cols: number, occupied: [number, number][]): Uint8Array {
  const cells = new Uint8Array(rows * cols);
  for (const [y, x] of occupied) cells[y * cols + x] = 255;
  return cells;
}

describe('inflateWallMap', () => {
  it('leaves the grid unchanged when margin or gs is non-positive', () => {
    const cells = grid(5, 5, [[2, 2]]);
    expect(inflateWallMap(cells, 5, 5, 0.02, 0)).toBe(cells);
    expect(inflateWallMap(cells, 5, 5, 0, 0.2)).toBe(cells);
  });

  it('inflates a single occupied cell into a circle of radius ceil(margin/gs), not a square', () => {
    const rows = 11, cols = 11;
    const cx = 5, cy = 5;
    const cells = grid(rows, cols, [[cy, cx]]);
    const out = inflateWallMap(cells, rows, cols, 1, 3);
    const at = (y: number, x: number) => out[y * cols + x];
    expect(at(cy, cx + 3)).toBe(255);
    expect(at(cy + 3, cx)).toBe(255);
    expect(at(cy + 3, cx + 3)).toBe(0);
    expect(at(cy + 2, cx + 2)).toBe(255);
    expect(at(cy + 3, cx + 4)).toBe(0);
  });

  it('matches a brute-force circular dilation for a small irregular grid', () => {
    const rows = 9, cols = 9, r = 2;
    const cells = grid(rows, cols, [[1, 1], [4, 6], [7, 2]]);
    const out = inflateWallMap(cells, rows, cols, 1, r);
    const expected = new Uint8Array(rows * cols);
    for (let y = 0; y < rows; y++) {
      for (let x = 0; x < cols; x++) {
        let inside = false;
        for (let oy = 0; oy < rows && !inside; oy++) {
          for (let ox = 0; ox < cols; ox++) {
            if (!cells[oy * cols + ox]) continue;
            const dy = y - oy, dx = x - ox;
            if (dx * dx + dy * dy <= r * r) { inside = true; break; }
          }
        }
        expected[y * cols + x] = inside ? 255 : 0;
      }
    }
    expect(out).toEqual(expected);
  });

  it('r is at least 1 even for a tiny margin relative to gs', () => {
    const rows = 5, cols = 5;
    const cells = grid(rows, cols, [[2, 2]]);
    const out = inflateWallMap(cells, rows, cols, 1.0, 0.01);
    expect(out[2 * cols + 1]).toBe(255);
    expect(out[2 * cols + 3]).toBe(255);
  });
});
