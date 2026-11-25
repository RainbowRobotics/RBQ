import { describe, expect, it } from 'vitest';
import { decodeElevationZ } from './elevationMap';

const FIXTURE =
  'RVoBBwAAAAkAAADNzEw9AACQwAAAEEBvEoM6MGDBgwYMGDBgwYK//edHcfHE6nwJnAJtT2ebM6czgE2TmebU9OZwAATqJj1PTuxU8nJzPNqeTpszwBAjPNidT1szwA==';

function bytes(b64: string): ArrayBuffer {
  const b = Buffer.from(b64, 'base64');
  return b.buffer.slice(b.byteOffset, b.byteOffset + b.byteLength) as ArrayBuffer;
}

describe('decodeElevationZ (EZ1)', () => {
  it('C++ 인코더가 만든 격자를 그대로 푼다 — 유효 표시 같고 높이는 1 mm 의 절반 안', () => {
    const g = decodeElevationZ(bytes(FIXTURE));
    expect(g).not.toBeNull();
    if (!g) return;
    expect([g.rows, g.cols, g.originX, g.originY]).toEqual([7, 9, -4.5, 2.25]);
    expect(g.gs).toBeCloseTo(0.05, 6);
    for (let r = 0; r < 7; r++) {
      for (let c = 0; c < 9; c++) {
        const i = r * 9 + c;
        const valid = i % 5 !== 0;
        expect(g.valid[i]).toBe(valid ? 1 : 0);
        if (!valid) continue;
        const h = i === 40 ? 1234.5678 : (((r * 7 + c * 3) % 11) - 5) * 0.013 + (r > 3 ? 0.15 : 0);
        expect(Math.abs(g.height[i] - h)).toBeLessThanOrEqual(0.0005 + Math.abs(h) * 2.4e-7);
      }
    }
  });

  it('잘렸거나 머리가 틀린 것은 null', () => {
    const full = new Uint8Array(bytes(FIXTURE));
    for (let n = 0; n < full.length; n++) expect(decodeElevationZ(full.slice(0, n).buffer)).toBeNull();
    const wrong = full.slice();
    wrong[0] = 0x58;
    expect(decodeElevationZ(wrong.buffer)).toBeNull();
    const ver = full.slice();
    ver[2] = 2;
    expect(decodeElevationZ(ver.buffer)).toBeNull();
  });
});
