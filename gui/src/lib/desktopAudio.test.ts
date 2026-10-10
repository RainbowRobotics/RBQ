import { describe, it, expect } from 'vitest';
import { agcProcess } from './desktopAudio';

const dbfs = (x: Float32Array) => 20 * Math.log10(Math.sqrt(x.reduce((a, v) => a + v * v, 0) / x.length));
const sine = (amp: number, n = 1024, f = 300) => Float32Array.from({ length: n }, (_, i) => amp * Math.sin((2 * Math.PI * f * i) / 48000));

describe('agcProcess', () => {
  it('작은 말소리(-30 dBFS)를 2 s 안에 -20 dBFS 근처로 키운다', () => {
    const st = { gain: 1 };
    let out: Float32Array = new Float32Array();
    for (let k = 0; k < 94; k++) out = agcProcess(sine(0.0447), st);
    expect(dbfs(out)).toBeGreaterThan(-23);
    expect(dbfs(out)).toBeLessThan(-17);
  });
  it('조용할 때(-60 dBFS)는 게인을 올리지 않는다', () => {
    const st = { gain: 1 };
    for (let k = 0; k < 94; k++) agcProcess(sine(0.001), st);
    expect(st.gain).toBe(1);
  });
  it('큰 소리(0 dBFS)는 찢지 않고 ±1 안에 누른다', () => {
    const st = { gain: 8 };
    const out = agcProcess(sine(1), st);
    expect(Math.max(...out.map(Math.abs))).toBeLessThan(1);
    expect(st.gain).toBeLessThan(8);
  });
});
