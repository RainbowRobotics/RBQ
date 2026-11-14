
const BIG = 1e18;

function dt1d(f: Float64Array, n: number): Float64Array {
  const d = new Float64Array(n);
  const v = new Int32Array(n);
  const z = new Float64Array(n + 1);
  let k = 0;
  v[0] = 0;
  z[0] = -Infinity;
  z[1] = Infinity;
  for (let q = 1; q < n; q++) {
    let s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    while (s <= z[k]) {
      k--;
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2 * q - 2 * v[k]);
    }
    k++;
    v[k] = q;
    z[k] = s;
    z[k + 1] = Infinity;
  }
  k = 0;
  for (let q = 0; q < n; q++) {
    while (z[k + 1] < q) k++;
    d[q] = (q - v[k]) * (q - v[k]) + f[v[k]];
  }
  return d;
}

function squaredDistToOccupied(cells: Uint8Array, rows: number, cols: number): Float64Array {
  const g = new Float64Array(rows * cols);
  const colBuf = new Float64Array(rows);
  for (let x = 0; x < cols; x++) {
    for (let y = 0; y < rows; y++) colBuf[y] = cells[y * cols + x] ? 0 : BIG;
    const d = dt1d(colBuf, rows);
    for (let y = 0; y < rows; y++) g[y * cols + x] = d[y];
  }
  const out = new Float64Array(rows * cols);
  const rowBuf = new Float64Array(cols);
  for (let y = 0; y < rows; y++) {
    const rowOff = y * cols;
    for (let x = 0; x < cols; x++) rowBuf[x] = g[rowOff + x];
    const d = dt1d(rowBuf, cols);
    for (let x = 0; x < cols; x++) out[rowOff + x] = d[x];
  }
  return out;
}

export function inflateWallMap(
  cells: Uint8Array, rows: number, cols: number, gs: number, marginM: number,
): Uint8Array {
  if (marginM <= 0 || gs <= 0) return cells;
  const r = Math.max(1, Math.ceil(marginM / gs));
  const r2 = r * r;
  const sq = squaredDistToOccupied(cells, rows, cols);
  const out = new Uint8Array(rows * cols);
  for (let i = 0; i < out.length; i++) out[i] = sq[i] <= r2 ? 255 : 0;
  return out;
}
