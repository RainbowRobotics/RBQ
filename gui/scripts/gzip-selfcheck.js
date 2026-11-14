const pako = require('pako'), zlib = require('zlib'), assert = require('assert');
const CHUNK = 3 * 512 * 1024;
const src = Buffer.from(Array.from({length: 4_000_000}, (_, i) =>
  `{"timestamp":"12:00:${i%60}","application":"Motion","level":"INFO","message":"row ${i}"}\n`).join('').slice(0, 4_000_000));

for (const total of [0, 10, CHUNK, src.length]) {
  const buf = src.subarray(0, total);
  const def = new pako.Deflate({ gzip: true, level: 6 });
  const parts = []; def.onData = (c) => parts.push(c);
  if (total === 0) def.push(new Uint8Array(0), true);
  else for (let pos = 0; pos < total; pos += CHUNK) {
    const len = Math.min(CHUNK, total - pos);
    const b64 = buf.subarray(pos, pos + len).toString('base64');
    def.push(new Uint8Array(Buffer.from(b64, 'base64')), pos + len >= total);
  }
  assert(!def.err, `deflate err: ${def.msg}`);
  const gz = Buffer.concat(parts.map(Buffer.from));
  assert.deepStrictEqual(zlib.gunzipSync(gz), buf, `왕복 불일치 total=${total}`);
  console.log(`ok total=${total} → gzip ${gz.length}B (${total ? (gz.length/total*100).toFixed(1) : '-'}%)`);
}
console.log('PASS');
