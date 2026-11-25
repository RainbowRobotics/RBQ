#!/usr/bin/env node
import { cp, mkdir, rm, writeFile, readFile, stat } from 'node:fs/promises';
import { existsSync } from 'node:fs';
import { dirname, join, relative, resolve, posix } from 'node:path';
import { fileURLToPath } from 'node:url';
import { buildLevelMjcf } from './sim-level-mjcf.mjs';
import { writePolicy } from './gen-policy.mjs';

const GUI = dirname(dirname(fileURLToPath(import.meta.url)));
const RES = join(GUI, '..', 'resources');
const PKG = join(GUI, 'node_modules', '@mujoco', 'mujoco');
const OUT = join(GUI, 'public', 'mujoco');
const ENTRY = 'model/rbq_environment.xml';
const ENTRY_DIR = posix.dirname(ENTRY);

if (!existsSync(RES)) {
  console.error(`[sync-mujoco] 원본 없음: ${RES}`);
  process.exit(1);
}
if (!existsSync(PKG)) {
  console.error('[sync-mujoco] @mujoco/mujoco 미설치 — npm ci 먼저');
  process.exit(1);
}

async function collect(relXml, seen = new Set()) {
  if (seen.has(relXml)) return seen;
  seen.add(relXml);
  const dir = posix.dirname(relXml);
  const xml = await readFile(join(RES, relXml), 'utf8');

  const dirOf = (attr) => {
    const m = xml.match(new RegExp(`${attr}\\s*=\\s*"([^"]*)"`));
    return m ? posix.normalize(posix.join(ENTRY_DIR, m[1])) : ENTRY_DIR;
  };
  const meshdir = dirOf('meshdir');
  const texturedir = dirOf('texturedir');

  for (const m of xml.matchAll(/<include\s+file\s*=\s*"([^"]+)"/g)) {
    await collect(posix.normalize(posix.join(dir, m[1])), seen);
  }
  for (const m of xml.matchAll(/<(mesh|texture|hfield)\b[^>]*\bfile\s*=\s*"([^"]+)"/g)) {
    const base = m[1] === 'texture' ? texturedir : meshdir;
    seen.add(posix.normalize(posix.join(base, m[2])));
  }
  return seen;
}

const refs = [...(await collect(ENTRY))].sort();
const missing = refs.filter((r) => !existsSync(join(RES, r)));
if (missing.length) {
  console.error(`[sync-mujoco] 참조 파일 없음:\n  ${missing.join('\n  ')}`);
  process.exit(1);
}

await rm(OUT, { recursive: true, force: true });
await mkdir(OUT, { recursive: true });

const outRel = (r) => r;

for (const r of refs) {
  const to = join(OUT, outRel(r));
  await mkdir(dirname(to), { recursive: true });
  await cp(join(RES, r), to);
}

await cp(join(PKG, 'mujoco.js'), join(OUT, 'mujoco.js'));
await cp(join(PKG, 'mujoco.wasm'), join(OUT, 'mujoco.wasm'));
await writeFile(
  join(OUT, 'loader.js'),
  `// 번들러를 우회해 글루를 진짜 모듈 URL 로 평가시키는 얇은 껍데기 (sync-mujoco.mjs 생성).\n` +
    `import loadMujoco from './mujoco.js';\n` +
    `window.__mujocoFactory = loadMujoco;\n` +
    `window.dispatchEvent(new Event('mujoco-glue-ready'));\n`,
);

const LV = JSON.parse(await readFile(join(GUI, 'src', 'lib', 'sim', 'levels.json'), 'utf8'));
const mapFiles = [];
for (const lv of LV.levels) {
  const { xml, shapes } = buildLevelMjcf(lv, LV.colors);
  await writeFile(join(OUT, 'model', `level_${lv.id}.xml`), xml);
  await writeFile(join(OUT, 'model', `level_${lv.id}.shapes.json`), JSON.stringify({ colors: LV.colors, shapes, spawn: lv.spawn }));
  mapFiles.push(`model/level_${lv.id}.xml`);
}

const pol = await writePolicy(OUT, join(RES, 'policy', 'rbq10_trot'));

const files = [...refs.map(outRel), ...mapFiles].sort();
await writeFile(join(OUT, 'files.json'), JSON.stringify(files, null, 0));

let bytes = 0;
for (const f of files) bytes += (await stat(join(OUT, f))).size;
console.log(
  `[sync-mujoco] 정책 rbq10_trot iter${pol.meta.iteration} ${(pol.bytes / 1048576).toFixed(2)}MB ` +
    `(${pol.meta.layers.map((l) => l.in).join('→')}→${pol.meta.numActions})`,
);
console.log(
  `[sync-mujoco] ${files.length}개 자산 ${(bytes / 1048576).toFixed(1)}MB + 엔진 ` +
    `${((await stat(join(OUT, 'mujoco.wasm'))).size / 1048576).toFixed(1)}MB → public/mujoco/`,
);

import { build } from 'esbuild';
import { copyFile } from 'node:fs/promises';
await build({
  entryPoints: [join(GUI, 'src/sim-worker/worker.ts')],
  bundle: true,
  format: 'iife',
  target: 'es2020',
  minify: true,
  outfile: join(OUT, 'worker.js'),
  alias: { '@': join(GUI, 'src') },
  logLevel: 'error',
});
await copyFile(join(GUI, 'src/sim-worker/worker.html'), join(OUT, 'worker.html'));
console.log('[mujoco:sync] worker.js + worker.html');
