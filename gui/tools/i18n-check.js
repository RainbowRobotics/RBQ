#!/usr/bin/env node
const fs = require('fs'), path = require('path');
const ROOT = path.join(__dirname, '..', 'src');
const en = fs.readFileSync(path.join(ROOT, 'lib/i18n.en.ts'), 'utf8');
const modDir = path.join(ROOT, 'modules');
const modEn = fs.existsSync(modDir) ? fs.readdirSync(modDir, { withFileTypes: true })
  .filter((d) => d.isDirectory() && fs.existsSync(path.join(modDir, d.name, 'i18n.en.ts')))
  .map((d) => fs.readFileSync(path.join(modDir, d.name, 'i18n.en.ts'), 'utf8')).join('\n') : '';
const KEYS = new Set([...(en + '\n' + modEn).matchAll(/^\s*"((?:[^"\\]|\\.)*)"\s*:/gm)].map((m) => m[1]));
if (KEYS.size < 100) {
  console.error(`i18n-check: EN 맵에서 키를 ${KEYS.size}개밖에 못 읽었다 — i18n.en.ts 포맷이 바뀌었는지`);
  console.error('  확인할 것(파서는 `^  "키": "값",` 한 줄 형식을 가정한다). 번역 누락 문제가 아니다.');
  process.exit(2);
}
const KO = /[가-힣]/;
const files = [];
(function walk(d) {
  for (const f of fs.readdirSync(d, { withFileTypes: true })) {
    const p = path.join(d, f.name);
    if (f.isDirectory()) walk(p);
    else if (/\.tsx?$/.test(f.name) && !/i18n|\.test\./.test(f.name)) files.push(p);
  }
})(ROOT);

let miss = 0, compose = 0;
for (const p of files) {
  const src = fs.readFileSync(p, 'utf8');
  const rel = path.relative(path.join(__dirname, '..'), p);
  for (const re of [/\bt\(\s*'((?:[^'\\]|\\.)*)'\s*\)/g, /\bt\(\s*"((?:[^"\\]|\\.)*)"\s*\)/g, /\bt\(\s*`([^`$]*)`\s*\)/g]) {
    for (const m of src.matchAll(re)) {
      if (KO.test(m[1]) && !KEYS.has(m[1])) { console.log(`[번역없음] ${rel}: ${m[1]}`); miss++; }
    }
  }
  for (const m of src.matchAll(/\bt\(\s*`([^`]*)`\s*\)/g)) {
    if (m[1].includes('${')) { console.log(`[조합후t()] ${rel}: \`${m[1].slice(0, 50)}\` — 조각별로 t() 를 걸어야 한다`); compose++; }
  }
}
console.log(`\n번역없음 ${miss}건 · 조합후t() ${compose}건`);
process.exit(miss + compose ? 1 : 0);
