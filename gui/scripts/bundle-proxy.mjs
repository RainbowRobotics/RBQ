import { execSync } from 'node:child_process';
import { mkdirSync, renameSync, existsSync, rmSync, cpSync } from 'node:fs';
import { fileURLToPath } from 'node:url';
import { dirname, join } from 'node:path';

const root = join(dirname(fileURLToPath(import.meta.url)), '..');
const outDir = join(root, 'src-tauri', 'binaries');
mkdirSync(outDir, { recursive: true });

const ext = process.platform === 'win32' ? '.exe' : '';
const rawOut = join(outDir, `rbq-proxy${ext}`);

execSync(
  `npx pkg tools/rbq-web-proxy.js --targets host --public --output "${rawOut}"`,
  { cwd: root, stdio: 'inherit' }
);

const triple = execSync('rustc --print host-tuple').toString().trim();
const finalOut = join(outDir, `rbq-proxy-${triple}${ext}`);
renameSync(rawOut, finalOut);

if (process.platform !== 'win32') {
  execSync(`chmod +x "${finalOut}"`);
}
if (!existsSync(finalOut)) throw new Error(`사이드카 생성 실패: ${finalOut}`);
console.log(`사이드카 생성: ${finalOut}`);

const wrtcSrcRoot = join(root, 'node_modules', '@roamhq');
const wrtcStage = join(outDir, 'wrtc-modules', 'node_modules', '@roamhq');
if (existsSync(join(wrtcSrcRoot, 'wrtc'))) {
  rmSync(join(outDir, 'wrtc-modules'), { recursive: true, force: true });
  mkdirSync(wrtcStage, { recursive: true });
  cpSync(join(wrtcSrcRoot, 'wrtc'), join(wrtcStage, 'wrtc'), { recursive: true });
  for (const p of ['wrtc-linux-x64', 'wrtc-darwin-x64', 'wrtc-darwin-arm64', 'wrtc-win32-x64']) {
    if (existsSync(join(wrtcSrcRoot, p))) cpSync(join(wrtcSrcRoot, p), join(wrtcStage, p), { recursive: true });
  }
  console.log(`wrtc-modules 스테이징: ${join(outDir, 'wrtc-modules')}`);
} else {
  console.log('주의: @roamhq/wrtc 미설치 — 데스크탑 WebRTC 브리지 비활성(웹 모드는 무관). npm install 후 재실행 권장.');
}

const homeCache = join(process.env.HOME || '', '.cache', 'rbq', 'ffmpeg');
let ffmpegSrc = process.env.RBQ_FFMPEG_STATIC && existsSync(process.env.RBQ_FFMPEG_STATIC) ? process.env.RBQ_FFMPEG_STATIC
  : existsSync(homeCache) ? homeCache : '';
let ffStatic = !!ffmpegSrc;
if (!ffmpegSrc) { try { ffmpegSrc = execSync('command -v ffmpeg').toString().trim(); } catch { ffmpegSrc = ''; } }
if (ffmpegSrc && existsSync(ffmpegSrc)) {
  const staged = join(outDir, `ffmpeg${ext}`);
  cpSync(ffmpegSrc, staged, { dereference: true });
  if (process.platform !== 'win32') execSync(`chmod +x "${staged}"`);
  console.log(`ffmpeg 스테이징: ${staged} (from ${ffmpegSrc}${ffStatic ? ', 정적✓' : ', ⚠동적 — 타깃에 ffmpeg lib 필요, 정적본 권장'})`);
} else {
  console.log('주의: ffmpeg 미발견 — 데스크탑 카메라 영상 비활성. 정적 ffmpeg를 ~/.cache/rbq/ffmpeg 에 두고 재실행.');
}
