const { withDangerousMod, withXcodeProject, IOSConfig } = require('expo/config-plugins');
const fs = require('node:fs');
const path = require('node:path');

function copyDir(src, dst) {
  fs.mkdirSync(dst, { recursive: true });
  for (const e of fs.readdirSync(src, { withFileTypes: true })) {
    const s = path.join(src, e.name);
    const d = path.join(dst, e.name);
    if (e.isDirectory()) copyDir(s, d);
    else fs.copyFileSync(s, d);
  }
}

function withSimAssetsIos(config) {
  config = withDangerousMod(config, [
    'ios',
    (cfg) => {
      const src = path.join(cfg.modRequest.projectRoot, 'public', 'mujoco');
      const appName = cfg.modRequest.projectName;
      const dst = path.join(cfg.modRequest.platformProjectRoot, appName, 'mujoco');
      if (!fs.existsSync(path.join(src, 'worker.html'))) {
        console.warn('[withSimAssets] public/mujoco/worker.html 없음 — `npm run mujoco:sync` 먼저. iOS 시뮬 자산 없이 진행');
        return cfg;
      }
      fs.rmSync(dst, { recursive: true, force: true });
      copyDir(src, dst);
      console.log(`[withSimAssets] public/mujoco → ${path.relative(cfg.modRequest.projectRoot, dst)}`);
      return cfg;
    },
  ]);
  config = withXcodeProject(config, (cfg) => {
    const project = cfg.modResults;
    const appName = cfg.modRequest.projectName;
    const rel = path.join(appName, 'mujoco');
    if (!fs.existsSync(path.join(cfg.modRequest.platformProjectRoot, rel))) return cfg;
    if (!project.hasFile(rel)) {
      IOSConfig.XcodeUtils.addResourceFileToGroup({ filepath: rel, groupName: appName, project, isBuildFile: true, verbose: false });
      console.log(`[withSimAssets] Xcode 리소스 폴더 참조 추가: ${rel}`);
    }
    const refs = project.hash.project.objects.PBXFileReference;
    for (const key of Object.keys(refs)) {
      const r = refs[key];
      if (r && typeof r === 'object' && (r.path === `"${rel}"` || r.path === rel)) {
        r.lastKnownFileType = 'folder';
        delete r.explicitFileType;
        delete r.fileEncoding;
        delete r.includeInIndex;
      }
    }
    return cfg;
  });
  return config;
}

module.exports = function withSimAssets(config) {
  config = withSimAssetsIos(config);
  return withDangerousMod(config, [
    'android',
    (cfg) => {
      const src = path.join(cfg.modRequest.projectRoot, 'public', 'mujoco');
      const dst = path.join(cfg.modRequest.platformProjectRoot, 'app', 'src', 'main', 'assets', 'mujoco');
      if (!fs.existsSync(path.join(src, 'worker.html'))) {
        console.warn('[withSimAssets] public/mujoco/worker.html 없음 — `npm run mujoco:sync` 먼저. 시뮬 자산 없이 진행');
        return cfg;
      }
      fs.rmSync(dst, { recursive: true, force: true });
      copyDir(src, dst);
      console.log(`[withSimAssets] public/mujoco → ${path.relative(cfg.modRequest.projectRoot, dst)}`);
      return cfg;
    },
  ]);
};
