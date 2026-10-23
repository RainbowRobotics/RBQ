#!/usr/bin/env node
const http = require('http');
const https = require('https');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');

let robotAuthHeader = '';
function authHeaders(req) {
  const a = (req && req.headers.authorization) || robotAuthHeader;
  return a ? { Authorization: a } : {};
}

function forceH264Pt96(sdp) {
  const lines = sdp.split('\r\n');
  const mIdx = lines.findIndex((l) => l.startsWith('m=video'));
  if (mIdx < 0) return sdp;
  let end = lines.findIndex((l, i) => i > mIdx && l.startsWith('m='));
  if (end < 0) end = lines.length;
  const kept = lines.slice(mIdx + 1, end).filter((l) => l.length > 0 && !/^a=(rtpmap|fmtp|rtcp-fb):/.test(l));
  const mParts = lines[mIdx].split(' ');
  const rebuilt = [
    `${mParts[0]} ${mParts[1]} ${mParts[2]} 96`,
    ...kept,
    'a=rtpmap:96 H264/90000',
    'a=rtcp-fb:96 nack',
    'a=rtcp-fb:96 nack pli',
    'a=rtcp-fb:96 goog-remb',
    'a=fmtp:96 level-asymmetry-allowed=1;packetization-mode=1;profile-level-id=42e01f',
  ];
  const out = [...lines.slice(0, mIdx), ...rebuilt, ...lines.slice(end)].filter((l) => l.length > 0);
  return out.join('\r\n') + '\r\n';
}

const args = process.argv.slice(2);
function argOf(name, dflt) {
  const i = args.indexOf(name);
  return i >= 0 && args[i + 1] ? args[i + 1] : dflt;
}
let ROBOT = argOf('--robot', '127.0.0.1');
let ROBOT_VISION = argOf('--robot-vision', '') || ROBOT;
const PORT = Number(argOf('--port', '8090'));
const ROBOT_PORT = Number(argOf('--robot-port', process.env.RBQ_ROBOT_PORT || '8080'));
const VISION_PORT = Number(argOf('--vision-port', process.env.RBQ_VISION_PORT || '8081'));
const SLAM_PORT = Number(argOf('--slam-port', process.env.RBQ_SLAM_PORT || '8082'));
const HOST = argOf('--host', '127.0.0.1');
const LOCAL_ADDR = argOf('--local-address', '');
const REVIEW = argOf('--review', process.env.RBQ_REVIEW || '');
const DIST = path.resolve(__dirname, '..', argOf('--dist', 'dist'));

let RV_TOKEN = process.env.RBQ_WEBRTC_TOKEN || argOf('--webrtc-token', '');

const MIME = {
  '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json',
  '.png': 'image/png', '.jpg': 'image/jpeg', '.svg': 'image/svg+xml', '.ico': 'image/x-icon',
  '.glb': 'model/gltf-binary', '.woff2': 'font/woff2', '.ttf': 'font/ttf', '.map': 'application/json',
};

const IN_STEAM_RUNTIME = fs.existsSync('/usr/bin/steam-runtime-urlopen');
function hostSpawn(tag, cmd, args) {
  const env = { ...process.env };
  delete env.LD_LIBRARY_PATH;
  delete env.LD_PRELOAD;
  for (const k of ['ELECTRON_RUN_AS_NODE', 'RBQ_PROXY_TOKEN', 'RBQ_WEBRTC_TOKEN', 'RBQ_DESKTOP_SHELL', 'RBQ_WIN_INSTALLER']) delete env[k];
  try {
    spawn(cmd, args, { stdio: 'ignore', detached: true, windowsHide: true, env })
      .on('error', (e) => console.error(`[${tag}] spawn 실패:`, cmd, e.message));
  } catch (e) { console.error(`[${tag}] spawn 예외:`, e.message); }
}
function hostOpenUrl(tag, target) {
  if (process.platform === 'win32') return hostSpawn(tag, 'explorer.exe', [target]);
  if (IN_STEAM_RUNTIME) return hostSpawn(tag, 'steam-runtime-urlopen', [target]);
  const gaming = !!process.env.GAMESCOPE_WAYLAND_DISPLAY;
  if (target.startsWith('steam://')) return hostSpawn(tag, 'steam', ['-ifrunning', target]);
  return gaming ? hostSpawn(tag, 'steam', ['-ifrunning', `steam://openurl/${target}`]) : hostSpawn(tag, 'xdg-open', [target]);
}
function hostRun(tag, cmd, args) {
  if (IN_STEAM_RUNTIME) return hostSpawn(tag, 'steam-runtime-launch-client', ['--alongside-steam', '--host', '--', cmd, ...args]);
  return hostSpawn(tag, cmd, args);
}
const DESKTOP_SHELL = !!(process.env.APPIMAGE || process.env.RBQ_DESKTOP_SHELL);
function isSteamOS() {
  for (const f of ['/run/host/os-release', '/etc/os-release']) {
    try { return /^ID=steamos$/m.test(fs.readFileSync(f, 'utf8')); } catch { }
  }
  return false;
}

const server = http.createServer((req, res) => {
  const dl = req.headers['x-download-base'];
  if (dl && !S3_FIXED && isLoopback(req) && /^https:\/\/[^/]+$/.test(dl)) S3_BASE = dl;
  const url = new URL(req.url, 'http://x');
  if (['/fw-download', '/app-update'].includes(url.pathname) && !S3_BASE) { res.writeHead(404); return res.end('download base not set'); }
  if (url.pathname === '/wifi-settings') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    if (process.platform === 'win32') hostOpenUrl('wifi-settings', 'ms-settings:network-wifi');
    else if (process.env.GAMESCOPE_WAYLAND_DISPLAY) hostOpenUrl('wifi-settings', 'steam://open/settings');
    else hostRun('wifi-settings', 'kcmshell6', ['kcm_networkmanagement']);
    res.writeHead(204); return res.end();
  }
  if (url.pathname === '/local-sim' || url.pathname === '/probe-robot') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    const host = url.pathname === '/local-sim' ? '127.0.0.1' : String(url.searchParams.get('ip') || '').trim();
    if (!/^[A-Za-z0-9.\-]{1,253}$/.test(host)) { res.writeHead(400); return res.end(); }
    const q = http.get({ host, port: ROBOT_PORT, path: '/api/robot/serial_number', headers: authHeaders(req), timeout: 1500 }, (r) => {
      let body = '';
      r.on('data', (d) => { body += d; });
      r.on('end', () => {
        let serial = ''; let shaped = false;
        try { const j = JSON.parse(body); shaped = typeof j.serial_number === 'string'; serial = String(j.serial_number || '').trim(); } catch { }
        res.writeHead(200, { 'Content-Type': 'application/json' });
        res.end(JSON.stringify({ ok: r.statusCode === 200 && shaped, serial, auth: r.statusCode === 401 }));
      });
    });
    const no = () => { if (!res.headersSent) { res.writeHead(200, { 'Content-Type': 'application/json' }); res.end(JSON.stringify({ ok: false })); } };
    q.on('timeout', () => { q.destroy(); no(); });
    q.on('error', no);
    return;
  }
  if (url.pathname === '/kv') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    const key = String(url.searchParams.get('key') || '');
    if (!/^[A-Za-z0-9_-]{1,64}$/.test(key)) { res.writeHead(400); return res.end(); }
    const dir = path.join(process.env.XDG_CONFIG_HOME || path.join(require('os').homedir(), '.config'), 'rbq');
    const file = path.join(dir, `${key}.json`);
    if (req.method === 'GET') {
      fs.readFile(file, 'utf8', (err, data) => {
        if (err) { res.writeHead(404); return res.end(); }
        res.writeHead(200, { 'Content-Type': 'application/json' }); res.end(data);
      });
      return;
    }
    if (req.method === 'PUT') {
      let body = '';
      req.on('data', (d) => { body += d; if (body.length > 1_000_000) req.destroy(); });
      req.on('end', () => {
        try {
          fs.mkdirSync(dir, { recursive: true });
          const tmp = `${file}.tmp`;
          fs.writeFileSync(tmp, body);
          fs.renameSync(tmp, file);
          res.writeHead(204); res.end();
        } catch (e) { res.writeHead(500); res.end(String(e && e.message)); }
      });
      return;
    }
    res.writeHead(405); return res.end();
  }
  if (url.pathname === '/bb-lib') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    const root = path.join(process.env.XDG_DATA_HOME || path.join(require('os').homedir(), '.local', 'share'), 'rbq', 'blackbox');
    const SERIAL_OK = (s) => /^(@nosn|[A-Za-z0-9._-]{1,64})$/.test(s) && !/^\.+$/.test(s);
    const NAME_OK = (n) => /^blackbox-[A-Za-z0-9._-]{1,120}\.zip$/.test(n);
    const serial = String(url.searchParams.get('serial') || '');
    const name = String(url.searchParams.get('name') || '');
    if (req.method === 'GET' && !serial && !name) {
      const out = [];
      let dirs = [];
      try { dirs = fs.readdirSync(root, { withFileTypes: true }); } catch { }
      for (const d of dirs) {
        if (!d.isDirectory() || !SERIAL_OK(d.name)) continue;
        let files = [];
        try { files = fs.readdirSync(path.join(root, d.name)); } catch { continue; }
        for (const f of files) {
          if (!NAME_OK(f)) continue;
          try { out.push({ serial: d.name, name: f, size: fs.statSync(path.join(root, d.name, f)).size }); } catch { }
        }
      }
      res.writeHead(200, { 'Content-Type': 'application/json' });
      return res.end(JSON.stringify(out));
    }
    if (!SERIAL_OK(serial) || !NAME_OK(name)) { res.writeHead(400); return res.end(); }
    const dir = path.join(root, serial);
    const file = path.join(dir, name);
    if (req.method === 'GET') {
      fs.stat(file, (err, st) => {
        if (err) { res.writeHead(404); return res.end(); }
        res.writeHead(200, { 'Content-Type': 'application/zip', 'Content-Length': st.size });
        fs.createReadStream(file).pipe(res);
      });
      return;
    }
    if (req.method === 'PUT') {
      try { fs.mkdirSync(dir, { recursive: true }); } catch (e) { res.writeHead(500); return res.end(String(e && e.message)); }
      const tmp = `${file}.tmp`;
      const out = fs.createWriteStream(tmp);
      const fail = (e) => {
        out.destroy(); fs.rm(tmp, { force: true }, () => {});
        if (!res.headersSent) { res.writeHead(e && e.code === 'ENOSPC' ? 507 : 500); res.end(String(e && e.message)); }
      };
      out.on('error', fail);
      req.on('error', fail);
      req.on('aborted', () => fail(new Error('aborted')));
      out.on('finish', () => {
        try { fs.renameSync(tmp, file); res.writeHead(204); res.end(); } catch (e) { fail(e); }
      });
      req.pipe(out);
      return;
    }
    if (req.method === 'DELETE') {
      fs.rm(file, { force: true }, (err) => {
        if (err) { res.writeHead(500); return res.end(String(err.message)); }
        fs.rmdir(dir, () => {});
        res.writeHead(204); res.end();
      });
      return;
    }
    res.writeHead(405); return res.end();
  }
  if (url.pathname === '/touch-native') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    if (process.env.GAMESCOPE_WAYLAND_DISPLAY) hostRun('touch-native', 'gamescopectl', ['touch_click_mode', '4']);
    res.writeHead(204); return res.end();
  }
  if (url.pathname === '/open-url') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    const target = url.searchParams.get('u') || '';
    if (!/^https?:\/\//.test(target)) { res.writeHead(400); return res.end(); }
    hostOpenUrl('open-url', target);
    res.writeHead(204); return res.end();
  }
  if (url.pathname === '/osk') {
    hostOpenUrl('osk', 'steam://open/keyboard');
    res.writeHead(204); return res.end();
  }
  if (url.pathname === '/netid') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    const VIRTUAL = /^(docker|br-|veth|virbr|vmnet|lo)/;
    const nets = require('os').networkInterfaces();
    const id = Object.entries(nets)
      .filter(([n]) => !VIRTUAL.test(n))
      .flatMap(([n, addrs]) => (addrs || [])
        .filter((a) => a.family === 'IPv4' && !a.internal)
        .map((a) => `${n}:${a.address}/${a.netmask}`))
      .sort().join(',');
    res.writeHead(200, { 'Content-Type': 'application/json' });
    return res.end(JSON.stringify({ id }));
  }
  if (url.pathname === '/platform') {
    const steamos = isSteamOS();
    const gaming = !!process.env.GAMESCOPE_WAYLAND_DISPLAY;
    res.writeHead(200, { 'Content-Type': 'application/json' });
    return res.end(JSON.stringify({ steamos, gaming }));
  }
  if (url.pathname === '/local/list' || url.pathname === '/local/file') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    if (!process.env.GAMESCOPE_WAYLAND_DISPLAY) { res.writeHead(404); return res.end(); }
    return url.pathname === '/local/list' ? localList(url, res) : localFile(url, res);
  }
  if (url.pathname === '/app-update' && !isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
  if (url.pathname === '/app-update') return appSelfUpdate(url, res);
  if (url.pathname === '/whoami') {
    if (!process.env.RBQ_PROXY_TOKEN) { res.writeHead(501); return res.end('no identity'); }
    return jsonRes(res, 200, { token: process.env.RBQ_PROXY_TOKEN, pid: process.pid });
  }
  if (url.pathname === '/app-status') return appStatus(url, res);
  if ((url.pathname === '/app-select' || url.pathname === '/app-remove') && !isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
  if (url.pathname === '/app-select') return appSelect(url, res);
  if (url.pathname === '/app-remove') return appRemove(url, res);
  if (url.pathname === '/fw-download') return fwDownload(url, res);
  if (url.pathname === '/fw-file') return fwFile(url, res);
  if (url.pathname.startsWith('/issue/')) {
    const sub = url.pathname.slice('/issue'.length);
    if (sub !== '/health' && sub !== '/report') { res.writeHead(404); return res.end(); }
    const base = (isLoopback(req) && req.headers['x-review-target']) || REVIEW;
    if (!/^https?:\/\//.test(base)) { res.writeHead(404); return res.end(); }
    const target = new URL(base.replace(/\/+$/, '') + sub);
    const fwd = (target.protocol === 'https:' ? https : http).request(target, {
      method: req.method, timeout: 20000,
      headers: {
        'content-type': req.headers['content-type'] || 'application/json',
        ...(req.headers['content-length'] ? { 'content-length': req.headers['content-length'] } : {}),
      },
    }, (r2) => { res.writeHead(r2.statusCode || 502, { 'content-type': r2.headers['content-type'] || 'application/json' }); r2.pipe(res); });
    fwd.on('error', () => { res.writeHead(502); res.end(); });
    fwd.on('timeout', () => fwd.destroy(new Error('timeout')));
    req.pipe(fwd);
    return;
  }
  if (url.pathname === '/proxy/target') return proxyTarget(req, res);
  if (url.pathname.startsWith('/api/')) {
    if (req.headers.authorization && isLoopback(req)) robotAuthHeader = req.headers.authorization;
    return proxyHttp(req, res, ROBOT, ROBOT_PORT, url.pathname + url.search);
  }
  if (url.pathname.startsWith('/streamer/'))
    return proxyHttp(req, res, ROBOT_VISION, VISION_PORT, url.pathname.slice('/streamer'.length) + url.search);
  if (url.pathname.startsWith('/slamnav/'))
    return proxyHttp(req, res, ROBOT_VISION, SLAM_PORT, url.pathname.slice('/slamnav'.length) + url.search);
  serveStatic(url.pathname, res);
});

let S3_BASE = argOf('--download-base', process.env.RBQ_DOWNLOAD_BASE || '');
const S3_FIXED = !!S3_BASE;
const CTRL_ASSET = `rbq-controller-${process.arch === 'arm64' ? 'aarch64' : 'x86_64'}.AppImage`;
const CTRL_APPS = {
  release: { dir: 'RBQ',         entry: 'RBQ',         name: 'RBQ',         legacy: 'rbq-controller' },
  nightly: { dir: 'RBQ-nightly', entry: 'RBQ-nightly', name: 'RBQ-nightly', legacy: 'rbq-controller-nightly' },
};
const kNightlyKeep = 10;
const CTRL_LINK_DIR = 'latest';

function xdgUserDir(key, fallback) {
  const home = process.env.HOME || '';
  try {
    const conf = fs.readFileSync(path.join(process.env.XDG_CONFIG_HOME || path.join(home, '.config'), 'user-dirs.dirs'), 'utf8');
    const m = new RegExp(`^\\s*${key}="(.+)"`, 'm').exec(conf);
    if (m) return m[1].replace('$HOME', home);
  } catch { }
  return path.join(home, fallback);
}
const ctrlBase = (ch) => path.join(xdgUserDir('XDG_DOCUMENTS_DIR', 'Documents'), CTRL_APPS[ch].dir);
const ctrlLink = (ch) => path.join(ctrlBase(ch), CTRL_LINK_DIR, CTRL_ASSET);
const inTree = (dir, p) => p === dir || p.startsWith(dir + path.sep);
const isDay = (s) => /^\d{4}-\d{2}-\d{2}$/.test(s);

function ctrlBuilds(ch) {
  try {
    return fs.readdirSync(ctrlBase(ch), { withFileTypes: true })
      .filter((e) => e.isDirectory() && e.name !== CTRL_LINK_DIR)
      .map((e) => e.name)
      .sort((a, b) => (isDay(a) === isDay(b) ? b.localeCompare(a) : (isDay(a) ? -1 : 1)));
  } catch { return []; }
}

function ctrlCurrent(ch) {
  try { return path.basename(path.dirname(fs.realpathSync(ctrlLink(ch)))); } catch { return null; }
}

function ctrlPrune(ch, running) {
  const base = ctrlBase(ch);
  const keep = new Set([ctrlCurrent(ch), running].filter(Boolean));
  const builds = ctrlBuilds(ch);
  if (ch === 'nightly') builds.slice(0, kNightlyKeep).forEach((d) => keep.add(d));
  for (const d of builds) {
    if (keep.has(d)) continue;
    if (ch !== 'nightly' && fwTarIn(path.join(base, d))) continue;
    fs.rmSync(path.join(base, d), { recursive: true, force: true });
    console.log(`[app-update] 옛 빌드 정리(${ch}): ${d}`);
  }
}

function ctrlDesktopEntry(ch, exec) {
  const home = process.env.HOME || '';
  const app = CTRL_APPS[ch];
  const apps = path.join(process.env.XDG_DATA_HOME || path.join(home, '.local', 'share'), 'applications');
  const icon = path.join(home, '.local', 'share', 'icons', `${app.entry}.png`);
  fs.mkdirSync(apps, { recursive: true });
  fs.mkdirSync(path.dirname(icon), { recursive: true });
  if (!fs.existsSync(icon)) {
    for (const c of ['.DirIcon', 'usr/share/icons/hicolor/128x128/apps/app.png']) {
      try { fs.copyFileSync(path.join(process.env.APPDIR || '', c), icon); break; } catch { }
    }
  }
  const iconLine = fs.existsSync(icon) ? `Icon=${icon}\n` : '';
  const file = path.join(apps, `${app.entry}.desktop`);
  fs.writeFileSync(file, `[Desktop Entry]\nName=${app.name}\nExec=${exec}\n${iconLine}`
    + 'Type=Application\nCategories=Utility;\nTerminal=false\n');
  fs.chmodSync(file, 0o755);
  const shortcut = path.join(xdgUserDir('XDG_DESKTOP_DIR', 'Desktop'), `${app.entry}.desktop`);
  try { fs.rmSync(shortcut, { force: true }); fs.symlinkSync(file, shortcut); } catch { }
  for (const stale of [path.join(apps, `${app.legacy}.desktop`),
                       path.join(xdgUserDir('XDG_DESKTOP_DIR', 'Desktop'), `${app.legacy}.desktop`),
                       path.join(home, '.local', 'share', 'icons', `${app.legacy}.png`)]) {
    try { fs.rmSync(stale, { force: true }); } catch { }
  }
  const env = { ...process.env };
  for (const k of Object.keys(env)) if (/^(LD_|GST_|GTK_|GIO_|QT_)/.test(k)) delete env[k];
  const run = (bin, args) => {
    try { spawn(bin, args, { env, stdio: 'ignore' }).on('error', () => {}); } catch { }
  };
  run('gio', ['set', shortcut, 'metadata::trusted', 'true']);
  run('xdg-desktop-menu', ['forceupdate']);
  return file;
}

function ctrlPoint(ch, build) {
  const target = path.join(ctrlBase(ch), build, CTRL_ASSET);
  if (!fs.existsSync(target)) throw new Error(`빌드 없음: ${build}`);
  const link = ctrlLink(ch);
  fs.mkdirSync(path.dirname(link), { recursive: true });
  const tmpLink = `${link}.tmp`;
  fs.rmSync(tmpLink, { force: true });
  fs.symlinkSync(target, tmpLink);
  fs.renameSync(tmpLink, link);
  try { ctrlDesktopEntry(ch, link); } catch (e) { console.log(`[app-update] .desktop 실패: ${e.message || e}`); }
  return link;
}

function ctrlPruneStale() {
  const appimage = process.env.APPIMAGE;
  if (!appimage) return;
  let real;
  try { real = fs.realpathSync(appimage); } catch { return; }
  for (const ch of Object.keys(CTRL_APPS)) {
    if (!inTree(ctrlBase(ch), real)) continue;
    const running = path.dirname(real) === ctrlBase(ch) ? null : path.basename(path.dirname(real));
    if (!running && !ctrlCurrent(ch)) return;
    ctrlPrune(ch, running);
  }
}

function fetchNightlyManifest(cb) {
  if (!S3_BASE) return cb(null);
  https.get(`${S3_BASE}/nightly/nightly.txt`, (r) => {
    if (r.statusCode !== 200) { r.resume(); return cb(null); }
    let body = '';
    r.setEncoding('utf8');
    r.on('data', (c) => { body += c; });
    r.on('end', () => cb(body));
  }).on('error', () => cb(null));
}
const manifestValue = (text, key) => (new RegExp(`^${key}:\\s*(\\S+)`, 'm').exec(text || '') || [])[1] || '';

const jsonRes = (res, code, obj) => {
  res.writeHead(code, { 'Content-Type': 'application/json' });
  res.end(JSON.stringify(obj));
};

function appStatus(url, res) {
  if (!process.env.APPIMAGE) { res.writeHead(501); return res.end('not running from an AppImage'); }
  const ch = url.searchParams.get('channel') || 'release';
  if (!CTRL_APPS[ch]) return jsonRes(res, 400, { error: 'bad channel' });
  const appimage = process.env.APPIMAGE || '';
  let real = appimage;
  try { real = fs.realpathSync(appimage); } catch { }
  const current = ctrlCurrent(ch);
  jsonRes(res, 200, {
    appimage: !!appimage,
    installed: !!current,
    managed: !!appimage && inTree(ctrlBase(ch), real),
    base: ctrlBase(ch),
    current,
    builds: ctrlBuilds(ch).map((name) => {
      const dir = path.join(ctrlBase(ch), name);
      const tar = fwTarIn(dir);
      let size = 0;
      if (tar) { try { size = fs.statSync(path.join(dir, tar)).size; } catch { } }
      return {
        name,
        current: name === current,
        app: fs.existsSync(path.join(dir, CTRL_ASSET)),
        version: manifestValue(
          (() => { try { return fs.readFileSync(path.join(dir, 'nightly.txt'), 'utf8'); } catch { return ''; } })(),
          'version'),
        tar: tar ? { name: tar, size } : null,
      };
    }),
  });
}

function appSelect(url, res) {
  if (!process.env.APPIMAGE) { res.writeHead(501); return res.end('not running from an AppImage'); }
  const ch = url.searchParams.get('channel') || 'release';
  const build = url.searchParams.get('build') || '';
  if (!CTRL_APPS[ch]) return jsonRes(res, 400, { error: 'bad channel' });
  if (!/^[A-Za-z0-9._-]+$/.test(build)) return jsonRes(res, 400, { error: 'bad build' });
  if (build === CTRL_LINK_DIR) return jsonRes(res, 400, { error: 'bad build' });
  try {
    ctrlPoint(ch, build);
    console.log(`[app-update] ${ch} 빌드 전환 → ${build}`);
    jsonRes(res, 200, { ok: true, current: build });
  } catch (e) {
    jsonRes(res, 400, { error: String((e && e.message) || e) });
  }
}

function fwTarIn(dir) {
  try { return fs.readdirSync(dir).find((f) => /^RBQ-.+\.tar\.gz$/.test(f)) || null; } catch { return null; }
}

function fwTarget(ch, tag, folder) {
  const name = ch === 'nightly' ? 'RBQ-nightly.tar.gz' : `RBQ-${tag}.tar.gz`;
  return { dir: path.join(ctrlBase(ch), folder), name, url: ch === 'nightly'
    ? `${S3_BASE}/nightly/RBQ-nightly.tar.gz`
    : `${S3_BASE}/${tag}/RBQ-${tag}.tar.gz` };
}

function fwFolder(ch, tag, cb) {
  if (ch !== 'nightly') return cb(tag);
  fetchNightlyManifest((text) => {
    const day = manifestValue(text, 'built_at').slice(0, 10);
    cb(isDay(day) ? day : (ctrlCurrent(ch) || tag));
  });
}

const SAFE_SEG = /^[A-Za-z0-9._-]+$/;

function fwDownload(url, res) {
  if (!DESKTOP_SHELL) { res.writeHead(501); return res.end('not running from a desktop shell'); }
  const ch = url.searchParams.get('channel') || 'release';
  const tag = url.searchParams.get('tag') || '';
  if (!CTRL_APPS[ch] || !SAFE_SEG.test(tag)) { res.writeHead(400); return res.end('bad request'); }
  fwFolder(ch, tag, (folder) => {
    const t = fwTarget(ch, tag, folder);
    try { fs.mkdirSync(t.dir, { recursive: true }); }
    catch (e) { res.writeHead(500); return res.end(`ERR:${e.message || e}`); }
    const dest = path.join(t.dir, t.name);
    const tmp = `${dest}.part`;
    console.log(`[fw] ${ch} ${tag} → ${dest}`);
    res.writeHead(200, { 'Content-Type': 'text/plain' });
    let got = 0, total = 0;
    let done = false;
    const ctl = {};
    const tick = setInterval(() => { try { res.write(`P ${got} ${total}\n`); } catch { } }, 500);
    const finish = (line) => { done = true; clearInterval(tick); try { res.end(line + '\n'); } catch { } };
    res.on('close', () => {
      if (done) return;
      done = true; clearInterval(tick);
      try { if (ctl.cancel) ctl.cancel(); } catch { }
      try { fs.unlinkSync(tmp); } catch { }
      console.log(`[fw] 취소 — ${dest}`);
    });
    downloadToFile(t.url, tmp, 5, (err) => {
      if (done) return;
      if (err) {
        try { fs.unlinkSync(tmp); } catch { }
        return finish(`ERR:${err.message || err}`);
      }
      try {
        fs.renameSync(tmp, dest);
        const mf = path.join(t.dir, 'nightly.txt');
        if (ch === 'nightly' && !fs.existsSync(mf)) {
          fetchNightlyManifest((text) => { if (text) { try { fs.writeFileSync(mf, text); } catch { } } });
        }
        finish(`OK ${t.name} ${fs.statSync(dest).size}`);
      } catch (e) {
        try { fs.unlinkSync(tmp); } catch { }
        finish(`ERR:${e.message || e}`);
      }
    }, (chunk, len) => { got += chunk; total = len; }, ctl);
  });
}

function fwFile(url, res) {
  if (!DESKTOP_SHELL) { res.writeHead(501); return res.end('not running from a desktop shell'); }
  const ch = url.searchParams.get('channel') || 'release';
  const tag = url.searchParams.get('tag') || '';
  const want = url.searchParams.get('folder') || '';
  if (!CTRL_APPS[ch] || !SAFE_SEG.test(tag)) { res.writeHead(400); return res.end(); }
  if (want && !SAFE_SEG.test(want)) { res.writeHead(400); return res.end(); }
  const pick = (cb) => (want ? cb(want) : fwFolder(ch, tag, cb));
  pick((folder) => {
    const t = fwTarget(ch, tag, folder);
    const name = want ? (fwTarIn(t.dir) || t.name) : t.name;
    const file = path.join(t.dir, name);
    let st;
    try { st = fs.statSync(file); } catch { res.writeHead(404); return res.end(); }
    res.writeHead(200, { 'Content-Type': 'application/octet-stream', 'Content-Length': st.size });
    fs.createReadStream(file).pipe(res);
  });
}

function appRemove(url, res) {
  if (!process.env.APPIMAGE) { res.writeHead(501); return res.end('not running from an AppImage'); }
  const ch = url.searchParams.get('channel') || 'release';
  const build = url.searchParams.get('build') || '';
  if (!CTRL_APPS[ch] || !SAFE_SEG.test(build)) return jsonRes(res, 400, { error: 'bad request' });
  if (build === CTRL_LINK_DIR) return jsonRes(res, 400, { error: 'bad request' });
  if (ctrlCurrent(ch) === build) return jsonRes(res, 400, { error: '지금 쓰는 빌드는 지울 수 없습니다' });
  const dir = path.join(ctrlBase(ch), build);
  if (!fs.existsSync(dir)) return jsonRes(res, 404, { error: 'no such build' });
  try {
    fs.rmSync(dir, { recursive: true, force: true });
    console.log(`[app-update] 제거(${ch}): ${build}`);
    jsonRes(res, 200, { ok: true });
  } catch (e) {
    jsonRes(res, 500, { error: String((e && e.message) || e) });
  }
}

function appSelfUpdate(url, res) {
  const appimage = process.env.APPIMAGE || '';
  const winExe = process.env.RBQ_WIN_INSTALLER || '';
  if (url.searchParams.get('check')) { res.writeHead(appimage || winExe ? 204 : 501, winExe ? { 'X-App-Mode': 'exe' } : {}); return res.end(); }
  if (winExe) return winSelfUpdate(url, res);
  if (!appimage) { res.writeHead(501); return res.end('not running from an AppImage'); }
  const SAFE = /^[A-Za-z0-9._-]+$/;
  const channel = url.searchParams.get('channel') || '';
  const tag = url.searchParams.get('tag') || '';
  const asset = url.searchParams.get('asset') || CTRL_ASSET;
  if (!CTRL_APPS[channel]) { res.writeHead(400); return res.end('bad channel'); }
  if (!SAFE.test(asset)) { res.writeHead(400); return res.end('bad asset'); }
  if (!SAFE.test(tag)) { res.writeHead(400); return res.end('bad tag'); }
  const beta = channel === 'nightly';
  const installIt = url.searchParams.get('install') !== '0';
  const src = beta ? `${S3_BASE}/nightly/${asset}` : `${S3_BASE}/${tag}/${asset}`;

  const withFolder = (fn) => {
    if (!beta) return fn(tag, '');
    fetchNightlyManifest((text) => {
      const day = manifestValue(text, 'built_at').slice(0, 10);
      fn(isDay(day) ? day : tag, text || '');
    });
  };

  withFolder((folder, manifest) => {
    const base = ctrlBase(channel);
    let real = appimage;
    try { real = fs.realpathSync(appimage); } catch { }
    const adopting = !inTree(base, real);
    const dest = path.join(base, folder, asset);
    const tmp = `${dest}.new`;
    try { fs.mkdirSync(path.dirname(dest), { recursive: true }); }
    catch (e) { res.writeHead(500); return res.end(`ERR:${e.message || e}`); }
    console.log(`[app-update] ${channel} ${tag} → ${dest}${adopting ? '  (온라인 설치)' : ''}`);
    res.writeHead(200, { 'Content-Type': 'text/plain' });
    let got2 = 0;
    let total2 = 0;
    const hb = setInterval(() => {
      try { res.write(total2 > 0 ? `P ${got2} ${total2}\n` : '.'); } catch { }
    }, 3000);
    const finish = (line) => { clearInterval(hb); try { res.end('\n' + line); } catch { } };
    downloadToFile(src, tmp, 5, (err) => {
      if (err) {
        try { fs.unlinkSync(tmp); } catch { }
        console.log(`[app-update] 실패: ${err.message || err}`);
        return finish(`ERR:${err.message || err}`);
      }
      try {
        fs.chmodSync(tmp, 0o755);
        fs.renameSync(tmp, dest);
        if (beta && manifest) fs.writeFileSync(path.join(base, folder, 'nightly.txt'), manifest);
        if (installIt) {
          ctrlPoint(channel, folder);
          ctrlPrune(channel, adopting ? null : path.basename(path.dirname(real)));
        }
        console.log(`[app-update] 완료 — ${installIt ? '재시작하면 새 버전' : '받아만 둠'}`);
        if (!beta || !installIt) return finish('OK');
        const t2 = fwTarget(channel, tag, folder);
        const tarTmp = path.join(t2.dir, `${t2.name}.part`);
        console.log(`[app-update] 로봇 패키지도 함께: ${path.join(t2.dir, t2.name)}`);
        return downloadToFile(t2.url, tarTmp, 5, (e2) => {
          if (e2) {
            try { fs.unlinkSync(tarTmp); } catch { }
            console.log(`[app-update] 로봇 패키지 실패(앱은 정상): ${e2.message || e2}`);
          } else {
            try { fs.renameSync(tarTmp, path.join(t2.dir, t2.name)); } catch { }
          }
          finish('OK');
        }, (n, len) => { got2 += n; total2 = len; });
      } catch (e) {
        try { fs.unlinkSync(tmp); } catch { }
        finish(`ERR:${e.message || e}`);
      }
    });
  });
}

const WIN_ASSET = 'rbq-controller-x64-setup.exe';
function winSelfUpdate(url, res) {
  const SAFE = /^[A-Za-z0-9._-]+$/;
  const channel = url.searchParams.get('channel') || '';
  const tag = url.searchParams.get('tag') || '';
  if (!CTRL_APPS[channel]) { res.writeHead(400); return res.end('bad channel'); }
  if (!SAFE.test(tag)) { res.writeHead(400); return res.end('bad tag'); }
  if (url.searchParams.get('install') === '0') { res.writeHead(501); return res.end('fetch-only is AppImage only'); }
  const src = channel === 'nightly' ? `${S3_BASE}/nightly/${WIN_ASSET}` : `${S3_BASE}/${tag}/${WIN_ASSET}`;
  const tmp = require('os').tmpdir();
  const dest = path.join(tmp, `rbq-setup-${channel}-${tag}.exe`);
  for (const f of fs.readdirSync(tmp)) if (/^rbq-setup-.*\.exe(\.part)?$/.test(f)) { try { fs.unlinkSync(path.join(tmp, f)); } catch { } }
  console.log(`[app-update] ${channel} ${tag} → ${dest}`);
  res.writeHead(200, { 'Content-Type': 'text/plain' });
  const hb = setInterval(() => { try { res.write('.'); } catch { } }, 3000);
  downloadToFile(src, `${dest}.part`, 5, (dlErr) => verifySha256(`${dest}.part`, `${src}.sha256`, dlErr, (err) => {
    clearInterval(hb);
    if (!err) {
      try { fs.renameSync(`${dest}.part`, dest); } catch (e) { err = e; }
    }
    if (err) {
      try { fs.unlinkSync(`${dest}.part`); } catch { }
      console.log(`[app-update] 실패: ${err.message || err}`);
      return res.end(`\nERR:${err.message || err}`);
    }
    res.end('\nOK');
    hostSpawn('app-update', dest, ['/S', '--updated', '--force-run']);
  }));
}

function verifySha256(file, sumUrl, prevErr, cb) {
  if (prevErr) return cb(prevErr);
  https.get(sumUrl, (r) => {
    if (r.statusCode !== 200) { r.resume(); return cb(new Error(`체크섬 없음 (HTTP ${r.statusCode})`)); }
    let body = '';
    r.setEncoding('utf8');
    r.on('data', (c) => { body += c; });
    r.on('end', () => {
      const want = (/\b[0-9a-f]{64}\b/i.exec(body) || [])[0];
      if (!want) return cb(new Error('체크섬 형식 오류'));
      const h = require('crypto').createHash('sha256');
      fs.createReadStream(file).on('error', cb).on('data', (c) => h.update(c)).on('end', () => {
        const got = h.digest('hex');
        cb(got === want.toLowerCase() ? null : new Error(`체크섬 불일치 (${got.slice(0, 12)}…)`));
      });
    });
  }).on('error', cb);
}

function downloadToFile(u, dest, redirects, cb, onData, ctl) {
  const req = https.get(u, (r) => {
    if (ctl) ctl.cancel = () => r.destroy();
    if (r.statusCode >= 300 && r.statusCode < 400 && r.headers.location && redirects > 0) {
      r.resume(); return downloadToFile(r.headers.location, dest, redirects - 1, cb, onData, ctl);
    }
    if (r.statusCode !== 200) { r.resume(); return cb(new Error(`HTTP ${r.statusCode}`)); }
    const len = Number(r.headers['content-length'] || 0);
    if (onData) r.on('data', (c) => onData(c.length, len));
    const f = fs.createWriteStream(dest);
    r.pipe(f);
    f.on('finish', () => f.close(() => cb(null)));
    f.on('error', cb);
    r.on('error', cb);
  }).on('error', cb);
  if (ctl) ctl.cancel = () => req.destroy();
}

function proxyHttp(req, res, host, port, upstreamPath) {
  const up = http.request(
    { host, port, path: upstreamPath, method: req.method, headers: { ...req.headers, host: `${host}:${port}` },
      ...(LOCAL_ADDR ? { localAddress: LOCAL_ADDR } : {}) },
    (upRes) => {
      res.writeHead(upRes.statusCode, upRes.headers);
      upRes.pipe(res);
    },
  );
  up.on('error', (e) => {
    res.writeHead(502, { 'content-type': 'application/json' });
    res.end(JSON.stringify({ error: 'upstream ' + e.code, robot: host, port }));
  });
  req.pipe(up);
}

function sendFile(file, type, cache, res, retried = false) {
  fs.readFile(file, (err, buf) => {
    if (err) {
      if (!retried) { setTimeout(() => sendFile(file, type, cache, res, true), 50); return; }
      console.log(`[static] 읽기 실패: ${file} — ${err.code || err.message}`);
      if (!res.headersSent) res.writeHead(500);
      return res.end();
    }
    if (res.destroyed) return;
    res.writeHead(200, { 'content-type': type, 'content-length': buf.length, 'cache-control': cache });
    res.end(buf);
  });
}

function serveStatic(pathname, res) {
  let rel = decodeURIComponent(pathname).replace(/\/+$/, '') || '/index';
  const tryFiles = [rel, rel + '.html', '/index.html'].map((p) => path.join(DIST, p));
  for (const f of tryFiles) {
    if (!f.startsWith(DIST)) continue;
    if (fs.existsSync(f) && fs.statSync(f).isFile()) {
      const ext = path.extname(f);
      const cache = ext === '.html' ? 'no-cache'
        : /-[0-9a-f]{8,}\.[a-z0-9]+$|\.[0-9a-f]{16,}\./.test(path.basename(f)) ? 'public, max-age=31536000, immutable'
        : 'no-cache';
      sendFile(f, MIME[ext] || 'application/octet-stream', cache, res);
      return;
    }
  }
  res.writeHead(404);
  res.end('not found');
}

const LOOPBACK_NAMES = new Set(['127.0.0.1', 'localhost', '[::1]']);
function localList(url, res) {
  const os = require('os');
  const home = os.homedir();
  const dir = path.resolve(url.searchParams.get('dir') || home);
  const roots = [['홈', home], ['다운로드', 'Downloads'], ['음악', 'Music'], ['문서', 'Documents'], ['바탕화면', 'Desktop']]
    .map(([label, p]) => ({ label, path: path.resolve(home, p) }))
    .filter((r) => { try { return fs.statSync(r.path).isDirectory(); } catch { return false; } });
  const media = path.join('/run/media', os.userInfo().username);
  try { for (const m of fs.readdirSync(media)) roots.push({ label: m, path: path.join(media, m) }); } catch { }
  let entries = [];
  try {
    entries = fs.readdirSync(dir, { withFileTypes: true }).filter((d) => !d.name.startsWith('.')).map((d) => {
      const full = path.join(dir, d.name);
      let st = null; try { st = fs.statSync(full); } catch { }
      return st && { name: d.name, dir: st.isDirectory(), size: st.isDirectory() ? 0 : st.size, mtime: st.mtimeMs };
    }).filter(Boolean).sort((a, b) => (a.dir === b.dir ? a.name.localeCompare(b.name) : a.dir ? -1 : 1));
  } catch (e) {
    res.writeHead(400, { 'Content-Type': 'application/json' });
    return res.end(JSON.stringify({ error: e.code || String(e) }));
  }
  res.writeHead(200, { 'Content-Type': 'application/json' });
  res.end(JSON.stringify({ dir, parent: path.dirname(dir) === dir ? null : path.dirname(dir), roots, entries }));
}

function localFile(url, res) {
  const p = path.resolve(url.searchParams.get('path') || '');
  let st; try { st = fs.statSync(p); } catch { res.writeHead(404); return res.end(); }
  if (!st.isFile()) { res.writeHead(400); return res.end('not a file'); }
  res.writeHead(200, { 'Content-Type': 'application/octet-stream', 'Content-Length': st.size });
  fs.createReadStream(p).on('error', () => res.destroy()).pipe(res);
}

function isLoopback(req) {
  const a = req.socket.remoteAddress || '';
  if (!(a === '127.0.0.1' || a === '::1' || a === '::ffff:127.0.0.1')) return false;
  const host = String(req.headers.host || '');
  if (!LOOPBACK_NAMES.has(host.replace(/:\d+$/, ''))) return false;
  const origin = req.headers.origin;
  if (!origin) return true;
  try { return new URL(origin).host === host; } catch { return false; }
}

function proxyTarget(req, res) {
  const json = (code, body) => {
    res.writeHead(code, { 'content-type': 'application/json' });
    res.end(JSON.stringify(body));
  };
  if (!isLoopback(req)) return json(403, { error: 'loopback only' });
  if (req.method === 'GET') return json(200, { robot: ROBOT, vision: ROBOT_VISION, hasToken: RV_TOKEN !== '' });
  if (req.method !== 'POST') return json(405, { error: 'GET or POST' });

  const ct = String(req.headers['content-type'] || '').split(';')[0].trim().toLowerCase();
  if (ct !== 'application/json') return json(415, { error: 'content-type: application/json' });

  let raw = '';
  req.on('data', (c) => {
    raw += c;
    if (raw.length > 4096) { req.destroy(); }
  });
  req.on('end', () => {
    let body;
    try { body = JSON.parse(raw || '{}'); } catch { return json(400, { error: 'bad json' }); }

    const host = (v) => {
      if (typeof v !== 'string') return null;
      const t = v.trim();
      return t && !/[\s/:@]/.test(t) ? t : null;
    };
    const robot = host(body.robot);
    if (!robot) return json(400, { error: 'robot: 호스트명 또는 IP 만' });
    const vision = body.vision === undefined || body.vision === '' ? robot : host(body.vision);
    if (!vision) return json(400, { error: 'vision: 호스트명 또는 IP 만' });

    if (typeof body.token === 'string') RV_TOKEN = body.token;

    const before = { robot: ROBOT, vision: ROBOT_VISION };
    ROBOT = robot;
    ROBOT_VISION = vision;

    console.log(`proxy target: ${before.robot}/${before.vision} → ${ROBOT}/${ROBOT_VISION}`);
    json(200, { robot: ROBOT, vision: ROBOT_VISION, hasToken: RV_TOKEN !== '' });
  });
}



const PIDFILE = path.join(require('os').tmpdir(), `rbq-proxy-${PORT}.pid`);

function killStale() {
  let old = 0;
  try { old = Number(fs.readFileSync(PIDFILE, 'utf8').trim()); } catch { return; }
  if (!old || old === process.pid) return;
  try {
    const cmd = fs.readFileSync(`/proc/${old}/cmdline`, 'utf8');
    if (!cmd.includes('rbq-proxy') && !cmd.includes('rbq-web-proxy')) return;
    process.kill(old, 'SIGTERM');
    console.log(`rbq-web-proxy: 옛 인스턴스 정리 (pid ${old})`);
    const until = Date.now() + 3000;
    const nap = () => Atomics.wait(new Int32Array(new SharedArrayBuffer(4)), 0, 0, 100);
    while (Date.now() < until) {
      try { process.kill(old, 0); } catch { return; }
      nap();
    }
    try { process.kill(old, 'SIGKILL'); } catch { }
  } catch { }
}

let byeSent = false;
function shutdown(why) {
  if (byeSent) return;
  byeSent = true;
  console.log(`rbq-web-proxy: 종료 (${why})`);
  try { fs.unlinkSync(PIDFILE); } catch { }
  try { server.close(); } catch { }
  process.exit(0);
}
for (const sig of ['SIGTERM', 'SIGINT', 'SIGHUP']) process.on(sig, () => shutdown(sig));

if (process.argv.includes('--exit-with-parent')) {
  const parent0 = process.ppid;
  setInterval(() => {
    if (process.ppid !== parent0 || process.ppid <= 1) shutdown('부모 종료');
  }, 2000).unref();
}

killStale();
try { fs.writeFileSync(PIDFILE, String(process.pid)); } catch { }

server.listen(PORT, HOST, () => {
  console.log(`rbq-web-proxy: http://localhost:${PORT} (bind ${HOST})  →  motion ${ROBOT} / vision ${ROBOT_VISION} (:${ROBOT_PORT}/:${VISION_PORT})`);
  console.log(`  dist: ${DIST}${fs.existsSync(path.join(DIST, 'index.html')) ? '' : '  (⚠ index.html 없음 — npx expo export --platform web 먼저)'}`);
  try { ctrlPruneStale(); } catch (e) { console.log(`[app-update] 정리 실패: ${e.message || e}`); }
});
