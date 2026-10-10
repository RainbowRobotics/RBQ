#!/usr/bin/env node
const http = require('http');
const https = require('https');
const fs = require('fs');
const path = require('path');
const { spawn } = require('child_process');
const WebSocket = require('ws');
const WebSocketServer = WebSocket.WebSocketServer || WebSocket.Server;

const FFMPEG = process.env.RBQ_FFMPEG || (() => {
  try { const p = path.join(path.dirname(process.execPath), 'ffmpeg'); return fs.existsSync(p) ? p : 'ffmpeg'; } catch { return 'ffmpeg'; }
})();

let wrtc = null;
{
  const req = require;
  for (const c of [
    path.join(path.dirname(process.execPath), 'wrtc-modules', 'node_modules', '@roamhq', 'wrtc'),
    '@roamhq/wrtc',
  ]) {
    try { wrtc = req(c); break; } catch { }
  }
}
let werift = null;
try { werift = require('werift'); } catch { }

let robotAuthHeader = '';
function authHeaders(req) {
  const a = (req && req.headers.authorization) || robotAuthHeader;
  return a ? { Authorization: a } : {};
}
const WRTC_CH = ['log', 'motion-state', 'command', 'estop'];
const extChannels = (req) => {
  const raw = new URL(req.url, 'http://x').searchParams.get('ext');
  const ext = raw ? raw.split(',') : [];
  return ext.length <= 4 && ext.every((l) => /^[a-z0-9-]{1,32}$/.test(l) && !WRTC_CH.includes(l)) && new Set(ext).size === ext.length ? ext : null;
};

function filterLoopbackCandidates(sdp, host) {
  const h = String(host);
  if (/^127\.|^localhost$/.test(h)) {
    return String(sdp).split(/\r?\n/)
      .filter((l) => !l.startsWith('a=candidate:') || /(127\.0\.0\.1|::1)/.test(l))
      .join('\r\n');
  }
  const m = h.match(/^(\d+\.\d+\.\d+)\.\d+$/);
  if (!m) return sdp;
  const prefix = m[1] + '.';
  const lines = String(sdp).split(/\r?\n/);
  const kept = lines.filter((l) => !l.startsWith('a=candidate:') || l.includes(' ' + prefix.slice(0, -1)) || l.split(' ')[4]?.startsWith(prefix));
  const candKept = kept.filter((l) => l.startsWith('a=candidate:')).length;
  const candAll = lines.filter((l) => l.startsWith('a=candidate:')).length;
  return candKept > 0 && candKept < candAll ? kept.join('\r\n') : sdp;
}

const VID_T = { FRAME: 0x00, COMMAND: 0x01, AUDIO: 0x02, MIC: 0x03, VSTATE: 0x04, PCD: 0x05, ELEVATION: 0x06, WALLMAP: 0x07, CTRL: 0xff };

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
const HOST = argOf('--host', '127.0.0.1');
const LOCAL_ADDR = argOf('--local-address', '');
const REVIEW = argOf('--review', process.env.RBQ_REVIEW || '');
const DIST = path.resolve(__dirname, '..', argOf('--dist', 'dist'));

const RV_URL = argOf('--rendezvous', '');
const RV_ROBOT = argOf('--robot-id', '');
let RV_TOKEN = process.env.RBQ_WEBRTC_TOKEN || argOf('--webrtc-token', '');
const USE_RV = RV_URL.trim() !== '' && RV_ROBOT.trim() !== '';
if (RV_URL.trim() !== '' && RV_ROBOT.trim() === '')
  console.warn('[proxy] --rendezvous 는 주어졌지만 --robot-id 가 비어 랑데부를 쓰지 않습니다 (HTTP 직결).');

function rvIceUrl() { return RV_URL.replace(/^ws/, 'http').replace(/\/ws$/, '/ice'); }

function fetchIceServers() {
  return new Promise((resolve) => {
    if (!USE_RV) return resolve([]);
    try {
      const u = new URL(rvIceUrl()); u.searchParams.set('id', RV_ROBOT);
      const mod = u.protocol === 'https:' ? https : http;
      const rq = mod.request(u, { method: 'GET' }, (res) => {
        let d = ''; res.on('data', (c) => { d += c; });
        res.on('end', () => {
          try { const j = JSON.parse(d); resolve(Array.isArray(j.iceServers) && j.iceServers.length ? j.iceServers : [{ urls: 'stun:stun.cloudflare.com:3478' }]); }
          catch { resolve([{ urls: 'stun:stun.cloudflare.com:3478' }]); }
        });
      });
      rq.on('error', () => resolve([{ urls: 'stun:stun.cloudflare.com:3478' }]));
      rq.setTimeout(4000, () => { rq.destroy(); resolve([{ urls: 'stun:stun.cloudflare.com:3478' }]); });
      rq.end();
    } catch { resolve([{ urls: 'stun:stun.cloudflare.com:3478' }]); }
  });
}

function rendezvousExchange(service, clientId, sdp, timeoutMs = 8000) {
  return new Promise((resolve, reject) => {
    let done = false; let timer = null; let ws;
    const finish = (err, ans) => { if (done) return; done = true; if (timer) clearTimeout(timer); try { ws && ws.close(); } catch {} err ? reject(err) : resolve(ans); };
    try { ws = new WebSocket(RV_URL); } catch (e) { return reject(new Error('랑데부 WS 생성 실패: ' + e.message)); }
    timer = setTimeout(() => finish(new Error('랑데부 타임아웃')), timeoutMs);
    ws.on('open', () => ws.send(JSON.stringify({ type: 'offer', robot: RV_ROBOT, service, clientId, sdp, token: RV_TOKEN })));
    ws.on('message', (data) => {
      let m; try { m = JSON.parse(data.toString()); } catch { return; }
      if (m && m.type === 'answer') { if (!m.sdp) return finish(new Error('랑데부 answer에 sdp 없음')); finish(null, { sdp: String(m.sdp), features: m.features }); }
      else if (m && m.type === 'error') finish(new Error('랑데부 오류: ' + (m.reason || '알 수 없음')));
    });
    ws.on('error', () => finish(new Error('랑데부 WS 오류')));
    ws.on('close', () => finish(new Error('랑데부 WS 닫힘 (answer 전)')));
  });
}

const MIME = {
  '.html': 'text/html', '.js': 'text/javascript', '.css': 'text/css', '.json': 'application/json',
  '.png': 'image/png', '.jpg': 'image/jpeg', '.svg': 'image/svg+xml', '.ico': 'image/x-icon',
  '.glb': 'model/gltf-binary', '.woff2': 'font/woff2', '.ttf': 'font/ttf', '.map': 'application/json',
};

const NET = { rx: 0, tx: 0 };
const rtpLen = (pkt) => (pkt?.payload?.length ?? 0) + 12;

const server = http.createServer((req, res) => {
  const dl = req.headers['x-download-base'];
  if (dl && !S3_FIXED && isLoopback(req) && /^https:\/\/[^/]+$/.test(dl)) S3_BASE = dl;
  const url = new URL(req.url, 'http://x');
  if (['/s3', '/fw-download', '/app-update'].includes(url.pathname) && !S3_BASE) { res.writeHead(404); return res.end('download base not set'); }
  if (url.pathname === '/wifi-settings') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    const gaming = !!process.env.GAMESCOPE_WAYLAND_DISPLAY;
    const cmd = gaming ? ['steam', ['-ifrunning', 'steam://open/settings']] : ['kcmshell6', ['kcm_networkmanagement']];
    const env = { ...process.env };
    delete env.LD_LIBRARY_PATH;
    delete env.LD_PRELOAD;
    try {
      spawn(cmd[0], cmd[1], { stdio: 'ignore', detached: true, env })
        .on('error', (e) => console.error('[wifi-settings] spawn 실패:', cmd[0], e.message));
    } catch (e) { console.error('[wifi-settings] spawn 예외:', e.message); }
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
    if (process.env.GAMESCOPE_WAYLAND_DISPLAY) {
      const env = { ...process.env };
      delete env.LD_LIBRARY_PATH;
      delete env.LD_PRELOAD;
      try {
        spawn('gamescopectl', ['touch_click_mode', '4'], { stdio: 'ignore', detached: true, env })
          .on('error', (e) => console.error('[touch-native] spawn 실패:', e.message));
      } catch (e) { console.error('[touch-native] spawn 예외:', e.message); }
    }
    res.writeHead(204); return res.end();
  }
  if (url.pathname === '/open-url') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    const target = url.searchParams.get('u') || '';
    if (!/^https?:\/\//.test(target)) { res.writeHead(400); return res.end(); }
    const gaming = !!process.env.GAMESCOPE_WAYLAND_DISPLAY;
    const cmd = gaming ? ['steam', ['-ifrunning', `steam://openurl/${target}`]] : ['xdg-open', [target]];
    const env = { ...process.env };
    delete env.LD_LIBRARY_PATH;
    delete env.LD_PRELOAD;
    try {
      spawn(cmd[0], cmd[1], { stdio: 'ignore', detached: true, env })
        .on('error', (e) => console.error('[open-url] spawn 실패:', cmd[0], e.message));
    } catch (e) { console.error('[open-url] spawn 예외:', e.message); }
    res.writeHead(204); return res.end();
  }
  if (url.pathname === '/osk') {
    const env = { ...process.env };
    delete env.LD_LIBRARY_PATH;
    delete env.LD_PRELOAD;
    try { spawn('steam', ['-ifrunning', 'steam://open/keyboard'], { stdio: 'ignore', detached: true, env }).on('error', () => {}); } catch { }
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
    let steamos = false;
    try { steamos = require('fs').readFileSync('/etc/os-release', 'utf8').includes('ID=steamos'); } catch { }
    const gaming = !!process.env.GAMESCOPE_WAYLAND_DISPLAY;
    res.writeHead(200, { 'Content-Type': 'application/json' });
    return res.end(JSON.stringify({ steamos, gaming }));
  }
  if (url.pathname === '/local/list' || url.pathname === '/local/file') {
    if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
    if (!process.env.GAMESCOPE_WAYLAND_DISPLAY) { res.writeHead(404); return res.end(); }
    return url.pathname === '/local/list' ? localList(url, res) : localFile(url, res);
  }
  if (url.pathname === '/netstats') {
    res.writeHead(200, { 'Content-Type': 'application/json' });
    return res.end(JSON.stringify(NET));
  }
  if (url.pathname === '/host-battery') {
    const fs2 = require('fs');
    let out = { present: false };
    try {
      const base = '/sys/class/power_supply';
      const bat = fs2.readdirSync(base).find((n) => /^BAT/i.test(n));
      if (bat) {
        const rd = (f) => { try { return fs2.readFileSync(`${base}/${bat}/${f}`, 'utf8').trim(); } catch { return ''; } };
        const pct = parseInt(rd('capacity'), 10);
        const status = rd('status').trim();
        if (Number.isFinite(pct)) out = { present: true, percent: pct, charging: status === 'Charging' };
      }
    } catch { }
    res.writeHead(200, { 'Content-Type': 'application/json' });
    return res.end(JSON.stringify(out));
  }
  if (url.pathname === '/app-update' && !isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
  if (url.pathname === '/app-update') return appSelfUpdate(url, res);
  if (url.pathname === '/whoami') {
    if (!process.env.RBQ_PROXY_TOKEN) { res.writeHead(501); return res.end('no identity'); }
    return jsonRes(res, 200, { token: process.env.RBQ_PROXY_TOKEN, pid: process.pid });
  }
  if (url.pathname === '/s3') return s3Read(req, url, res);
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
  serveStatic(url.pathname, res);
});

let S3_BASE = argOf('--download-base', process.env.RBQ_DOWNLOAD_BASE || '');
const S3_FIXED = !!S3_BASE;
const CTRL_ASSET = 'rbq-controller-x86_64.AppImage';
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

function s3Read(req, url, res) {
  if (!isLoopback(req)) { res.writeHead(403); return res.end('loopback only'); }
  const p = url.searchParams.get('path') || '';
  const ok = !p.includes('..') && (p === 'index.json'
    || p === 'nightly/nightly.txt'
    || /^[A-Za-z0-9._-]+\/release-notes\.json$/.test(p));
  if (!ok) return jsonRes(res, 400, { error: 'bad path' });
  const fail = (why) => {
    if (res.headersSent) return res.destroy();
    res.writeHead(502); res.end(String(why));
  };
  const rq = https.get(`${S3_BASE}/${p}`, { timeout: 15000 }, (r) => {
    res.writeHead(r.statusCode || 502, { 'Content-Type': r.headers['content-type'] || 'application/json' });
    r.on('error', fail);
    r.pipe(res);
  });
  rq.on('timeout', () => rq.destroy(new Error('S3 timeout')));
  rq.on('error', fail);
}

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
  if (!process.env.APPIMAGE) { res.writeHead(501); return res.end('not running from an AppImage'); }
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
  if (!process.env.APPIMAGE) { res.writeHead(501); return res.end('not running from an AppImage'); }
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
  if (url.searchParams.get('check')) { res.writeHead(appimage ? 204 : 501); return res.end(); }
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

const wss = new WebSocketServer({ noServer: true });

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

    let closed = 0;
    for (const c of wss.clients) { try { c.close(4001, 'target changed'); closed++; } catch { } }

    console.log(`proxy target: ${before.robot}/${before.vision} → ${ROBOT}/${ROBOT_VISION} (연결 ${closed}개 정리)`);
    json(200, { robot: ROBOT, vision: ROBOT_VISION, hasToken: RV_TOKEN !== '', closed });
  });
}


server.on('upgrade', (req, socket, head) => {
  if (!isLoopback(req)) return socket.destroy();
  const { pathname, searchParams } = new URL(req.url, 'http://x');
  if (pathname === '/webrtc') {
    if (!wrtc) return socket.destroy();
    const ext = extChannels(req);
    if (!ext) return socket.destroy();
    wss.handleUpgrade(req, socket, head, (client) => bridgeWebrtc(client, ext).catch((e) => console.warn('[proxy] webrtc 브리지 실패', e)));
  } else if (pathname === '/webrtc-video') {
    if (!werift) return socket.destroy();
    const own = searchParams.get('own') === '1';
    wss.handleUpgrade(req, socket, head, (client) => bridgeWebrtcVideo(client, own).catch((e) => console.warn('[proxy] webrtc-video 브리지 실패', e)));
  } else {
    socket.destroy();
  }
});


async function bridgeWebrtc(client, ext = []) {
  const channels = [...WRTC_CH, ...ext];
  let closed = false;
  const onGone = () => { closed = true; };
  client.on('close', onGone); client.on('error', onGone);
  const iceServers = await fetchIceServers();
  if (closed || client.readyState !== WebSocket.OPEN) return;
  const pc = new wrtc.RTCPeerConnection(USE_RV ? { iceServers } : undefined);
  const dcs = {};
  const ctrl = (obj) => { if (client.readyState === WebSocket.OPEN) client.send(Buffer.concat([Buffer.from([0xff]), Buffer.from(JSON.stringify(obj))]), { binary: true }); };

  channels.forEach((label, id) => {
    const dc = pc.createDataChannel(label, label === 'motion-state' ? { ordered: false, maxRetransmits: 0 } : {});
    try { dc.binaryType = 'arraybuffer'; } catch { }
    dcs[label] = dc;
    dc.onopen = () => ctrl({ t: 'open', ch: label });
    dc.onclose = () => ctrl({ t: 'close', ch: label });
    dc.onmessage = (e) => {
      const p = typeof e.data === 'string' ? Buffer.from(e.data) : Buffer.from(e.data);
      NET.rx += p.length;
      if (client.readyState !== WebSocket.OPEN) return;
      client.send(Buffer.concat([Buffer.from([id]), p]), { binary: true });
    };
  });

  pc.onconnectionstatechange = () => ctrl({ t: 'pc', state: pc.connectionState });

  client.on('message', (data) => {
    const buf = Buffer.isBuffer(data) ? data : Buffer.from(data);
    if (buf.length < 1) return;
    const label = channels[buf[0]];
    const dc = dcs[label];
    if (!dc || dc.readyState !== 'open') return;
    const payload = buf.subarray(1);
    const asText = label === 'command' || label === 'estop' || ext.includes(label);
    try { dc.send(asText ? payload.toString('utf8') : payload); NET.tx += payload.length; } catch { }
  });

  const cleanup = () => { try { pc.close(); } catch { } try { client.close(); } catch { } };
  client.off('close', onGone); client.off('error', onGone);
  client.on('close', cleanup); client.on('error', cleanup);

  (async () => {
    try {
      await pc.setLocalDescription(await pc.createOffer());
      const sdp = await gatheredSdp(pc);
      const ans = USE_RV
        ? await rendezvousExchange('motion', 'desktop-proxy-' + process.pid, sdp)
        : await postOfferToRobot(filterLoopbackCandidates(sdp, ROBOT));
      if (!ans || !ans.sdp) throw new Error('answer sdp 없음');
      await pc.setRemoteDescription({ type: 'answer', sdp: ans.sdp });
    } catch (e) { ctrl({ t: 'error', msg: String((e && e.message) || e) }); cleanup(); }
  })();
}

function gatheredSdp(pc) {
  return new Promise((resolve) => {
    if (pc.iceGatheringState === 'complete') return resolve(pc.localDescription.sdp);
    const check = () => { if (pc.iceGatheringState === 'complete') { pc.removeEventListener('icegatheringstatechange', check); resolve(pc.localDescription.sdp); } };
    pc.addEventListener('icegatheringstatechange', check);
    setTimeout(() => resolve(pc.localDescription ? pc.localDescription.sdp : ''), 2500);
  });
}

function postOfferToRobot(sdp) {
  return new Promise((resolve, reject) => {
    const body = JSON.stringify({ sdp, takeover: true, clientId: 'desktop-proxy-' + process.pid, token: RV_TOKEN });
    const rq = http.request({ host: ROBOT, port: ROBOT_PORT, path: '/api/webrtc/offer', method: 'POST', ...(LOCAL_ADDR ? { localAddress: LOCAL_ADDR } : {}),
      headers: { ...authHeaders(), 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(body) } },
      (res) => { let d = ''; res.on('data', (c) => { d += c; }); res.on('end', () => { try { resolve(JSON.parse(d)); } catch (e) { reject(e); } }); });
    rq.on('error', reject); rq.write(body); rq.end();
  });
}

function postOfferToVision(sdp, takeover) {
  return new Promise((resolve, reject) => {
    const body = JSON.stringify({ sdp, takeover: !!takeover, spectate: !takeover, clientId: 'desktop-proxy-vid-' + process.pid, token: RV_TOKEN });
    const rq = http.request({ host: ROBOT_VISION, port: VISION_PORT, path: '/api/webrtc/offer', method: 'POST', ...(LOCAL_ADDR ? { localAddress: LOCAL_ADDR } : {}),
      headers: { ...authHeaders(), 'Content-Type': 'application/json', 'Content-Length': Buffer.byteLength(body) } },
      (res) => { let d = ''; res.on('data', (c) => { d += c; }); res.on('end', () => { try { resolve(JSON.parse(d)); } catch (e) { reject(e); } }); });
    rq.on('error', reject); rq.write(body); rq.end();
  });
}

async function bridgeWebrtcVideo(client, takeover) {
  const { RTCPeerConnection, useH264, RTCRtpCodecParameters, MediaStreamTrack } = werift;
  let closed = false;
  const onGone = () => { closed = true; };
  client.on('close', onGone); client.on('error', onGone);
  const iceServers = await fetchIceServers();
  if (closed || client.readyState !== WebSocket.OPEN) return;
  const h264Codec = useH264();
  h264Codec.payloadType = 96;
  const opusCodec = new RTCRtpCodecParameters({ mimeType: 'audio/OPUS', clockRate: 48000, channels: 2, payloadType: 111 });
  const pc = new RTCPeerConnection(
    USE_RV ? { codecs: { video: [h264Codec], audio: [opusCodec] }, iceServers, maxMessageSize: 4 * 1024 * 1024 }
           : { codecs: { video: [h264Codec], audio: [opusCodec] }, maxMessageSize: 4 * 1024 * 1024 });
  let videoTrack = null, transceiver = null;
  const ctrl = (obj) => { if (client.readyState === WebSocket.OPEN) client.send(Buffer.concat([Buffer.from([VID_T.CTRL]), Buffer.from(JSON.stringify(obj))]), { binary: true }); };

  const ff = spawnKid(FFMPEG, ['-hide_banner', '-loglevel', 'error',
    '-probesize', '32', '-analyzeduration', '0', '-flags', 'low_delay',
    '-f', 'h264', '-i', 'pipe:0', '-c:v', 'mjpeg', '-q:v', '6', '-flush_packets', '1', '-f', 'image2pipe', 'pipe:1']);
  let acc = Buffer.alloc(0);
  ff.stdout.on('data', (chunk) => {
    acc = acc.length ? Buffer.concat([acc, chunk]) : chunk;
    for (;;) {
      const soi = acc.indexOf('\xff\xd8', 0, 'binary');
      if (soi < 0) { if (acc.length > 4 << 20) acc = Buffer.alloc(0); break; }
      const eoi = acc.indexOf('\xff\xd9', soi + 2, 'binary');
      if (eoi < 0) { if (soi > 0) acc = acc.subarray(soi); break; }
      const jpeg = acc.subarray(soi, eoi + 2);
      acc = acc.subarray(eoi + 2);
      if (client.readyState === WebSocket.OPEN && client.bufferedAmount <= 8 * 1024 * 1024)
        client.send(Buffer.concat([Buffer.from([VID_T.FRAME]), jpeg]), { binary: true });
    }
  });
  ff.stderr.on('data', () => {});
  ff.on('error', (e) => ctrl({ t: 'error', msg: 'ffmpeg ' + e.message }));
  const _dumpFd = process.env.RBQ_VID_DUMP ? require('fs').openSync(process.env.RBQ_VID_DUMP, 'w') : null;
  const safeWrite = (b) => { try { ff.stdin.write(b); } catch {} if (_dumpFd) { try { require('fs').writeSync(_dumpFd, b); } catch {} } };

  const SC = Buffer.from([0, 0, 0, 1]);
  let sps = null, pps = null, started = false;

  const { dePacketizeRtpPackets } = werift;
  const seqLt = (a, b) => { const d = (b - a) & 0xffff; return d !== 0 && d < 0x8000; };
  const store = new Map();
  let next = null;
  let framePkts = [];
  let gapTimer = null, lastPli = 0;
  const MAX_STORE = 1024, GAP_MS = 150;
  const splitNals = (annexB) => {
    const out = [];
    let sc = -1;
    for (let k = 0; k + 4 <= annexB.length; k++) if (annexB[k] === 0 && annexB[k + 1] === 0 && annexB[k + 2] === 0 && annexB[k + 3] === 1) { sc = k; break; }
    while (sc >= 0) {
      let nsc = -1;
      for (let k = sc + 4; k + 4 <= annexB.length; k++) if (annexB[k] === 0 && annexB[k + 1] === 0 && annexB[k + 2] === 0 && annexB[k + 3] === 1) { nsc = k; break; }
      out.push(annexB.subarray(sc + 4, nsc < 0 ? annexB.length : nsc));
      sc = nsc;
    }
    return out;
  };
  const emitFrame = (pkts) => {
    if (!pkts.length) return;
    let res; try { res = dePacketizeRtpPackets('MPEG4/ISO/AVC', pkts); } catch { return; }
    if (!res || !res.data || !res.data.length) return;
    const nals = splitNals(res.data);
    let hasIdr = false, hasSps = false, hasPps = false, hasVcl = false;
    for (const nal of nals) {
      const t = nal[0] & 0x1f;
      if (t === 7) { sps = Buffer.concat([SC, nal]); hasSps = true; }
      else if (t === 8) { pps = Buffer.concat([SC, nal]); hasPps = true; }
      else if (t === 5) { hasIdr = true; hasVcl = true; }
      else if (t >= 1 && t <= 5) hasVcl = true;
    }
    if (hasIdr) {
      if (!sps || !pps) return;
      started = true;
      if (!(hasSps && hasPps)) safeWrite(Buffer.concat([sps, pps]));
      safeWrite(res.data);
      ctrl({ t: 'idr' });
      return;
    }
    if (!hasVcl) return;
    if (!started) return;
    safeWrite(res.data);
  };
  const requestKeyframe = () => {
    const now = Date.now();
    if (now - lastPli < 1000 || !videoTrack || !transceiver) return;
    lastPli = now;
    try { transceiver.receiver.sendRtcpPLI(videoTrack.ssrc); } catch {}
  };
  const drain = () => {
    while (store.has(next)) {
      const p = store.get(next); store.delete(next);
      if (framePkts.length && framePkts[0].header.timestamp !== p.header.timestamp) { emitFrame(framePkts); framePkts = []; }
      framePkts.push(p);
      next = (next + 1) & 0xffff;
    }
  };
  const skipGap = () => {
    if (store.has(next) || store.size === 0) return;
    let cand = null;
    for (const s of store.keys()) if (cand === null || seqLt(s, cand)) cand = s;
    if (cand !== null) { _dbgSkip++; next = cand; framePkts = []; requestKeyframe(); drain(); }
  };
  let _dbgRx = 0, _dbgFirst = null, _dbgLast = null, _dbgSkip = 0, _dbgT0 = Date.now();
  const onPkt = (pkt) => {
    const s = pkt.header.sequenceNumber;
    if (process.env.RBQ_VID_DEBUG) {
      _dbgRx++; if (_dbgFirst === null) _dbgFirst = s; _dbgLast = s;
      if (Date.now() - _dbgT0 > 3000) {
        const span = ((_dbgLast - _dbgFirst) & 0xffff) + 1;
        const lossPct = span > 0 ? (100 * (1 - _dbgRx / span)).toFixed(1) : '?';
        process.stderr.write(`[vid] rx=${_dbgRx} span=${span} loss≈${lossPct}% skipGap=${_dbgSkip}\n`);
        _dbgRx = 0; _dbgFirst = null; _dbgSkip = 0; _dbgT0 = Date.now();
      }
    }
    if (next === null) next = s;
    if (seqLt(s, next)) return;
    store.set(s, pkt);
    drain();
    if (store.size > MAX_STORE) skipGap();
    if (!store.has(next) && store.size > 0) {
      if (gapTimer) clearTimeout(gapTimer);
      gapTimer = setTimeout(() => { gapTimer = null; skipGap(); }, GAP_MS);
    }
  };

  const os = require('os');
  const dgram = require('dgram');
  const AUD_DOWN_PORT = 40000 + (process.pid % 10000) * 2;
  const AUD_UP_PORT = AUD_DOWN_PORT + 2;
  const micTrack = new MediaStreamTrack({ kind: 'audio' });
  let audDownFf = null, audDownUdp = null, audUpFf = null, audUpUdp = null, audSdpPath = null, audUpPace = null;

  const startAudioDown = () => {
    if (audDownFf) return;
    audSdpPath = path.join(os.tmpdir(), `rbq-aud-${process.pid}-${AUD_DOWN_PORT}.sdp`);
    fs.writeFileSync(audSdpPath, ['v=0', 'o=- 0 0 IN IP4 127.0.0.1', 's=rbq-audio', 'c=IN IP4 127.0.0.1', 't=0 0',
      `m=audio ${AUD_DOWN_PORT} RTP/AVP 111`, 'a=rtpmap:111 opus/48000/2', ''].join('\n'));
    audDownUdp = dgram.createSocket('udp4');
    audDownFf = spawnKid(FFMPEG, ['-hide_banner', '-loglevel', 'error',
      '-protocol_whitelist', 'file,udp,rtp', '-i', audSdpPath,
      '-af', 'aresample=async=1', '-f', 's16le', '-ar', '48000', '-ac', '1', 'pipe:1']);
    audDownFf.stdout.on('data', (pcm) => {
      if (client.readyState === WebSocket.OPEN && client.bufferedAmount <= 8 * 1024 * 1024)
        client.send(Buffer.concat([Buffer.from([VID_T.AUDIO]), pcm]), { binary: true });
    });
    audDownFf.stderr.on('data', () => {});
    audDownFf.on('error', (e) => ctrl({ t: 'error', msg: 'audio ffmpeg ' + e.message }));
  };
  const startAudioUp = () => {
    if (audUpFf) return;
    audUpUdp = dgram.createSocket('udp4');
    let upN = 0, upErr = 0;
    const upQ = [];
    audUpUdp.on('message', (buf) => { upQ.push(buf); if (upQ.length > 10) upQ.shift(); });
    audUpPace = setInterval(() => {
      const buf = upQ.shift();
      if (!buf) return;
      try {
        micTrack.writeRtp(buf);
        NET.tx += buf.length;
        if (++upN === 1) console.log('[aud] 업링크 RTP 송출 시작');
      } catch (e) { if (++upErr <= 3) console.log('[aud] writeRtp 실패:', String(e && e.message || e)); }
    }, 20);
    audUpUdp.bind(AUD_UP_PORT, '127.0.0.1');
    audUpFf = spawnKid(FFMPEG, ['-hide_banner', '-loglevel', 'error', '-probesize', '32', '-analyzeduration', '0',
      '-f', 's16le', '-ar', '48000', '-ac', '1', '-i', 'pipe:0',
      '-c:a', 'libopus', '-b:a', '32k', '-application', 'voip', '-frame_duration', '20',
      '-payload_type', '111', '-flush_packets', '1', '-max_delay', '0', '-f', 'rtp', `rtp://127.0.0.1:${AUD_UP_PORT}`]);
    audUpFf.stderr.on('data', (d) => console.log('[aud] 인코더:', String(d).slice(0, 200)));
    audUpFf.on('error', (e) => ctrl({ t: 'error', msg: 'mic ffmpeg ' + e.message }));
  };
  const stopAudio = () => {
    if (audUpPace) { clearInterval(audUpPace); audUpPace = null; }
    for (const p of [audDownFf, audUpFf]) { if (p) { try { p.stdin.end(); } catch {} try { p.kill('SIGKILL'); } catch {} } }
    for (const s of [audDownUdp, audUpUdp]) { if (s) { try { s.close(); } catch {} } }
    if (audSdpPath) { try { fs.unlinkSync(audSdpPath); } catch {} }
    audDownFf = audUpFf = audDownUdp = audUpUdp = audSdpPath = null;
  };

  const vsDc = pc.createDataChannel('vision-state');
  vsDc.stateChanged.subscribe((s) => { if (s === 'open') ctrl({ t: 'open', ch: 'vision-state' }); else if (s === 'closed') ctrl({ t: 'close', ch: 'vision-state' }); });
  vsDc.onMessage.subscribe((msg) => {
    if (client.readyState !== WebSocket.OPEN) return;
    const p = Buffer.isBuffer(msg) ? msg : Buffer.from(msg);
    client.send(Buffer.concat([Buffer.from([VID_T.VSTATE]), p]), { binary: true });
  });

  const cmdDc = pc.createDataChannel('command');
  cmdDc.stateChanged.subscribe((s) => { if (s === 'open') ctrl({ t: 'open', ch: 'command' }); else if (s === 'closed') ctrl({ t: 'close', ch: 'command' }); });
  cmdDc.onMessage.subscribe((msg) => {
    const p = typeof msg === 'string' ? Buffer.from(msg) : Buffer.from(msg);
    NET.rx += p.length;
    if (client.readyState !== WebSocket.OPEN) return;
    client.send(Buffer.concat([Buffer.from([VID_T.COMMAND]), p]), { binary: true });
  });

  const pcdDc = pc.createDataChannel('pointcloud');
  pcdDc.stateChanged.subscribe((st) => { if (st === 'open') ctrl({ t: 'open', ch: 'pointcloud' }); else if (st === 'closed') ctrl({ t: 'close', ch: 'pointcloud' }); });
  pcdDc.onMessage.subscribe((msg) => {
    if (typeof msg === 'string') return;
    NET.rx += msg.length;
    if (client.readyState !== WebSocket.OPEN) return;
    client.send(Buffer.concat([Buffer.from([VID_T.PCD]), Buffer.from(msg)]), { binary: true });
  });

  const elevationDc = pc.createDataChannel('elevation-map');
  elevationDc.onMessage.subscribe((msg) => {
    if (client.readyState !== WebSocket.OPEN) return;
    const p = Buffer.isBuffer(msg) ? msg : Buffer.from(msg);
    client.send(
      Buffer.concat([Buffer.from([VID_T.ELEVATION]), p]),
      { binary: true },
    );
  });

  elevationDc.stateChanged.subscribe((s) =>
    console.log('[map] elevation-map:', s),
  );

  const wallMapDc = pc.createDataChannel('wall-map');
  wallMapDc.onMessage.subscribe((msg) => {
    if (client.readyState !== WebSocket.OPEN) return;
    const p = Buffer.isBuffer(msg) ? msg : Buffer.from(msg);
    client.send(
      Buffer.concat([Buffer.from([VID_T.WALLMAP]), p]),
      { binary: true },
    );
  });

  wallMapDc.stateChanged.subscribe((s) =>
  console.log('[map] wall-map:', s),
  );

  pc.onTrack.subscribe((track) => {
    if (track.kind === 'audio') {
      startAudioDown();
      track.onReceiveRtp.subscribe((pkt) => {
        NET.rx += rtpLen(pkt);
        try { audDownUdp.send(pkt.serialize(), AUD_DOWN_PORT, '127.0.0.1'); } catch { }
      });
      return;
    }
    if (track.kind !== 'video') return;
    videoTrack = track;
    requestKeyframe();
    track.onReceiveRtp.subscribe((pkt) => { NET.rx += rtpLen(pkt); onPkt(pkt); });
  });
  pc.connectionStateChange.subscribe((state) => ctrl({ t: 'pc', state }));

  const cleanup = () => { if (gapTimer) { clearTimeout(gapTimer); gapTimer = null; } stopAudio(); try { ff.stdin.end(); } catch {} try { ff.kill('SIGKILL'); } catch {} try { pc.close(); } catch {} try { client.close(); } catch {} };
  client.on('message', (data) => {
    const buf = Buffer.isBuffer(data) ? data : Buffer.from(data);
    if (buf.length < 1) return;
    if (buf[0] === VID_T.COMMAND && cmdDc.readyState === 'open') { try { cmdDc.send(buf.subarray(1).toString('utf8')); } catch {} }
    else if (buf[0] === VID_T.VSTATE && vsDc.readyState === 'open') { try { vsDc.send(buf.subarray(1)); NET.tx += buf.length - 1; } catch {} }
    else if (buf[0] === VID_T.MIC) {
      if (!audUpFf) console.log('[aud] 첫 MIC 프레임 — 업링크 인코더 기동');
      startAudioUp();
      try { audUpFf.stdin.write(buf.subarray(1)); } catch (e) { console.log('[aud] stdin 실패:', String(e && e.message || e)); }
    }
  });
  client.off('close', onGone); client.off('error', onGone);
  client.on('close', cleanup); client.on('error', cleanup);

  (async () => {
    try {
      transceiver = pc.addTransceiver('video', { direction: 'recvonly' });
      pc.addTransceiver(micTrack, { direction: 'sendrecv' });
      await pc.setLocalDescription(await pc.createOffer());
      const ans = USE_RV
        ? await rendezvousExchange('vision', 'desktop-proxy-vid-' + process.pid, pc.localDescription.sdp)
        : await postOfferToVision(filterLoopbackCandidates(pc.localDescription.sdp, ROBOT_VISION), takeover);
      if (!ans || !ans.sdp) throw new Error('vision answer sdp 없음');
      await pc.setRemoteDescription({ type: 'answer', sdp: ans.sdp });
      try {
        const at = (pc.transceivers || []).find((t) => t.kind === 'audio');
        if (at) console.log('[aud] 협상:', at.direction, at.sender && at.sender.codec && at.sender.codec.mimeType);
      } catch { }
    } catch (e) { ctrl({ t: 'error', msg: String((e && e.message) || e) }); cleanup(); }
  })();
}

const PIDFILE = path.join(require('os').tmpdir(), `rbq-proxy-${PORT}.pid`);
const kids = new Set();

function spawnKid(cmd, args, opts) {
  const p = spawn(cmd, args, opts);
  kids.add(p);
  p.on('exit', () => kids.delete(p));
  return p;
}

function killKids() {
  for (const p of kids) { try { p.kill('SIGKILL'); } catch { } }
  kids.clear();
}

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
  killKids();
  try { fs.unlinkSync(PIDFILE); } catch { }
  try { server.close(); } catch { }
  process.exit(0);
}
for (const sig of ['SIGTERM', 'SIGINT', 'SIGHUP']) process.on(sig, () => shutdown(sig));
process.on('exit', killKids);

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
