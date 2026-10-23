const { app, BrowserWindow, ipcMain, session } = require('electron');
const { spawn } = require('child_process');
const fs = require('fs');
const http = require('http');
const net = require('net');
const path = require('path');
const wifi = require(process.platform === 'win32' ? './wifi-netsh' : './wifi');

const PORT_MIN = 8090;
const PORT_MAX = 8099;

const firstExisting = (...ps) => ps.find((p) => fs.existsSync(p)) || ps[0];
const PROXY_JS = firstExisting(path.join(__dirname, 'tools/rbq-web-proxy.js'), path.join(__dirname, '../tools/rbq-web-proxy.js'));
const DIST = firstExisting(path.join(__dirname, 'dist'), path.join(__dirname, '../dist'));

function isSteamOS() {
  for (const f of ['/run/host/os-release', '/etc/os-release']) {
    try { return /^ID=steamos$/m.test(fs.readFileSync(f, 'utf8')); } catch { }
  }
  return false;
}
const deck = isSteamOS();
const winInstalled = process.platform === 'win32'
  && fs.readdirSync(path.dirname(process.execPath)).some((f) => /^Uninstall .*\.exe$/i.test(f));

const primary = app.requestSingleInstanceLock();
if (!primary) {
  app.quit();
} else {
  app.on('second-instance', () => {
    if (!win) return;
    if (win.isMinimized()) win.restore();
    win.show();
    win.focus();
  });
}

let win = null;
let child = null;
let generation = 0;
let current = null;
let port = PORT_MIN;
let repicked = false;
let frontendBooted = false;
const token = `${process.pid.toString(16)}-${Date.now().toString(16)}`;

function send(event, payload) {
  if (win && !win.isDestroyed()) win.webContents.send('rbq-event', event, payload);
}

function portFree(p) {
  return new Promise((resolve) => {
    const s = net.createServer().once('error', () => resolve(false));
    s.listen(p, '127.0.0.1', () => s.close(() => resolve(true)));
  });
}
async function pickFreePort(base) {
  for (let p = base; p <= PORT_MAX; p++) if (await portFree(p)) return p;
  return null;
}

function proxyIsOurs(p) {
  return new Promise((resolve) => {
    const req = http.get({ host: '127.0.0.1', port: p, path: '/whoami', timeout: 700 }, (res) => {
      let body = '';
      res.on('data', (d) => { body += d; });
      res.on('end', () => resolve(body.includes(token)));
    });
    req.on('timeout', () => req.destroy());
    req.on('error', () => resolve(false));
  });
}
const sleep = (ms) => new Promise((r) => setTimeout(r, ms));

function spawnProxy(args) {
  const gen = ++generation;
  const argv = [PROXY_JS, '--robot', args.ip, '--robot-vision', args.visionIp || args.ip,
    '--port', String(port), '--host', '127.0.0.1', '--dist', DIST, '--exit-with-parent'];
  const env = { ...process.env, ELECTRON_RUN_AS_NODE: '1', RBQ_PROXY_TOKEN: token, RBQ_DESKTOP_SHELL: 'electron' };
  if (winInstalled) env.RBQ_WIN_INSTALLER = process.execPath;
  if (args.rendezvousUrl && args.robotId) {
    argv.push('--rendezvous', args.rendezvousUrl, '--robot-id', args.robotId);
    if (args.webrtcToken) env.RBQ_WEBRTC_TOKEN = args.webrtcToken;
  }
  const p = spawn(process.execPath, argv, { env, stdio: ['ignore', 'inherit', 'inherit'] });
  child = p;
  current = JSON.stringify(args);
  const myPort = port;

  (async () => {
    for (let i = 0; i < 300; i++) {
      if (gen !== generation || myPort !== port) return;
      if (await proxyIsOurs(myPort)) return send('proxy-status', { running: true });
      await sleep(100);
    }
    send('proxy-status', { running: false, detail: `프록시가 ${myPort}에 준비되지 않음(30초 초과)` });
  })();

  p.on('exit', async (code) => {
    if (gen !== generation) return;
    if (!repicked && !(await portFree(myPort)) && !(await proxyIsOurs(myPort))) {
      repicked = true;
      const next = await pickFreePort(myPort + 1);
      if (next) {
        port = next;
        console.error(`[proxy] ${myPort} 를 다른 프로그램이 쓰고 있음 → ${next}`);
        spawnProxy(args);
        if (win && !frontendBooted) void navigate();
        return;
      }
      return send('proxy-status', { running: false, detail: `사용 가능한 포트가 없습니다(${PORT_MIN}~${PORT_MAX})` });
    }
    send('proxy-status', { running: false, detail: `proxy exited: ${code}` });
  });
}

let restartChain = Promise.resolve();
let requested = null;
function restartProxy(args) {
  requested = args;
  restartChain = restartChain.then(async () => {
    const want = requested;
    if (JSON.stringify(want) === current && await proxyIsOurs(port)) return send('proxy-status', { running: true });
    generation++;
    if (child) child.kill();
    for (let i = 0; i < 40 && !(await portFree(port)); i++) await sleep(50);
    spawnProxy(want);
  });
}

async function navigate() {
  for (let i = 0; i < 300 && !(await proxyIsOurs(port)); i++) await sleep(100);
  if (!(await proxyIsOurs(port))) return console.error('[proxy] 30초 안에 우리 프록시가 뜨지 않음 — 빈 화면 유지');
  const q = process.env.RBQ_URL_QUERY ? `/?${process.env.RBQ_URL_QUERY}` : '';
  await win.loadURL(`http://localhost:${port}${q}`).catch((e) => console.error('[nav]', e.message));
}

function createWindow() {
  win = new BrowserWindow({
    width: 1280,
    height: 800,
    backgroundColor: '#0b0d10',
    autoHideMenuBar: true,
    fullscreen: deck || process.platform === 'win32',
    webPreferences: { preload: path.join(__dirname, 'preload.js'), contextIsolation: true, sandbox: false },
  });
  win.setMenu(null);
  win.webContents.on('did-finish-load', () => { if (deck && !zoomSet) win.webContents.setZoomFactor(1.25); });
  win.on('closed', () => { win = null; });

  const ours = (u) => u.startsWith(`http://localhost:${port}/`) || u === `http://localhost:${port}`;
  const toHost = (u) => { if (/^https?:\/\//.test(u)) http.get(`http://127.0.0.1:${port}/open-url?u=${encodeURIComponent(u)}`).on('error', () => {}); };
  win.webContents.setWindowOpenHandler(({ url }) => { toHost(url); return { action: 'deny' }; });
  win.webContents.on('will-navigate', (e, url) => { if (!ours(url)) { e.preventDefault(); toHost(url); } });

  let loadFails = 0;
  win.webContents.on('did-finish-load', () => { loadFails = 0; });
  win.webContents.on('did-fail-load', (_e, code, _desc, _url, isMainFrame) => {
    if (!isMainFrame || code === -3 || ++loadFails > 3) return;
    setTimeout(() => { if (win) void navigate(); }, 1000 * loadFails);
  });
  win.webContents.on('render-process-gone', (_e, d) => {
    console.error('[renderer] gone:', d.reason);
    if (d.reason !== 'clean-exit' && win && ++loadFails <= 3) void navigate();
  });
  void navigate();
}
let zoomSet = false;

ipcMain.on('rbq-event', async (_e, event, payload = {}) => {
  switch (event) {
    case 'restart-proxy':
      frontendBooted = true;
      return restartProxy({
        ip: payload.ip, visionIp: (payload.visionIp || '').trim() || payload.ip,
        rendezvousUrl: payload.rendezvousUrl || '', robotId: payload.robotId || '', webrtcToken: payload.webrtcToken || '',
      });
    case 'request-proxy-status': {
      const running = await proxyIsOurs(port);
      return send('proxy-status', { running, detail: running ? '' : '프록시 미응답(상태 요청)' });
    }
    case 'toggle-fullscreen':
      return win && win.setFullScreen(!win.isFullScreen());
    case 'app-quit':
      return app.quit();
    case 'ui-zoom': {
      const s = typeof payload.scale === 'number' ? Math.min(2, Math.max(1, payload.scale)) : null;
      zoomSet = s !== null;
      return win && win.webContents.setZoomFactor(s ?? (deck ? 1.25 : 1));
    }
    case 'wifi-scan':
    case 'wifi-connect':
    case 'wifi-current': {
      const reqId = payload.reqId || '';
      try {
        const data = event === 'wifi-scan' ? await wifi.scan()
          : event === 'wifi-current' ? await wifi.current()
          : await wifi.connect(payload.ssid, payload.password ?? null);
        send(`${event}-result`, { reqId, ok: true, data });
      } catch (err) {
        send(`${event}-result`, { reqId, ok: false, error: String((err && err.message) || err) });
      }
      return undefined;
    }
    default:
      return undefined;
  }
});

if (primary) app.whenReady().then(async () => {
  session.defaultSession.setPermissionRequestHandler((wc, perm, cb) => {
    cb(perm === 'media' && wc.getURL().startsWith(`http://localhost:${port}`));
  });
  session.defaultSession.on('will-download', (_e, item) => {
    const dir = app.getPath('downloads');
    const name = item.getFilename();
    if (!deck) return item.setSaveDialogOptions({ title: '파일 저장', defaultPath: path.join(dir, name) });
    const ext = path.extname(name), base = name.slice(0, name.length - ext.length);
    let dest = path.join(dir, name);
    for (let n = 1; fs.existsSync(dest); n++) dest = path.join(dir, `${base} (${n})${ext}`);
    item.setSavePath(dest);
  });
  port = (await pickFreePort(PORT_MIN)) ?? PORT_MIN;
  spawnProxy({ ip: '127.0.0.1', visionIp: '127.0.0.1', rendezvousUrl: '', robotId: '', webrtcToken: '' });
  createWindow();
});

app.on('window-all-closed', () => app.quit());
app.on('will-quit', () => { generation++; if (child) child.kill(); });
