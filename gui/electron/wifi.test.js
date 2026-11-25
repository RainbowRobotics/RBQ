const test = require('node:test');
const assert = require('node:assert');
const net = require('node:net');
const os = require('node:os');
const path = require('node:path');
const fs = require('node:fs');
const { execFile } = require('node:child_process');

function probe(address) {
  const code = `process.on('uncaughtException', (e) => { console.log('UNCAUGHT ' + e.message); process.exit(3); });
    setTimeout(() => { console.log('HANG'); process.exit(4); }, 4000).unref();
    require('./wifi').current().then(() => console.log('OK'), (e) => console.log('REJECTED ' + e.message));`;
  return new Promise((resolve) => {
    execFile(process.execPath, ['-e', code], { cwd: __dirname, env: { ...process.env, DBUS_SYSTEM_BUS_ADDRESS: address } },
      (_err, stdout) => resolve(stdout.trim()));
  });
}

test('버스 소켓이 없다 — 예외 없이 거절', async () => {
  const out = await probe(`unix:path=${path.join(os.tmpdir(), 'rbq-no-such-bus')}`);
  assert.match(out, /^REJECTED /);
});

test('붙자마자 끊긴다(EPIPE) — 예외 없이 거절', async () => {
  const sock = path.join(os.tmpdir(), `rbq-dead-bus-${process.pid}`);
  fs.rmSync(sock, { force: true });
  const server = net.createServer((c) => c.destroy());
  await new Promise((r) => server.listen(sock, r));
  try {
    const out = await probe(`unix:path=${sock}`);
    assert.match(out, /^REJECTED /);
  } finally {
    server.close();
    fs.rmSync(sock, { force: true });
  }
});
