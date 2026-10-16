const test = require('node:test');
const assert = require('node:assert');
const { parseNetworks, parseInterface, profileXml } = require('./wifi-netsh');

test('망 목록 — 보안·개방·신호', () => {
  const out = 'Interface name : Wi-Fi\r\nThere are 2 networks currently visible.\r\n\r\nSSID 1 : RBQ_EXAMPLE\r\n    Network type            : Infrastructure\r\n    Authentication          : WPA2-Personal\r\n    Encryption              : CCMP\r\n    BSSID 1                 : 00:11:22:33:44:55\r\n         Signal             : 72%\r\n\r\nSSID 2 : guest\r\n    Authentication          : Open\r\n    BSSID 1                 : 66:77:88:99:aa:bb\r\n         Signal             : 40%\r\n';
  assert.deepStrictEqual(parseNetworks(out), [
    { ssid: 'RBQ_EXAMPLE', signal: 72, secured: true, saved: false, active: false },
    { ssid: 'guest', signal: 40, secured: false, saved: false, active: false },
  ]);
});

test('현재망 — 접속 중/끊김', () => {
  const on = '    Name                   : Wi-Fi\r\n    State                  : connected\r\n    SSID                   : RBQ_EXAMPLE\r\n    BSSID                  : 00:11:22:33:44:55\r\n    Signal                 : 88%\r\n';
  assert.deepStrictEqual(parseInterface(on), { ssid: 'RBQ_EXAMPLE', signal: 88, secured: true, saved: false, active: true });
  assert.strictEqual(parseInterface('    Name : Wi-Fi\r\n    State : disconnected\r\n'), null);
});

test('프로필 XML — 이스케이프·개방망엔 키 없음', () => {
  const x = profileXml('A&B', 'p<w>');
  assert.ok(x.includes('<name>A&amp;B</name>') && x.includes('<keyMaterial>p&lt;w&gt;</keyMaterial>') && x.includes('WPA2PSK'));
  const o = profileXml('guest', null);
  assert.ok(o.includes('<authentication>open</authentication>') && !o.includes('keyMaterial'));
});
