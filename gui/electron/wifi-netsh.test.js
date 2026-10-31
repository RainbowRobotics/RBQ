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

test('망 목록 — 한국어(오픈·WPA2-개인)', () => {
  const out = ' \r\n인터페이스 이름: Wi-Fi \r\n현재 2개 네트워크를 볼 수 있습니다. \r\n\r\n'
    + 'SSID 1 : RBQ_EXAMPLE\r\n    네트워크 종류            : 인프라\r\n    인증          : WPA2-개인\r\n    암호화              : CCMP \r\n'
    + '    BSSID 1                 : 00:11:22:33:44:55\r\n         신호             : 72%  \r\n         무선 송수신 장치 유형         : 802.11ac\r\n         밴드               : 5 GHz\r\n         채널            : 36 \r\n\r\n'
    + 'SSID 2 : guest\r\n    네트워크 종류            : 인프라\r\n    인증          : 오픈\r\n    암호화              : 없음 \r\n'
    + '    BSSID 1                 : 66:77:88:99:aa:bb\r\n         신호             : 40%  \r\n';
  assert.deepStrictEqual(parseNetworks(out), [
    { ssid: 'RBQ_EXAMPLE', signal: 72, secured: true, saved: false, active: false },
    { ssid: 'guest', signal: 40, secured: false, saved: false, active: false },
  ]);
});

test('현재망 — 한국어 연결됨/연결하는 중/연결되지 않음', () => {
  const iface = (state) => '\r\n    이름                   : Wi-Fi\r\n    설명                   : Intel(R) Wi-Fi 6E AX211 160MHz\r\n'
    + '    GUID                   : 0f0e0d0c-0b0a-0908-0706-050403020100\r\n    물리적 주소            : 00:11:22:33:44:55\r\n'
    + `    인터페이스 유형         : 기본\r\n    상태                   : ${state}\r\n`
    + '    SSID                   : RBQ_EXAMPLE\r\n    AP BSSID               : 66:77:88:99:aa:bb\r\n'
    + '    네트워크 종류          : 인프라\r\n    송수신 장치 종류       : 802.11ac\r\n    인증                   : WPA2-개인\r\n'
    + '    수신 속도(Mbps)        : 866.7\r\n    전송 속도(Mbps)        : 866.7\r\n    신호                   : 88% \r\n';
  assert.deepStrictEqual(parseInterface(iface('연결됨')), { ssid: 'RBQ_EXAMPLE', signal: 88, secured: true, saved: false, active: true });
  assert.strictEqual(parseInterface(iface('연결하는 중')), null);
  assert.strictEqual(parseInterface(iface('연결되지 않음')), null);
});
