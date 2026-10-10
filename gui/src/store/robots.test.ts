import { describe, it, expect, beforeEach, vi } from 'vitest';

vi.mock('@/lib/connectNow', () => ({ connectNow: vi.fn(), currentTarget: vi.fn(() => ({ route: 'direct', addr: '192.168.0.10' })) }));
vi.mock('@/lib/rest', () => ({ rest: { serialNumber: vi.fn() } }));
vi.mock('@/lib/robotScan', () => ({ probeLan: vi.fn(async () => null), ROBOT_LAN_IP: '192.168.0.10' }));
const settingsState = { robotVersion: 'none', setRobotVersion: (v: string) => { settingsState.robotVersion = v; } };
vi.mock('@/store/settings', () => ({ useSettings: { getState: () => settingsState } }));
vi.mock('@/lib/robotWifi', () => ({ switchToRobotWifi: vi.fn(async () => 'unsupported') }));
vi.mock('@/lib/desktopBridge', () => ({
  isDesktop: () => true, wifiCurrent: vi.fn(async () => ({ ssid: 'Rainbow_Office', ip: '10.0.0.5' })),
  wifiScan: vi.fn(async () => []), wifiConnect: vi.fn(async () => {}),
}));
vi.mock('@/lib/currentSsid', () => ({ currentSsid: vi.fn(async () => null) }));
vi.mock('@/lib/demoFlag', () => ({ isDemo: () => false, getDemoBaseRobot: () => null, getDemoBaseLevel: () => null }));
vi.mock('@react-native-async-storage/async-storage', () => ({
  default: { getItem: vi.fn(async () => null), setItem: vi.fn(async () => {}), removeItem: vi.fn(async () => {}) },
}));

import { connectNow } from '@/lib/connectNow';
import { rest } from '@/lib/rest';
import { probeLan } from '@/lib/robotScan';
import { currentSsid } from '@/lib/currentSsid';
import { switchToRobotWifi } from '@/lib/robotWifi';
import { useRobot } from '@/store/robot';
import { useAccount } from '@/store/account';
import { useViewport } from '@/store/viewport';
import { robotAuth, ipKey, reportAuthFailure } from '@/lib/auth';
import { useRobots, passwordKeyFor, targetFor, fromRemote, migrateLegacy, mergeProfiles, SIM_SERIAL, type RobotProfile } from './robots';

const lanBot: RobotProfile = { serial: 'RBQ10-001', name: '3층 순찰기', lan: { ip: '192.168.0.10' }, lastVia: 'lan', lastSeenAt: 10 };
const rvBot: RobotProfile = { serial: 'RBQ10-009', name: 'RBQ10-009',
  rendezvous: { url: 'wss://rv.example/ws', robotId: 'RBQ10-009' }, lastVia: 'rendezvous', lastSeenAt: 5 };

beforeEach(() => {
  vi.clearAllMocks();
  useRobots.setState({ local: [], currentSerial: null, wifiAsk: null, wifiAskedNo: false });
  useAccount.setState({ account: null, robots: [], pin: '' });
  useRobot.setState({ ip: '192.168.0.10', conn: 'connected' });
});

describe('targetFor', () => {
  it('lastVia=lan 이면 직결 타깃, 토큰 없음', () => {
    const t = targetFor(lanBot)!;
    expect(t.route).toBe('direct'); expect(t.addr).toBe('192.168.0.10'); expect(t.label).toBe('3층 순찰기');
  });
  it('lastVia=rendezvous 면 랑데부 타깃 — addr 는 robotId', () => {
    const t = targetFor(rvBot)!;
    expect(t.route).toBe('rendezvous'); expect(t.addr).toBe('RBQ10-009'); expect(t.rendezvousUrl).toBe('wss://rv.example/ws');
  });
  it('시뮬은 연결 대상이 아니다', () => {
    expect(targetFor({ serial: SIM_SERIAL, name: '시뮬레이터', lastVia: 'lan', lastSeenAt: 0 })).toBeNull();
  });
});

describe('fromRemote / migrateLegacy', () => {
  it('서버 계정 행을 프로필로 — 랑데부 값이 있으면 rendezvous 우선', () => {
    const p = fromRemote({ accountId: 'a', accountName: 'n', level: 1, robotSerial: 'S1', robotName: '순찰',
      lanIp: '192.168.0.10', wanIp: null, rendezvousUrl: 'wss://rv', robotId: 'S1', webrtcToken: null, expiresAt: null })!;
    expect(p.serial).toBe('S1'); expect(p.name).toBe('순찰'); expect(p.lastVia).toBe('rendezvous'); expect(p.lan?.ip).toBe('192.168.0.10');
  });
  it('robotSerial 없는 행은 null', () => {
    expect(fromRemote({ accountId: 'a', accountName: 'n', level: 1, robotSerial: null, robotName: null,
      lanIp: null, wanIp: null, rendezvousUrl: null, robotId: null, webrtcToken: null, expiresAt: null })).toBeNull();
  });
  it('옛 설정의 LAN 은 옮기지 않는다(로봇망은 고정 IP 프로브가 찾는다) — 공인 IP 만 pending 으로', () => {
    expect(migrateLegacy('192.168.0.77', '', '192.168.0.10', '')).toBeNull();
    const p = migrateLegacy('', '1.2.3.4', '192.168.0.10', 'tok')!;
    expect(p.serial).toBe('pending:1.2.3.4'); expect(p.wan?.ip).toBe('1.2.3.4'); expect(p.lastVia).toBe('wan');
  });
});

describe('useRobots', () => {
  it('select 는 currentSerial 을 바꾸고 connectNow 를 부른다', async () => {
    useRobots.setState({ local: [lanBot] });
    await useRobots.getState().select('RBQ10-001');
    expect(useRobots.getState().currentSerial).toBe('RBQ10-001');
    expect(connectNow).toHaveBeenCalledWith(expect.objectContaining({ addr: '192.168.0.10' }));
  });
  it('select(sim) 은 뷰포트만 sim 으로, 연결은 안 한다 · 실로봇을 고르면 pose3d 로 되돌린다', async () => {
    useRobots.setState({ local: [lanBot] });
    await useRobots.getState().select(SIM_SERIAL);
    expect(useViewport.getState().key).toBe('sim'); expect(connectNow).not.toHaveBeenCalled();
    await useRobots.getState().select('RBQ10-001');
    expect(useViewport.getState().key).toBe('pose3d'); expect(connectNow).toHaveBeenCalled();
  });
  it('leaveSim 은 시뮬 전 로봇으로 되돌리고 뷰포트를 pose3d 로 — 연결은 다시 하지 않는다', async () => {
    useRobots.setState({ local: [lanBot], currentSerial: 'RBQ10-001', lastRealSerial: null });
    await useRobots.getState().select(SIM_SERIAL);
    vi.mocked(connectNow).mockClear();
    useRobots.getState().leaveSim();
    expect(useRobots.getState().currentSerial).toBe('RBQ10-001');
    expect(useViewport.getState().key).toBe('pose3d'); expect(connectNow).not.toHaveBeenCalled();
    useRobots.getState().leaveSim();
    expect(useRobots.getState().currentSerial).toBe('RBQ10-001');
  });
  it('로컬 LAN 캐시와 서버 원격이 같은 시리얼이면 한 줄로 합쳐진다(이름·LAN 은 로컬, 원격은 서버)', () => {
    const remote = { accountId: 'a', accountName: 'n', level: 1 as const, robotSerial: 'RBQ10-001', robotName: '서버이름',
      lanIp: '10.0.0.9', wanIp: null, rendezvousUrl: 'wss://rv', robotId: 'RBQ10-001', webrtcToken: null, expiresAt: null };
    const m = mergeProfiles([lanBot], [remote]);
    expect(m).toHaveLength(1); expect(m[0].name).toBe('3층 순찰기'); expect(m[0].lan?.ip).toBe('192.168.0.10'); expect(m[0].rendezvous?.url).toBe('wss://rv');
  });
  it('LAN 과 원격이 둘 다 있으면 LAN 이 닿을 때만 LAN 으로, 아니면 원격으로 붙는다', async () => {
    useRobots.setState({ local: [lanBot] });
    useAccount.setState({ account: { accountId: 'a', accountName: 'n', level: 1, robotSerial: 'RBQ10-001', robotName: 'x', lanIp: null, wanIp: null, rendezvousUrl: 'wss://rv', robotId: 'RBQ10-001', webrtcToken: null, expiresAt: null },
      robots: [{ accountId: 'a', accountName: 'n', level: 1, robotSerial: 'RBQ10-001', robotName: 'x', lanIp: null, wanIp: null, rendezvousUrl: 'wss://rv', robotId: 'RBQ10-001', webrtcToken: null, expiresAt: null }] });
    useAccount.setState({ pin: '1234' });
    (probeLan as any).mockResolvedValueOnce('RBQ10-001');
    await useRobots.getState().select('RBQ10-001');
    expect(connectNow).toHaveBeenLastCalledWith(expect.objectContaining({ route: 'direct', addr: '192.168.0.10' }));
    expect(useRobots.getState().local[0].lastVia).toBe('lan');
    (probeLan as any).mockResolvedValueOnce(null);
    await useRobots.getState().select('RBQ10-001');
    expect(connectNow).toHaveBeenLastCalledWith(expect.objectContaining({ route: 'rendezvous' }));
    expect(useRobots.getState().local[0].lastVia).toBe('rendezvous');
  });
  it('adoptLanRobot — 망이 바뀌어 로봇망에 로봇(시리얼 API 응답)이 있으면 그 로봇으로, 없으면 그대로', async () => {
    const { currentSsid } = await import('@/lib/currentSsid');
    useRobots.setState({ local: [lanBot], currentSerial: 'RBQ10-001' });
    useRobot.setState({ via: 'rendezvous', conn: 'connected' });
    vi.mocked(connectNow).mockClear();
    (probeLan as any).mockResolvedValueOnce(null);
    await useRobots.getState().adoptLanRobot();
    expect(connectNow).not.toHaveBeenCalled();
    (currentSsid as any).mockResolvedValueOnce('RBQ_EXAMPLE_B');
    (probeLan as any).mockResolvedValueOnce('RBQ10-005').mockResolvedValueOnce(null);
    await useRobots.getState().adoptLanRobot();
    expect(useRobots.getState().currentSerial).toBe('RBQ10-005');
    expect(useRobots.getState().local.find((p) => p.serial === 'RBQ10-005')?.name).toBe('RBQ_EXAMPLE_B');
    expect(connectNow).toHaveBeenLastCalledWith(expect.objectContaining({ route: 'direct', addr: '192.168.0.10' }));
  });
  it('시리얼이 빈 로봇(MuJoCo 시뮬)도 주소를 열쇠로 저장된다 — 헤더가 로봇 없음이 되지 않게', async () => {
    useRobots.setState({ local: [{ serial: 'pending:127.0.0.1', name: '내 로봇', lan: { ip: '127.0.0.1' }, lastVia: 'lan', lastSeenAt: 1 }], currentSerial: 'pending:127.0.0.1' });
    useRobot.setState({ ip: '127.0.0.1', conn: 'connected' });
    (rest.serialNumber as any).mockResolvedValueOnce({ serial_number: '' });
    expect(await useRobots.getState().saveConnected()).toBe(true);
    expect(useRobots.getState().currentSerial).toBe('addr:127.0.0.1');
    expect(useRobots.getState().local.map((p) => p.serial)).toEqual(['addr:127.0.0.1']);
    expect(useRobots.getState().local[0].name).toBe('PC 시뮬레이터');
  });
  it('저장된 옛 기본 이름(이 PC 시뮬)은 읽을 때 PC 시뮬레이터로 — 사용자가 지은 이름은 그대로', () => {
    const merge = useRobots.persist.getOptions().merge!;
    const sim = (name: string): RobotProfile => ({ serial: 'addr:127.0.0.1', name, lan: { ip: '127.0.0.1' }, lastVia: 'lan', lastSeenAt: 1 });
    const out = merge({ local: [sim('이 PC 시뮬'), { ...lanBot, name: '이 PC 시뮬 2호' }], currentSerial: 'addr:127.0.0.1' },
      useRobots.getState()) as ReturnType<typeof useRobots.getState>;
    expect(out.local.map((p) => p.name)).toEqual(['PC 시뮬레이터', '이 PC 시뮬 2호']);
    expect(out.currentSerial).toBe('addr:127.0.0.1');
  });
  it('addAddress — 확인된 주소는 시리얼 한 줄, 같은 로봇을 다시 추가해도 늘지 않는다', async () => {
    useRobots.setState({ local: [{ serial: 'pending:10.0.0.5', name: '내 로봇', lan: { ip: '10.0.0.5' }, lastVia: 'lan', lastSeenAt: 1 },
      { serial: 'addr:10.0.0.5', name: '10.0.0.5', lan: { ip: '10.0.0.5' }, lastVia: 'lan', lastSeenAt: 1 }], currentSerial: null });
    await useRobots.getState().addAddress('10.0.0.5', 'RBQ10-007');
    await useRobots.getState().addAddress('10.0.0.5', 'RBQ10-007');
    expect(useRobots.getState().local.map((p) => p.serial)).toEqual(['RBQ10-007']);
    await useRobots.getState().addAddress('127.0.0.1', '');
    await useRobots.getState().addAddress('127.0.0.1', '');
    expect(useRobots.getState().local.map((p) => p.serial)).toEqual(['addr:127.0.0.1', 'RBQ10-007']);
    expect(useRobots.getState().local[0].name).toBe('PC 시뮬레이터');
  });
  it('로봇 비밀번호는 그 로봇에만 — 다른 주소로 새지 않고, 추가 중 받은 주소 비밀번호는 시리얼로 옮겨진다', async () => {
    const basic = (pw: string) => ({ Authorization: `Basic ${btoa(`rbq:${pw}`)}` });
    useRobots.setState({ local: [lanBot], currentSerial: lanBot.serial, passwords: {} });
    useRobots.getState().setPassword(passwordKeyFor()!, 'a');
    expect(robotAuth()).toEqual(basic('a'));
    expect(robotAuth('192.168.0.10')).toEqual(basic('a'));
    expect(robotAuth('10.0.0.5')).toEqual({});
    useRobots.getState().setPassword(passwordKeyFor('10.0.0.5')!, 'b');
    expect(useRobots.getState().passwords).toEqual({ [lanBot.serial]: 'a', [ipKey('10.0.0.5')]: 'b' });
    await useRobots.getState().addAddress('10.0.0.5', 'RBQ10-007');
    expect(useRobots.getState().passwords).toEqual({ [lanBot.serial]: 'a', 'RBQ10-007': 'b' });
  });
  it('WAN 직결 로봇 — 연결 401 뒤 입력값이 지금 로봇에 저장되고 offer(대상 없음)에 실린다', () => {
    const basic = (pw: string) => ({ Authorization: `Basic ${btoa(`rbq:${pw}`)}` });
    const wanBot: RobotProfile = { serial: 'pending:203.0.113.7', name: 'W', wan: { ip: '203.0.113.7' }, lastVia: 'wan', lastSeenAt: 1 };
    useRobots.setState({ local: [wanBot], currentSerial: wanBot.serial, passwords: {} });
    useRobot.setState({ ip: '203.0.113.7' });
    const key = passwordKeyFor();
    expect(key).toBe(wanBot.serial);
    useRobots.getState().setPassword(key!, 'w');
    expect(robotAuth()).toEqual(basic('w'));
    expect(robotAuth('203.0.113.7')).toEqual(basic('w'));
  });
  it('로봇망 .10 프로브 401 — 지금 로봇 항목을 덮어쓰지 않고 그 주소에 저장한다', () => {
    const basic = (pw: string) => ({ Authorization: `Basic ${btoa(`rbq:${pw}`)}` });
    useRobots.setState({ local: [lanBot], currentSerial: lanBot.serial, passwords: { [lanBot.serial]: 'a' } });
    const key = passwordKeyFor('192.168.0.10', true);
    expect(key).toBe(ipKey('192.168.0.10'));
    useRobots.getState().setPassword(key!, 'b');
    expect(useRobots.getState().passwords[lanBot.serial]).toBe('a');
    expect(robotAuth('192.168.0.10')).toEqual(basic('b'));
  });
  it('비밀번호를 넣기 전에 떠난 요청의 401 은 입력 창을 다시 띄우지 않는다', () => {
    useRobots.setState({ local: [lanBot], currentSerial: lanBot.serial, passwords: {} });
    const sentBefore = robotAuth();
    useRobots.getState().setPassword(lanBot.serial, 'a');
    useRobot.setState({ authNeeded: false });
    expect(reportAuthFailure(undefined, false, sentBefore)).toBe(false);
    expect(useRobot.getState().authNeeded).toBe(false);
    expect(reportAuthFailure(undefined, false, robotAuth())).toBe(true);
    expect(useRobot.getState().authNeeded).toBe(true);
  });
  it('옛 공용 폴백(*)은 저장소에서 지운다', () => {
    const merge = (useRobots as any).persist.getOptions().merge;
    const m = merge({ local: [], passwords: { '*': 'old', [lanBot.serial]: 'a' } }, useRobots.getState());
    expect(m.passwords).toEqual({ [lanBot.serial]: 'a' });
  });
  it('로봇 버전은 로봇마다 — 고른 로봇 값이 앱에 실리고, 다른 로봇으로 새지 않는다', async () => {
    useRobots.setState({ local: [{ ...lanBot, robotVersion: 'ver-a' }, { ...lanBot, serial: 'RBQ10-002', name: 'B' }] });
    await useRobots.getState().select('RBQ10-001');
    expect(settingsState.robotVersion).toBe('ver-a');
    await useRobots.getState().select('RBQ10-002');
    expect(settingsState.robotVersion).toBe('none');
    useRobots.getState().setRobotVersion('RBQ10-002', 'ver-b');
    expect(settingsState.robotVersion).toBe('ver-b');
    expect(useRobots.getState().local.find((p) => p.serial === 'RBQ10-001')?.robotVersion).toBe('ver-a');
  });
  it('로봇망(.10) 로봇들은 주소가 같아도 서로 지우지 않는다 — 시리얼 없는 로봇은 SSID 로 가른다', async () => {
    const { currentSsid } = await import('@/lib/currentSsid');
    useRobots.setState({ local: [], currentSerial: null });
    (currentSsid as any).mockResolvedValueOnce('RBQ_EXAMPLE_5G');
    await useRobots.getState().addAddress('192.168.0.10', '');
    (currentSsid as any).mockResolvedValueOnce('RBQ_EXAMPLE');
    await useRobots.getState().addAddress('192.168.0.10', 'RBQ1000000001');
    (currentSsid as any).mockResolvedValueOnce('RBQ_EXAMPLE_B');
    await useRobots.getState().addAddress('192.168.0.10', '');
    expect(useRobots.getState().local.map((p) => p.serial).sort()).toEqual(['RBQ1000000001', 'ssid:RBQ_EXAMPLE_5G', 'ssid:RBQ_EXAMPLE_B']);
    useRobots.getState().migrate('', '', '', '');
    expect(useRobots.getState().local).toHaveLength(3);
  });
  it('시리얼 미설정 로봇 AP 에 붙으면 SSID 열쇠로 그 로봇이 된다 — 다른 로봇과 섞이지 않는다', async () => {
    const { currentSsid } = await import('@/lib/currentSsid');
    useRobots.setState({ local: [lanBot], currentSerial: 'RBQ10-001' });
    useRobot.setState({ via: 'direct', conn: 'disconnected' });
    (currentSsid as any).mockReset(); (probeLan as any).mockReset();
    (currentSsid as any).mockResolvedValue('RBQ_EXAMPLE_5G');
    (probeLan as any).mockResolvedValue('');
    await useRobots.getState().adoptLanRobot();
    expect(useRobots.getState().currentSerial).toBe('ssid:RBQ_EXAMPLE_5G');
    expect(useRobots.getState().local.map((p) => p.serial).sort()).toEqual(['RBQ10-001', 'ssid:RBQ_EXAMPLE_5G']);
    (currentSsid as any).mockResolvedValue(null); (probeLan as any).mockResolvedValue(null);
  });
  it('원격 전용 로봇을 골라도 기기 캐시에는 안 남는다 — 로그아웃하면 사라져야 한다', async () => {
    const r = { accountId: 'a', accountName: 'n', level: 1 as const, robotSerial: 'RBQ10-009', robotName: '원격만', lanIp: null, wanIp: null,
      rendezvousUrl: 'wss://rv', robotId: 'RBQ10-009', webrtcToken: null, expiresAt: null };
    useAccount.setState({ account: r, robots: [r], pin: '1234' });
    await useRobots.getState().select('RBQ10-009');
    expect(connectNow).toHaveBeenLastCalledWith(expect.objectContaining({ route: 'rendezvous' }));
    expect(useRobots.getState().local).toEqual([]);
    expect(useRobots.getState().currentSerial).toBe('RBQ10-009');
  });
  it('로봇 WiFi 가 안 보이면(꺼짐) 연결하지 않고 선택을 되돌린 뒤 selectIssue 로 알린다', async () => {
    const off: RobotProfile = { serial: 'RBQ10-OFF', name: 'ROBOT-B', lan: { ip: '192.168.0.10', ssid: 'RBQ_EXAMPLE_5G' }, lastVia: 'lan', lastSeenAt: 1 };
    useRobots.setState({ switching: true, local: [off, { ...off, serial: 'RBQ10-ON', name: 'ROBOT-A', lan: { ip: '192.168.0.10', ssid: 'RBQ_EXAMPLE' } }], currentSerial: 'RBQ10-ON' });
    vi.mocked(switchToRobotWifi).mockResolvedValueOnce('not_visible');
    await useRobots.getState().select('RBQ10-OFF');
    expect(connectNow).not.toHaveBeenCalled();
    expect(useRobots.getState().currentSerial).toBe('RBQ10-ON');
    expect(useRobots.getState().selectIssue).toEqual({ name: 'ROBOT-B', ssid: 'RBQ_EXAMPLE_5G', kind: 'off' });
    expect(useRobots.getState().switching).toBe(false);
  });
  it('로봇을 바꾸는 동안 switching 이 켜지고 끝나면 내려간다(그 사이 로봇 버튼 막음)', async () => {
    useRobots.setState({ local: [{ serial: 'RBQ10-A', name: 'A', lan: { ip: '192.168.0.10', ssid: 'RBQ_EXAMPLE_A' }, lastVia: 'lan', lastSeenAt: 1 }], currentSerial: null, switching: false });
    let during = false;
    vi.mocked(switchToRobotWifi).mockImplementationOnce(async () => { during = useRobots.getState().switching; return 'switched'; });
    await useRobots.getState().select('RBQ10-A');
    expect(during).toBe(true);
    expect(useRobots.getState().switching).toBe(false);
  });
  it('시리얼 미설정 로봇(ssid: 열쇠)의 lan.ssid 는 지금 붙은 다른 AP 로 덮이지 않는다', async () => {
    useRobots.setState({ local: [{ serial: 'ssid:RBQ_EXAMPLE_B_5G', name: 'NEW', lan: { ip: '192.168.0.10', ssid: 'RBQ_EXAMPLE_B_5G' }, lastVia: 'lan', lastSeenAt: 1 }], currentSerial: null });
    vi.mocked(currentSsid).mockResolvedValue('RBQ_EXAMPLE_5G');
    await useRobots.getState().select('ssid:RBQ_EXAMPLE_B_5G');
    expect(useRobots.getState().local[0].lan?.ssid).toBe('RBQ_EXAMPLE_B_5G');
    vi.mocked(currentSsid).mockResolvedValue(null);
  });
  describe("자동 선택(wifi='ask')은 지금 망을 끊기 전에 묻는다", () => {
    const bot: RobotProfile = { serial: 'RBQ10-ASK', name: 'ROBOT-C', lan: { ip: '192.168.0.10', ssid: 'RBQ_EXAMPLE_5G' }, lastVia: 'lan', lastSeenAt: 1 };
    beforeEach(() => { useRobots.setState({ local: [bot], currentSerial: null }); });

    it('다른 망(사무실)에 붙어 있으면 옮기지도 붙지도 않고 wifiAsk 만 켠다', async () => {
      await useRobots.getState().select('RBQ10-ASK', 'ask');
      expect(switchToRobotWifi).not.toHaveBeenCalled();
      expect(connectNow).not.toHaveBeenCalled();
      expect(useRobots.getState().wifiAsk).toEqual({ serial: 'RBQ10-ASK', name: 'ROBOT-C', ssid: 'RBQ_EXAMPLE_5G' });
      expect(useRobots.getState().switching).toBe(false);
    });
    it("'예' 면 로봇 WiFi 로 옮기고 붙는다", async () => {
      await useRobots.getState().select('RBQ10-ASK', 'ask');
      vi.mocked(switchToRobotWifi).mockResolvedValueOnce('switched');
      useRobots.getState().answerWifiAsk(true);
      await vi.waitFor(() => expect(connectNow).toHaveBeenCalled());
      expect(switchToRobotWifi).toHaveBeenCalledWith('RBQ_EXAMPLE_5G', expect.any(Function), undefined, expect.anything());
      expect(useRobots.getState().wifiAsk).toBeNull();
    });
    it("'아니오' 면 망을 그대로 두고 붙어 본다 — 그리고 이 실행에선 다시 묻지 않는다", async () => {
      await useRobots.getState().select('RBQ10-ASK', 'ask');
      useRobots.getState().answerWifiAsk(false);
      await vi.waitFor(() => expect(connectNow).toHaveBeenCalled());
      expect(switchToRobotWifi).not.toHaveBeenCalled();
      expect(useRobots.getState().wifiAsk).toBeNull();
      expect(useRobots.getState().currentSerial).toBe('RBQ10-ASK');
      await useRobots.getState().select('RBQ10-ASK', 'ask');
      expect(useRobots.getState().wifiAsk).toBeNull();
      expect(switchToRobotWifi).not.toHaveBeenCalled();
    });
    it('기다리는 사이 사용자가 다른 로봇을 골랐으면 낡은 물음을 띄우지 않는다 — 답이 그 선택을 덮는다', async () => {
      vi.mocked(probeLan).mockImplementationOnce(async () => { useRobots.setState({ currentSerial: 'RBQ10-OTHER' }); return null; });
      await useRobots.getState().select('RBQ10-ASK', 'ask');
      expect(useRobots.getState().wifiAsk).toBeNull();
      expect(useRobots.getState().currentSerial).toBe('RBQ10-OTHER');
      expect(switchToRobotWifi).not.toHaveBeenCalled();
    });
    it("사람이 직접 고르면(wifi 기본='auto') 종전처럼 묻지 않고 옮긴다", async () => {
      vi.mocked(switchToRobotWifi).mockResolvedValueOnce('switched');
      await useRobots.getState().select('RBQ10-ASK');
      expect(useRobots.getState().wifiAsk).toBeNull();
      expect(switchToRobotWifi).toHaveBeenCalled();
    });
  });
  it('원격인데 코드가 없으면(재시작 뒤) 두드리지 않고 codeNeeded 만 켠다', async () => {
    const r = { accountId: 'a', accountName: 'n', level: 1 as const, robotSerial: 'RBQ10-009', robotName: '원격만', lanIp: null, wanIp: null,
      rendezvousUrl: 'wss://rv', robotId: 'RBQ10-009', webrtcToken: null, expiresAt: null };
    useAccount.setState({ account: r, robots: [r], pin: '' });
    await useRobots.getState().select('RBQ10-009');
    expect(connectNow).not.toHaveBeenCalled(); expect(useRobots.getState().codeNeeded).toBe(true);
    useAccount.setState({ pin: '1234' });
    await useRobots.getState().select('RBQ10-009');
    expect(connectNow).toHaveBeenCalled(); expect(useRobots.getState().codeNeeded).toBe(false);
  });
  it('saveConnected 는 시리얼을 읽어 프로필을 만들고 pending 을 교체한다', async () => {
    useRobots.setState({ local: [{ ...lanBot, serial: 'pending:192.168.0.10', name: '내 로봇' }], currentSerial: 'pending:192.168.0.10' });
    (rest.serialNumber as any).mockResolvedValue({ serial_number: 'RBQ10-001' });
    expect(await useRobots.getState().saveConnected()).toBe(true);
    const l = useRobots.getState().local;
    expect(l).toHaveLength(1); expect(l[0].serial).toBe('RBQ10-001'); expect(l[0].name).toBe('내 로봇');
    expect(useRobots.getState().currentSerial).toBe('RBQ10-001');
  });
  it('saveConnected 는 미연결이면 false', async () => {
    useRobot.setState({ conn: 'disconnected' });
    expect(await useRobots.getState().saveConnected()).toBe(false);
  });
  it('addLan 은 같은 주소의 pending 프로필을 치우고 선택·연결한다', async () => {
    useRobots.setState({ local: [{ ...lanBot, serial: 'pending:192.168.0.10', name: '내 로봇' }], currentSerial: 'pending:192.168.0.10' });
    await useRobots.getState().addLan('RBQ10-001', '192.168.0.10');
    const l = useRobots.getState().local;
    expect(l.map((p) => p.serial)).toEqual(['RBQ10-001']); expect(useRobots.getState().currentSerial).toBe('RBQ10-001');
    expect(connectNow).toHaveBeenCalledWith(expect.objectContaining({ addr: '192.168.0.10' }));
  });
  it('로봇망으로 붙으면 시리얼을 읽어 자동 캐시한다 — 같은 로봇은 한 줄', async () => {
    useRobots.setState({ local: [{ ...lanBot, serial: 'pending:192.168.0.10', name: '내 로봇' }], currentSerial: 'pending:192.168.0.10' });
    useRobot.setState({ conn: 'disconnected' });
    (rest.serialNumber as any).mockResolvedValue({ serial_number: 'RBQ10-001' });
    useRobot.setState({ conn: 'connected' });
    await new Promise((r) => setTimeout(r, 0));
    expect(useRobots.getState().local.map((p) => p.serial)).toEqual(['RBQ10-001']);
    expect(useRobots.getState().local[0].name).toBe('내 로봇');
  });
  it('clearLocal 은 목록과 선택을 비운다', () => {
    useRobots.setState({ local: [lanBot], currentSerial: 'RBQ10-001' });
    useRobots.getState().clearLocal();
    expect(useRobots.getState().local).toEqual([]); expect(useRobots.getState().currentSerial).toBeNull();
  });
});
