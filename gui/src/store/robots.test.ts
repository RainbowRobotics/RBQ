import { describe, it, expect, beforeEach, vi } from 'vitest';

vi.mock('@/lib/connectNow', () => ({ connectNow: vi.fn(), disconnectAll: vi.fn(), currentTarget: vi.fn(() => ({ route: 'direct', addr: '192.168.0.10' })) }));
vi.mock('@/lib/rest', () => ({ rest: { serialNumber: vi.fn(), version: vi.fn(async () => ({})) } }));
vi.mock('@/lib/firmwareRelease', () => ({ LEGACY_SYSTEM: 'legacy' }));
vi.mock('@/lib/ota', () => ({ otaProbe: vi.fn(async () => null), visionOtaIp: (v?: string) => (v ?? '').trim() || '192.168.0.12' }));
vi.mock('@/lib/robotScan', () => ({ probeLan: vi.fn(async () => null), ROBOT_LAN_IP: '192.168.0.10' }));
const settingsState = {
  robotVersion: 'none', setRobotVersion: (v: string) => { settingsState.robotVersion = v; },
  otaSystems: {} as Record<string, string>,
  rememberOtaSystem: (ip: string, system: string) => { settingsState.otaSystems[ip] = system; },
};
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

import { connectNow, disconnectAll } from '@/lib/connectNow';
import { rest } from '@/lib/rest';
import { probeLan } from '@/lib/robotScan';
import { otaProbe } from '@/lib/ota';
import { currentSsid } from '@/lib/currentSsid';
import { switchToRobotWifi } from '@/lib/robotWifi';
import { useRobot } from '@/store/robot';
import { useViewport } from '@/store/viewport';
import { robotAuth, ipKey, reportAuthFailure } from '@/lib/auth';
import { useRobots, knownSystem, passwordKeyFor, targetFor, migrateLegacy, SIM_SERIAL, type RobotProfile } from './robots';

const lanBot: RobotProfile = { serial: 'RBQ10-001', name: '3층 순찰기', lan: { ip: '192.168.0.10' }, lastVia: 'lan', lastSeenAt: 10 };

beforeEach(() => {
  vi.clearAllMocks();
  useRobots.setState({ local: [], currentSerial: null, wifiAsk: null, wifiAskedNo: false });
  useRobot.setState({ ip: '192.168.0.10', conn: 'connected' });
});

describe('targetFor', () => {
  it('lastVia=lan 이면 직결 타깃, 토큰 없음', () => {
    const t = targetFor(lanBot)!;
    expect(t.route).toBe('direct'); expect(t.addr).toBe('192.168.0.10'); expect(t.label).toBe('3층 순찰기');
  });
  it('시뮬은 연결 대상이 아니다', () => {
    expect(targetFor({ serial: SIM_SERIAL, name: '시뮬레이터', lastVia: 'lan', lastSeenAt: 0 })).toBeNull();
  });
});

describe('migrateLegacy', () => {
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
  it('adoptLanRobot — 망이 바뀌어 로봇망에 로봇(시리얼 API 응답)이 있으면 그 로봇으로, 없으면 그대로', async () => {
    const { currentSsid } = await import('@/lib/currentSsid');
    useRobots.setState({ local: [lanBot], currentSerial: 'RBQ10-001' });
    useRobot.setState({ via: 'remote', conn: 'connected' });
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

describe('로봇 시스템 — 연결될 때 배워 프로필에 남긴다(받을 패키지를 고른다)', () => {
  const connectTo = async (serial: string) => {
    useRobot.setState({ conn: 'disconnected' });
    useRobots.setState({ currentSerial: serial });
    vi.mocked(rest.serialNumber).mockResolvedValue({ serial_number: serial } as any);
    useRobot.setState({ conn: 'connected' });
  };
  beforeEach(() => { settingsState.otaSystems = {}; });

  it('정비 화면을 열지 않아도 Motion(/api/version)·비전 PC(OTA 핸드셰이크) 시스템이 그 로봇 줄에 남는다', async () => {
    useRobots.setState({ local: [lanBot] });
    vi.mocked(rest.version).mockResolvedValue({ version: '1.2609.0', system: 'u24-x86_64' } as any);
    vi.mocked(otaProbe).mockImplementation(async (ip: string) => (ip === '192.168.0.12' ? { protocol: 2, system: 'u22-x86_64' } : null));
    await connectTo('RBQ10-001');
    await vi.waitFor(() => expect(useRobots.getState().local[0].visionSystem).toBe('u22-x86_64'));
    expect(useRobots.getState().local[0].system).toBe('u24-x86_64');
    useRobot.setState({ conn: 'disconnected' });
    expect(knownSystem('motion')).toBe('u24-x86_64');
    expect(knownSystem('vision')).toBe('u22-x86_64');
  });

  it('구형 Network(/api/version 에 system 없음)는 Motion 의 OTA 핸드셰이크로 묻는다', async () => {
    useRobots.setState({ local: [lanBot] });
    vi.mocked(rest.version).mockResolvedValue({ version: '1.2608.0' } as any);
    vi.mocked(otaProbe).mockImplementation(async (ip: string) => (ip === '192.168.0.10' ? { protocol: 2, system: 'u22-x86_64' } : null));
    await connectTo('RBQ10-001');
    await vi.waitFor(() => expect(useRobots.getState().local[0].system).toBe('u22-x86_64'));
    expect(useRobots.getState().local[0].visionSystem).toBeUndefined();
  });

  it('OTA 데몬이 답했는데 시스템이 없으면 구형 로봇(legacy)으로 남긴다 — 답이 없는 것(모름)과 다르다', async () => {
    useRobots.setState({ local: [lanBot] });
    vi.mocked(rest.version).mockResolvedValue({ version: '1.2607.0' } as any);
    vi.mocked(otaProbe).mockImplementation(async (ip: string) => (ip === '192.168.0.10' ? { protocol: 2 } : null));
    await connectTo('RBQ10-001');
    await vi.waitFor(() => expect(useRobots.getState().local[0].system).toBe('legacy'));
    expect(useRobots.getState().local[0].visionSystem).toBeUndefined();
  });

  it('같은 주소(.10)의 두 로봇은 각자 값을 가진다 — 주소가 아니라 시리얼', async () => {
    const other: RobotProfile = { ...lanBot, serial: 'RBQ10-002', name: '2호기' };
    useRobots.setState({ local: [lanBot, other] });
    vi.mocked(otaProbe).mockResolvedValue(null);
    vi.mocked(rest.version).mockResolvedValue({ version: '1', system: 'u24-x86_64' } as any);
    await connectTo('RBQ10-001');
    await vi.waitFor(() => expect(useRobots.getState().local.find((p) => p.serial === 'RBQ10-001')?.system).toBe('u24-x86_64'));
    vi.mocked(rest.version).mockResolvedValue({ version: '1', system: 'u22-x86_64' } as any);
    await connectTo('RBQ10-002');
    await vi.waitFor(() => expect(useRobots.getState().local.find((p) => p.serial === 'RBQ10-002')?.system).toBe('u22-x86_64'));
    useRobot.setState({ conn: 'disconnected' });
    useRobots.setState({ currentSerial: 'RBQ10-001' });
    expect(knownSystem('motion', '192.168.0.10')).toBe('u24-x86_64');
  });

  it('saveConnected 가 줄을 다시 써도 배운 값을 잃지 않는다', async () => {
    useRobots.setState({ local: [{ ...lanBot, system: 'u24-x86_64', visionSystem: 'u22-x86_64' }], currentSerial: 'RBQ10-001' });
    vi.mocked(rest.serialNumber).mockResolvedValue({ serial_number: 'RBQ10-001' } as any);
    expect(await useRobots.getState().saveConnected()).toBe(true);
    expect(useRobots.getState().local[0]).toMatchObject({ system: 'u24-x86_64', visionSystem: 'u22-x86_64' });
  });

  it('허브에서 고른 뒤에도(연결 실패해도) 배운 값이 남는다', async () => {
    useRobots.setState({ local: [{ ...lanBot, system: 'u24-x86_64', visionSystem: 'u22-x86_64' }] });
    useRobot.setState({ conn: 'disconnected' });
    await useRobots.getState().select('RBQ10-001');
    expect(useRobots.getState().local[0]).toMatchObject({ system: 'u24-x86_64', visionSystem: 'u22-x86_64' });
    expect(knownSystem('motion')).toBe('u24-x86_64');
  });

  it('프로필에 없으면 그 주소에서 마지막에 확인한 값으로 물러선다', () => {
    useRobots.setState({ local: [lanBot], currentSerial: 'RBQ10-001' });
    settingsState.otaSystems = { '192.168.0.10': 'u22-x86_64' };
    expect(knownSystem('motion', '192.168.0.10')).toBe('u22-x86_64');
    expect(knownSystem('motion')).toBeUndefined();
  });
});
