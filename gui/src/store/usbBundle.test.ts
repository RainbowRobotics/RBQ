import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
vi.mock('./robot', async () => ({ useRobot: (await import('zustand')).create(() => ({ ip: '192.168.0.10', conn: 'connected', droppedAt: 0 as number | undefined })) }));
vi.mock('@/lib/i18n', () => ({ t: (k: string) => k }));
vi.mock('@/lib/ota', () => ({ otaTransfer: vi.fn(), otaProbe: vi.fn() }));
vi.mock('@/lib/rest', () => ({ rest: {}, actions: {} }));
vi.mock('@/lib/connection', () => ({ connection: {} }));
vi.mock('@/lib/fileStream', () => ({ readFileAt: vi.fn() }));
vi.mock('@/lib/firmwareRelease', () => ({ parseSidecars: vi.fn() }));
import { useRobot } from './robot';
import { useSwJobs } from './swUpdate';
import { rest } from '@/lib/rest';
import { useUsbJob, usbLine, waitBack, startUsbBundle, robotNow } from './usbBundle';

const S = () => useUsbJob.getState();

describe('USB 묶음 설치 — 상단바 한 줄', () => {
  beforeEach(() => useUsbJob.setState({ open: 0, unseen: false, step: null, prog: null, err: null, done: null, label: 'v1.2611.0', option: 'demo' }));

  it('단계마다 무엇을 하는지 — 보내는 % · 로봇 설치 중 · 다시 켜는 중(닫기 없음)', () => {
    useUsbJob.setState({ step: 'core', prog: { phase: 'transfer', percentage: 42, acked: 1, total: 2 } });
    expect(usbLine(S())).toMatchObject({ tone: 'run', closable: false });
    expect(usbLine(S())!.text).toContain('v1.2611.0');
    expect(usbLine(S())!.text).toContain('42%');
    useUsbJob.setState({ step: 'option', prog: { phase: 'applying', percentage: 100, acked: 2, total: 2 } });
    expect(usbLine(S())!.text).toContain('DEMO');
    expect(usbLine(S())!.text).toContain('설치하는 중');
    useUsbJob.setState({ step: 'restart', prog: null });
    expect(usbLine(S())!.text).toContain('다시 켜');
    useUsbJob.setState({ step: 'legacy' });
    expect(usbLine(S())!.text).toContain('구형');
  });

  it('화면 밖에서 끝나면 결과를 한 번 — 완료·실패, 화면이 떠 있으면 없음', () => {
    useUsbJob.setState({ step: 'option' });
    useUsbJob.setState({ step: null, done: '설치 완료 — 본체 v1.2611.0 · DEMO v1.2611.0' });
    expect(usbLine(S())).toMatchObject({ tone: 'ok', closable: true });
    useUsbJob.setState({ err: '로봇이 다시 연결되지 않았습니다' });
    expect(usbLine(S())).toMatchObject({ tone: 'bad', closable: true });
    useUsbJob.setState({ open: 1 });
    expect(usbLine(S())).toBeNull();
  });
});

describe('USB 묶음 설치 — 다시 연결 기다리기', () => {
  beforeEach(() => { vi.useFakeTimers(); useRobot.setState({ ip: '192.168.0.10', conn: 'connected', droppedAt: 0 }); });
  afterEach(() => vi.useRealTimers());

  it('지금 붙어 있어도 since 뒤에 끊긴 적이 없으면 기다린다 — 끊겼다 다시 붙으면 끝', async () => {
    let out = 'pending';
    waitBack('192.168.0.10', 1000, 60_000).then(() => { out = 'ok'; });
    await vi.advanceTimersByTimeAsync(10);
    expect(out).toBe('pending');
    useRobot.setState({ conn: 'connecting', droppedAt: 2000 });
    await vi.advanceTimersByTimeAsync(10);
    expect(out).toBe('pending');
    useRobot.setState({ conn: 'connected' });
    await vi.advanceTimersByTimeAsync(10);
    expect(out).toBe('ok');
  });

  it('기한을 넘기면 실패 — 배너가 도는 중에 머물지 않는다', async () => {
    useRobot.setState({ conn: 'connecting', droppedAt: 2000 });
    const p = waitBack('192.168.0.10', 1000, 60_000);
    const check = expect(p).rejects.toThrow('다시 연결되지 않았습니다');
    await vi.advanceTimersByTimeAsync(60_001);
    await check;
  });

  it('로봇(주소)이 바뀌면 멈춘다', async () => {
    const p = waitBack('192.168.0.10', 1000, 60_000);
    const check = expect(p).rejects.toThrow('바뀌어');
    useRobot.setState({ ip: '192.168.0.11' });
    await check;
  });
});

describe('USB 묶음 설치 — 대상은 Motion 뿐', () => {
  it('화면 대상이 Vision 이면 시작하지 않고 사유를 보인다', () => {
    useSwJobs.setState({ tgt: 'vision' });
    const manifest = { label: 'v1.2611.0', channel: 'release', created: '', cores: [], options: [], legacy: null };
    useUsbJob.setState({ step: null, err: null, picked: { file: { uri: 'x', name: 'rbq-usb-v1.2611.0.tar', size: 1 }, bundle: { manifest, entries: [] } } });
    startUsbBundle('192.168.0.10');
    expect(S().step).toBeNull();
    expect(S().err).toContain('Motion');
    useSwJobs.setState({ tgt: 'motion' });
  });
});

describe('USB 묶음 설치 — 다시 붙은 직후의 조회는 몇 번 다시 묻는다', () => {
  const r = rest as unknown as Record<string, unknown>;
  beforeEach(() => vi.useFakeTimers());
  afterEach(() => vi.useRealTimers());
  const http = (status: number) => Object.assign(new Error(`HTTP ${status}`), { status });

  it('판 조회가 처음 몇 번 실패해도 이어서 묻는다 — features 404(옛 로봇)는 옵션 없음', async () => {
    let n = 0;
    r.version = vi.fn(async () => { if (n++ < 2) throw http(0); return { version: '1.2611.0', system: 'u22-x86_64' }; });
    r.robotFeatures = vi.fn(async () => { throw http(404); });
    const p = robotNow('192.168.0.10');
    await vi.advanceTimersByTimeAsync(10_000);
    await expect(p).resolves.toMatchObject({ version: '1.2611.0', system: 'u22-x86_64', option: '' });
    expect(n).toBe(3);
    expect(r.robotFeatures).toHaveBeenCalledTimes(1);
  });

  it('features 가 잠깐 실패하면(404 아님) 다시 묻는다 — 옵션이 없다고 오판하지 않는다', async () => {
    let n = 0;
    r.version = vi.fn(async () => ({ version: '1.2611.0', system: 'u22-x86_64' }));
    r.robotFeatures = vi.fn(async () => { if (n++ < 1) throw http(0); return { features: { option: 'demo', option_version: 'v1.2611.0' } }; });
    const p = robotNow('192.168.0.10');
    await vi.advanceTimersByTimeAsync(10_000);
    await expect(p).resolves.toMatchObject({ option: 'demo', optionVersion: 'v1.2611.0' });
  });

  it('계속 안 되면 기한 안에 실패한다', async () => {
    r.version = vi.fn(async () => { throw http(0); });
    const p = robotNow('192.168.0.10');
    const check = expect(p).rejects.toThrow('HTTP 0');
    await vi.advanceTimersByTimeAsync(30_000);
    await check;
    expect(r.version).toHaveBeenCalledTimes(5);
  });
});
