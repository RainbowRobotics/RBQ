import { describe, it, expect, beforeEach, vi } from 'vitest';
vi.mock('./robot', async () => ({ useRobot: (await import('zustand')).create(() => ({ ip: '192.168.0.10' })) }));
import { useRobot } from './robot';
import { useSwJobs, swBanner, pkgVersion, stillOn } from './swUpdate';

const S = () => useSwJobs.getState();
const set = (p: Partial<ReturnType<typeof S>>) => useSwJobs.setState(p);
const FILE = { uri: 'x', name: 'core-v1.2611.0-u22-x86_64.rbq', size: 300 << 20 };

describe('swUpdate — 화면 밖에서 이어지는 업데이트', () => {
  beforeEach(() => useSwJobs.setState({
    open: 0, unseen: null, prog: null, sendingOpt: false, err: null, doneMsg: false,
    dlTag: null, rowDl: false, dlProg: null, rowProg: null, appBusy: false, appMsg: null, appErr: null, file: FILE,
  }));

  it('패널이 떠 있으면 배너가 없다', () => {
    set({ open: 1, prog: { phase: 'transfer', percentage: 40, acked: 1, total: 2 } });
    expect(swBanner(S())).toBeNull();
  });

  it('보내는 중 % 와 설치 중을 알린다', () => {
    set({ prog: { phase: 'transfer', percentage: 57, acked: 1, total: 2 } });
    expect(swBanner(S())).toMatchObject({ tone: 'run', closable: false });
    expect(swBanner(S())!.text).toContain('v1.2611.0');
    expect(swBanner(S())!.text).toContain('57%');
    set({ prog: { phase: 'applying', percentage: 100, acked: 2, total: 2 } });
    expect(swBanner(S())!.text).toContain('설치하는 중');
  });

  it('패널 밖에서 전송이 끝나면 결과를 한 번 알린다 — 성공·실패', () => {
    set({ prog: { phase: 'applying', percentage: 100, acked: 2, total: 2 } });
    set({ prog: { phase: 'complete', percentage: 100, acked: 2, total: 2 }, doneMsg: true });
    expect(S().unseen).toBe('send');
    expect(swBanner(S())).toMatchObject({ tone: 'ok', closable: true });
    set({ unseen: null });
    expect(swBanner(S())).toBeNull();

    set({ prog: { phase: 'transfer', percentage: 10, acked: 1, total: 2 }, doneMsg: false });
    set({ prog: null, err: 'deploy failed' });
    expect(swBanner(S())).toMatchObject({ tone: 'bad' });
    expect(swBanner(S())!.text).toContain('deploy failed');
  });

  it('패널이 떠 있는 동안 끝난 일은 배너로 다시 알리지 않는다', () => {
    set({ open: 1, prog: { phase: 'transfer', percentage: 10, acked: 1, total: 2 } });
    set({ prog: { phase: 'complete', percentage: 100, acked: 2, total: 2 }, doneMsg: true });
    set({ open: 0 });
    expect(S().unseen).toBeNull();
    expect(swBanner(S())).toBeNull();
  });

  it('옵션 먼저 보낸 뒤 본체로 넘어가는 순간에도 결과로 오인하지 않는다', () => {
    set({ sendingOpt: true, prog: { phase: 'transfer', percentage: 90, acked: 1, total: 2 } });
    set({ sendingOpt: false, prog: null });
    set({ prog: { phase: 'connecting', percentage: 0, acked: 0, total: 2 } });
    expect(swBanner(S())).toMatchObject({ tone: 'run' });
  });

  it('받기가 끝나면 로봇에 보내라고 알린다', () => {
    set({ dlTag: 'v1.2611.0', dlProg: { written: 100 << 20, total: 300 << 20 } });
    expect(swBanner(S())!.text).toContain('100 MB / 300 MB');
    set({ dlTag: null, dlProg: null });
    expect(swBanner(S())).toMatchObject({ tone: 'ok' });
    expect(swBanner(S())!.text).toContain('로봇에 보내세요');
  });

  it('패키지 이름에서 버전을 읽는다', () => {
    expect(pkgVersion('core-v1.2611.0-1102-u22-x86_64.rbq')).toBe('v1.2611.0-1102');
    expect(pkgVersion('RBQ-v1.19.1.tar.gz')).toBe('v1.19.1');
    expect(pkgVersion('notes.txt')).toBeUndefined();
  });
});

describe('swUpdate — 로봇을 바꾸면 지난 로봇의 결과를 지운다', () => {
  beforeEach(() => {
    useRobot.setState({ ip: '192.168.0.10' });
    useSwJobs.setState({ open: 0, prog: null, sendingOpt: false, dlTag: null, rowDl: false,
      askReboot: true, doneMsg: true, unseen: 'send', err: null, file: FILE, sigFor: FILE.uri });
  });

  it('끝난 전송의 재부팅 질문·결과·고른 파일이 새 로봇으로 넘어가지 않는다', () => {
    useRobot.setState({ ip: '192.168.0.11' });
    expect(S()).toMatchObject({ askReboot: false, doneMsg: false, unseen: null, file: null, sigFor: null });
  });

  it('같은 주소로 다시 붙으면 그대로 둔다', () => {
    useRobot.setState({ ip: '192.168.0.10' });
    expect(S()).toMatchObject({ askReboot: true, doneMsg: true, unseen: 'send' });
  });

  it('보내는 중이면 건드리지 않는다 — 전송은 처음 고른 로봇으로 계속 간다', () => {
    set({ prog: { phase: 'transfer', percentage: 40, acked: 1, total: 2 } });
    useRobot.setState({ ip: '192.168.0.11' });
    expect(S().file).toEqual(FILE);
  });
});

describe('swUpdate — 보내는 도중 로봇을 바꾸면 끝나도 재부팅을 묻지 않는다', () => {
  it('보내기 시작한 로봇에 아직 붙어 있을 때만 참', () => {
    useRobot.setState({ ip: '192.168.0.10' });
    expect(stillOn('192.168.0.10')).toBe(true);
    useRobot.setState({ ip: '192.168.0.11' });
    expect(stillOn('192.168.0.10')).toBe(false);
  });
});
