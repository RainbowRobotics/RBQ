import { describe, it, expect, beforeEach, vi } from 'vitest';
vi.mock('./robot', async () => ({ useRobot: (await import('zustand')).create(() => ({ ip: '192.168.0.10' })) }));
import { useRobot } from './robot';
import { useOptionJob, optionLine } from './optionInstall';

const S = () => useOptionJob.getState();
const ROW = { file: { uri: 'x', name: 'option-demo-v1.2611.0-u22-x86_64.rbo', size: 1 << 20 }, opt: { name: 'demo', label: 'v1.2611.0', system: 'u22-x86_64' }, cached: true };

describe('옵션 설치 — 업데이트 › 옵션을 벗어나도 이어진다', () => {
  beforeEach(() => useOptionJob.setState({ open: 0, unseen: false, busy: null, prog: null, err: null, done: null, removed: null, picked: null }));

  it('보내는 중·로봇에서 설치 중·제거 중을 알린다(닫기 없음)', () => {
    useOptionJob.setState({ busy: ROW.file.name, busyTitle: 'DEMO 옵션 v1.2611.0', prog: { phase: 'transfer', percentage: 42, acked: 1, total: 2 } });
    expect(optionLine(S())).toMatchObject({ tone: 'run', closable: false });
    expect(optionLine(S())!.text).toContain('DEMO');
    expect(optionLine(S())!.text).toContain('42%');
    useOptionJob.setState({ prog: { phase: 'applying', percentage: 100, acked: 2, total: 2 } });
    expect(optionLine(S())!.text).toContain('설치하는 중');
    useOptionJob.setState({ busy: 'remove', prog: null });
    expect(optionLine(S())!.text).toContain('제거하는 중');
  });

  it('화면 밖에서 끝나면 결과를 한 번 — 성공·실패, 화면이 떠 있으면 없음', () => {
    useOptionJob.setState({ busy: ROW.file.name, busyTitle: 'DEMO 옵션 v1.2611.0' });
    useOptionJob.setState({ busy: null, done: 'DEMO v1.2611.0 — 설치했습니다' });
    expect(optionLine(S())).toMatchObject({ tone: 'ok', closable: true });
    useOptionJob.setState({ err: '서명이 맞지 않습니다' });
    expect(optionLine(S())).toMatchObject({ tone: 'bad', closable: true });
    useOptionJob.setState({ open: 1 });
    expect(optionLine(S())).toBeNull();
  });
});

describe('옵션 설치 — 로봇을 바꾸면 지난 로봇의 결과를 지운다', () => {
  beforeEach(() => { useRobot.setState({ ip: '192.168.0.10' }); useOptionJob.setState({ busy: null, done: 'DEMO 옵션 — 설치했습니다', err: null, removed: 'demo', installed: 'demo', unseen: true }); });
  it('끝난 결과·제거 안내가 새 로봇에 붙지 않는다', () => {
    useRobot.setState({ ip: '192.168.0.11' });
    expect(S()).toMatchObject({ done: null, removed: null, installed: null, unseen: false });
  });
  it('설치 중이면 둔다 — 전송은 처음 고른 로봇으로 계속 간다', () => {
    useOptionJob.setState({ busy: 'option-demo-v1.2611.0-u22-x86_64.rbo' });
    useRobot.setState({ ip: '192.168.0.11' });
    expect(S().done).toBe('DEMO 옵션 — 설치했습니다');
  });
});
