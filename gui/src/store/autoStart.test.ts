import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
vi.mock('./robot', async () => ({ useRobot: (await import('zustand')).create(() => ({ ip: '192.168.0.10', robot: undefined as unknown })) }));
import { useRobot } from './robot';
import { useAutoStartJob, autoStartFresh, autoStartSent, autoStartOk, autoStartLine } from './autoStart';
import { INIT_STATE, INIT_STEPS } from '@/types/robot';

const S = () => useAutoStartJob.getState();
const CTRL = INIT_STEPS.indexOf('control');
const as = (p: { running?: boolean; start?: number; step?: number; fail?: number; ctrl?: number; fail_ch?: number; emo?: boolean }) => {
  const steps = Array(9).fill(INIT_STATE.idle);
  if (p.ctrl != null) steps[CTRL] = p.ctrl;
  if (p.fail != null) steps[p.fail] = INIT_STATE.fail;
  return { running: !!p.running, start_unix_ms: p.start, step: p.fail ?? p.step ?? 0, steps, fail_ch: p.fail_ch ?? -1,
    fail_code: 0, ch_home: [], emo_blocked: p.emo };
};
const robot = (a?: ReturnType<typeof as>) => useRobot.setState({ robot: a ? { autostart: a } : undefined } as never);

describe('자동 기동 — 창을 닫아도 이번 실행의 성공·실패 판정이 이어진다', () => {
  beforeEach(() => {
    vi.useFakeTimers();
    useRobot.setState({ ip: '192.168.0.10', robot: undefined } as never);
    useAutoStartJob.setState({ open: 0, unseen: false, started: false, pending: false, sawRun: false, running: false, startMs: null, info: null, lost: false });
  });
  afterEach(() => vi.useRealTimers());

  it('보낸 뒤 창을 닫아도 — 진행 중 → 성공을 한 번 알린다', () => {
    robot(as({ start: 100, ctrl: INIT_STATE.pass }));
    autoStartFresh();
    expect(autoStartOk(S())).toBe(false);
    autoStartSent();
    expect(autoStartLine(S())).toMatchObject({ tone: 'run', closable: false });
    robot(as({ running: true, start: 200, step: 3 }));
    expect(S()).toMatchObject({ sawRun: true, pending: false, running: true });
    expect(autoStartLine(S())!.text).toContain('진행 중');
    robot(as({ start: 200, ctrl: INIT_STATE.pass }));
    expect(autoStartOk(S())).toBe(true);
    expect(S().unseen).toBe(true);
    expect(autoStartLine(S())).toMatchObject({ tone: 'ok', closable: true });
  });

  it('EMO 로 멈춘 것도 성공 — 풀면 선다고 알린다', () => {
    autoStartSent();
    robot(as({ running: true, start: 1 }));
    robot(as({ start: 1, ctrl: INIT_STATE.warn, emo: true }));
    expect(autoStartLine(S())).toMatchObject({ tone: 'ok' });
    expect(autoStartLine(S())!.text).toContain('E-STOP');
  });

  it('실패하면 단계의 사유를 한 줄에', () => {
    autoStartSent();
    robot(as({ running: true, start: 300 }));
    robot(as({ start: 300, fail: INIT_STEPS.indexOf('comm'), fail_ch: 11 }));
    expect(autoStartOk(S())).toBe(false);
    expect(autoStartLine(S())).toMatchObject({ tone: 'bad', closable: true });
    expect(autoStartLine(S())!.text).toContain('보드가 응답하지 않습니다');
  });

  it('곧바로 실패해 running 을 못 봐도 시작 시각이 바뀌면 이번 런이다', () => {
    robot(as({ start: 100, ctrl: INIT_STATE.pass }));
    autoStartSent();
    robot(as({ start: 400, fail: INIT_STEPS.indexOf('precheck') }));
    expect(S()).toMatchObject({ sawRun: true, pending: false });
    expect(autoStartLine(S())!.text).toContain('이미 서 있는 상태');
  });

  it('8초 동안 데몬이 아무것도 안 알리면 응답 없음', () => {
    autoStartSent();
    vi.advanceTimersByTime(8000);
    expect(S().pending).toBe(false);
    expect(autoStartLine(S())).toMatchObject({ tone: 'bad' });
    expect(autoStartLine(S())!.text).toContain('응답하지 않습니다');
  });

  it('도는 중에 다시 열면 지우지 않는다 / 끝난 뒤 새로 열면 지운다([자동 기동]이 다시 뜬다)', () => {
    autoStartSent();
    robot(as({ running: true, start: 5 }));
    autoStartFresh();
    expect(S()).toMatchObject({ started: true, sawRun: true });
    robot(as({ start: 5, ctrl: INIT_STATE.pass }));
    autoStartFresh();
    expect(S()).toMatchObject({ started: false, sawRun: false });
    expect(autoStartOk(S())).toBe(false);
  });

  it('창도 없고 보내지도 않은 기동(남이 시작한 것)은 따라가지도 알리지도 않는다', () => {
    robot(as({ running: true, start: 9 }));
    expect(S().running).toBe(false);
    expect(autoStartLine(S())).toBeNull();
  });

  it('창이 떠 있는 동안 본 런은 닫은 뒤에도 이어 본다', () => {
    useAutoStartJob.setState({ open: 1 });
    autoStartFresh();
    robot(as({ running: true, start: 9 }));
    useAutoStartJob.setState({ open: 0 });
    expect(autoStartLine(S())).toMatchObject({ tone: 'run' });
    robot(as({ start: 9, ctrl: INIT_STATE.pass }));
    expect(autoStartLine(S())).toMatchObject({ tone: 'ok' });
  });

  it('기동 중 상태가 10초 넘게 안 오면(연결 끊김 — 마지막 값이 running 으로 남는다) 응답 없음, 다시 시작할 수 있다', () => {
    autoStartSent();
    robot(as({ running: true, start: 7 }));
    vi.advanceTimersByTime(9_999);
    expect(S().running).toBe(true);
    vi.advanceTimersByTime(1);
    expect(S()).toMatchObject({ running: false, pending: false });
    expect(autoStartLine(S())).toMatchObject({ tone: 'bad' });
    expect(autoStartLine(S())!.text).toContain('응답하지 않습니다');
  });

  it('상태가 계속 오면 오래 걸려도 끊지 않는다', () => {
    autoStartSent();
    for (let i = 0; i < 12; i++) { robot(as({ running: true, start: 7, step: 6 })); vi.advanceTimersByTime(5_000); }
    expect(S().running).toBe(true);
    robot(as({ start: 7, ctrl: INIT_STATE.pass }));
    expect(autoStartLine(S())).toMatchObject({ tone: 'ok' });
  });

  it('로봇(주소)을 바꾸면 지난 로봇의 기동을 지운다', () => {
    autoStartSent();
    useRobot.setState({ ip: '192.168.0.11' } as never);
    expect(S()).toMatchObject({ started: false, pending: false });
    expect(autoStartLine(S())).toBeNull();
  });
});
