import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
vi.mock('./robot', async () => ({ useRobot: (await import('zustand')).create(() => ({ ip: '192.168.0.10', robot: undefined as unknown })) }));
import { useRobot } from './robot';
import { useZmpJob, zmpBegin, zmpBusy, zmpPhase, zmpLine } from './zmpCalib';
import { ZMP_CALIB } from '@/types/robot';

const S = () => useZmpJob.getState();
const zc = (run: number, state: number, more: { reason?: number; percent?: number } = {}) =>
  ({ run, state, reason: 0, percent: 0, err_mm: 0, ...more });
const robot = (z?: ReturnType<typeof zc>) => useRobot.setState({ robot: z ? { zmp_calib: z } : undefined } as never);

describe('무게 중심 보정 — 창을 닫아도 이번 런의 판정이 이어진다', () => {
  beforeEach(() => {
    vi.useFakeTimers();
    useRobot.setState({ ip: '192.168.0.10', robot: undefined } as never);
    useZmpJob.setState({ open: 0, unseen: false, startRun: null, cur: null, timedOut: false, lost: false });
  });
  afterEach(() => vi.useRealTimers());

  it('보내기 전 런의 결과(latch)는 무시하고 새 런을 따라간다 — 정렬·보정 %·완료', () => {
    robot(zc(7, ZMP_CALIB.done, { percent: 100 }));
    zmpBegin();
    expect(zmpPhase(S())).toBe('pending');
    expect(zmpLine(S())).toMatchObject({ tone: 'run', closable: false });
    robot(zc(8, ZMP_CALIB.aligning));
    expect(zmpPhase(S())).toBe('aligning');
    expect(zmpLine(S())!.text).toContain('정렬');
    robot(zc(8, ZMP_CALIB.running, { percent: 40 }));
    expect(zmpLine(S())!.text).toContain('40%');
    robot(zc(8, ZMP_CALIB.done, { percent: 100 }));
    expect(zmpPhase(S())).toBe('done');
    expect(S().unseen).toBe(true);
    expect(zmpLine(S())).toMatchObject({ tone: 'ok', closable: true });
  });

  it('도는 중이면 busy — 다시 연 창이 명령을 또 보내지 않는 근거', () => {
    zmpBegin();
    expect(zmpBusy(S())).toBe(true);
    robot(zc(1, ZMP_CALIB.running, { percent: 10 }));
    expect(zmpBusy(S())).toBe(true);
    robot(zc(1, ZMP_CALIB.failed, { reason: 3 }));
    expect(zmpBusy(S())).toBe(false);
  });

  it('실패하면 사유를 한 줄에', () => {
    zmpBegin();
    robot(zc(1, ZMP_CALIB.failed, { reason: 3 }));
    expect(zmpLine(S())).toMatchObject({ tone: 'bad', closable: true });
    expect(zmpLine(S())!.text).toContain('평지');
  });

  it('8초 안에 새 런이 안 오면 실패(로봇 응답 없음)', () => {
    robot(zc(7, ZMP_CALIB.done));
    zmpBegin();
    vi.advanceTimersByTime(7999);
    expect(zmpPhase(S())).toBe('pending');
    vi.advanceTimersByTime(1);
    expect(zmpPhase(S())).toBe('failed');
    expect(zmpLine(S())!.text).toContain('응답하지 않습니다');
  });

  it('이번 런이 끝난 뒤 다른 런(남이 돌린 것)은 따라가지 않는다', () => {
    zmpBegin();
    robot(zc(1, ZMP_CALIB.failed, { reason: 2 }));
    robot(zc(2, ZMP_CALIB.running, { percent: 50 }));
    expect(S().cur!.run).toBe(1);
    expect(zmpPhase(S())).toBe('failed');
  });

  it('창이 떠 있으면 배너가 없다', () => {
    zmpBegin();
    useZmpJob.setState({ open: 1 });
    expect(zmpLine(S())).toBeNull();
  });

  it('로봇이 재시작하면(런 번호가 줄거나 키가 빠짐 = 0) 이번 런은 끝 — 응답 없음, 다시 열면 새로 보낼 수 있다', () => {
    robot(zc(7, ZMP_CALIB.done));
    zmpBegin();
    robot(zc(8, ZMP_CALIB.running, { percent: 40 }));
    useRobot.setState({ robot: {} } as never);
    expect(zmpPhase(S())).toBe('failed');
    expect(zmpBusy(S())).toBe(false);
    expect(zmpLine(S())!.text).toContain('응답하지 않습니다');
    zmpBegin();
    robot(zc(9, ZMP_CALIB.running, { percent: 20 }));
    robot(zc(1, ZMP_CALIB.aligning));
    expect(zmpPhase(S())).toBe('failed');
  });

  it('최대 시간(사전 걸음 10초 + 수렴 60초)을 넉넉히 넘기면 응답 없음 — 상태 수신이 끊겨 N% 로 남지 않게', () => {
    zmpBegin();
    robot(zc(1, ZMP_CALIB.running, { percent: 40 }));
    vi.advanceTimersByTime(89_999);
    expect(zmpPhase(S())).toBe('running');
    vi.advanceTimersByTime(1);
    expect(zmpPhase(S())).toBe('failed');
    expect(zmpLine(S())!.text).toContain('응답하지 않습니다');
  });

  it('기한 전에 끝난 런은 기한이 지나도 그대로', () => {
    zmpBegin();
    robot(zc(1, ZMP_CALIB.done, { percent: 100 }));
    vi.advanceTimersByTime(120_000);
    expect(zmpPhase(S())).toBe('done');
  });

  it('로봇(주소)을 바꾸면 지난 로봇의 보정을 지운다 — 새 로봇의 런 번호로 오판하지 않게', () => {
    zmpBegin();
    useRobot.setState({ ip: '192.168.0.11' } as never);
    expect(zmpPhase(S())).toBeNull();
    robot(zc(5, ZMP_CALIB.done));
    expect(zmpLine(S())).toBeNull();
  });
});
