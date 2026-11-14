import { describe, it, expect, beforeEach, afterEach, vi } from 'vitest';
vi.mock('./robot', async () => ({ useRobot: (await import('zustand')).create(() => ({ ip: '192.168.0.10', robot: undefined as unknown })) }));
vi.mock('@/lib/rest', () => ({ actions: { cameraCheckStart: vi.fn(), cameraCheckGet: vi.fn() }, restErrorText: String }));
import { useRobot } from './robot';
import { useRobotCheckJob, beginRun, runView, robotCheckLine, useCameraCheck, cameraCheckLine } from './robotCheck';
import { clearCameraCheck, startCameraCheck } from '@/lib/cameraCheck';
import { actions } from '@/lib/rest';
import { DQ_CHECK, GAIT_CHECK, JOINT_COMM } from '@/types/robot';
import type { CameraCheck } from '@/lib/rest';

const J = () => useRobotCheckJob.getState();
const robot = (r: object) => useRobot.setState({ robot: r as never });
const dq = (o: object = {}) => ({
  run: 7, state: DQ_CHECK.done, reason: 0, percent: 0, leg: -1, ask_joint: -1, ask_left: 0,
  step_result: [], step_err_deg: [], err_deg: [], result: Array(12).fill(0), limit_deg: 6, ...o,
});
const gait = (o: object = {}) => ({ run: 3, state: GAIT_CHECK.done, reason: 0, percent: 0, mode: 0, duration_s: 10, x_mm: 0, y_mm: 0, yaw_deg: 0, ...o });

describe('로봇 점검 ②④⑤ — 화면을 벗어나도 이번 실행을 기억한다', () => {
  beforeEach(() => {
    vi.useFakeTimers();
    useRobot.setState({ ip: '192.168.0.10', robot: { dq_check: dq() } as never });
    useRobotCheckJob.setState({ dq: null, comm: null, drift: null, last: null, open: 1, unseen: false });
  });
  afterEach(() => vi.useRealTimers());

  it('시작하면 보내기 직전 런 번호를 적고, 화면 밖에서도 로봇 값을 따라가 진행·물음을 알린다', () => {
    const send = vi.fn();
    beginRun('dq', send);
    expect(send).toHaveBeenCalledOnce();
    expect(J().dq).toMatchObject({ startRun: 7, noReply: false });
    useRobotCheckJob.setState({ open: 0 });
    robot({ dq_check: dq({ run: 8, state: DQ_CHECK.running, percent: 40 }) });
    expect(robotCheckLine(J())).toMatchObject({ tone: 'run', closable: false });
    expect(robotCheckLine(J())!.text).toContain('모터 캘리브레이션 상태');
    expect(robotCheckLine(J())!.text).toContain('40%');
    robot({ dq_check: dq({ run: 8, state: DQ_CHECK.running, percent: 50, ask_joint: 4, ask_left: 25 }) });
    expect(robotCheckLine(J())!.text).toContain('응답이 필요합니다');
    expect(robotCheckLine(J())!.text).toContain('25');
    const v = runView('dq', J().dq, useRobot.getState().robot?.dq_check);
    expect(v.running).toBe(true);
    expect(v.cur?.ask_joint).toBe(4);
  });

  it('화면 밖에서 끝나면 결과를 한 번 — 불량·주의 수, 화면이 떠 있으면 없음', () => {
    beginRun('dq', () => {});
    useRobotCheckJob.setState({ open: 0 });
    robot({ dq_check: dq({ run: 8, state: DQ_CHECK.running }) });
    const result = Array(12).fill(0); result[1] = 1; result[5] = 4;
    robot({ dq_check: dq({ run: 8, state: DQ_CHECK.done, result }) });
    expect(robotCheckLine(J())).toMatchObject({ tone: 'bad', closable: true });
    expect(robotCheckLine(J())!.text).toContain('불량 1개');
    expect(robotCheckLine(J())!.text).toContain('주의 1개');
    useRobotCheckJob.setState({ open: 1 });
    expect(robotCheckLine(J())).toBeNull();
  });

  it('5초 안에 새 런이 안 보이면 무응답 — 화면 밖이어도 실패로 끝난다', () => {
    beginRun('comm', () => {});
    expect(J().comm?.startRun).toBe(0);
    useRobotCheckJob.setState({ open: 0 });
    expect(robotCheckLine(J())!.tone).toBe('run');
    vi.advanceTimersByTime(5000);
    expect(J().comm?.noReply).toBe(true);
    expect(robotCheckLine(J())).toMatchObject({ tone: 'bad', closable: true });
    expect(robotCheckLine(J())!.text).toContain('실패');
  });

  it('④ 계측이 없는 로봇(시뮬)은 판정 없이 완료, 기준을 넘은 관절은 주의 수', () => {
    beginRun('comm', () => {});
    useRobotCheckJob.setState({ open: 0 });
    const base = { run: 1, state: JOINT_COMM.done, reason: 0, percent: 100, duration_s: 10, amplitude_deg: 30, hz_warn: 400, gap_warn_ms: 10,
      rx_hz_min: Array(12).fill(500), gap_max_ms: Array(12).fill(3) };
    robot({ joint_comm_check: { ...base, measured: false } });
    expect(robotCheckLine(J())).toMatchObject({ tone: 'ok' });
    expect(robotCheckLine(J())!.text).toContain('완료');
    beginRun('comm', () => {});
    const hz = Array(12).fill(500); hz[0] = 100;
    robot({ joint_comm_check: { ...base, run: 2, measured: true, rx_hz_min: hz } });
    expect(robotCheckLine(J())!.text).toContain('주의 1개');
  });

  it('⑤ 는 제자리 걸음(mode 0)만 — 계단 왕복이 칸을 가져가도 이번 런으로 읽지 않고, 밀리면 주의', () => {
    robot({ gait_check: gait() });
    beginRun('drift', () => {});
    expect(J().drift?.startRun).toBe(3);
    useRobotCheckJob.setState({ open: 0 });
    robot({ gait_check: gait({ run: 4, state: GAIT_CHECK.walking, mode: 1 }) });
    expect(J().drift?.seen).toBeUndefined();
    robot({ gait_check: gait({ run: 5, state: GAIT_CHECK.walking, percent: 30 }) });
    expect(robotCheckLine(J())!.text).toContain('30%');
    robot({ gait_check: gait({ run: 5, state: GAIT_CHECK.done, x_mm: 150 }) });
    expect(robotCheckLine(J())).toMatchObject({ tone: 'bad' });
    expect(robotCheckLine(J())!.text).toContain('주의');
  });

  it('도는 중에 로봇 값이 10초 넘게 안 오면(연결 끊김) 무응답으로 끝낸다 — 값이 계속 오면 길어도 그대로', () => {
    beginRun('dq', () => {});
    useRobotCheckJob.setState({ open: 0 });
    for (let i = 0; i < 30; i++) {
      robot({ dq_check: dq({ run: 8, state: DQ_CHECK.running, percent: i }) });
      vi.advanceTimersByTime(1000);
    }
    expect(J().dq?.noReply).toBe(false);
    vi.advanceTimersByTime(10000);
    expect(J().dq?.noReply).toBe(true);
    expect(robotCheckLine(J())).toMatchObject({ tone: 'bad', closable: true });
  });

  it('로봇을 바꾸면 지난 로봇의 이번 실행을 지운다', () => {
    beginRun('dq', () => {});
    useRobot.setState({ ip: '192.168.0.20' });
    expect(J().dq).toBeNull();
  });
});

const cam = (o: Partial<CameraCheck>): CameraCheck => ({ state: 'running', duration_s: 10, elapsed_s: 4, started_at: null, error: null, sensors: [], ...o });
const sensor = (verdict: 'pass' | 'fail' | 'skip') => ({ verdict }) as never;

describe('카메라 점검 ① — 화면을 벗어나도 받아 오기를 이어 간다', () => {
  beforeEach(() => useCameraCheck.setState({ result: null, error: '', open: 0, unseen: false }));

  it('도는 동안 진행 %, 화면 밖에서 끝나면 통과·불량 n개', () => {
    useCameraCheck.setState({ result: cam({}) });
    expect(cameraCheckLine(useCameraCheck.getState())).toMatchObject({ tone: 'run', closable: false });
    expect(cameraCheckLine(useCameraCheck.getState())!.text).toContain('40%');
    useCameraCheck.setState({ result: cam({ state: 'done', elapsed_s: 10, sensors: [sensor('pass'), sensor('fail'), sensor('fail'), sensor('skip')] }) });
    expect(cameraCheckLine(useCameraCheck.getState())).toMatchObject({ tone: 'bad', closable: true });
    expect(cameraCheckLine(useCameraCheck.getState())!.text).toContain('불량 2개');
    useCameraCheck.setState({ result: cam({ state: 'done', sensors: [sensor('pass'), sensor('skip')] }) });
    expect(cameraCheckLine(useCameraCheck.getState())).toMatchObject({ tone: 'ok' });
    useCameraCheck.setState({ open: 1 });
    expect(cameraCheckLine(useCameraCheck.getState())).toBeNull();
  });

  it('측정 중 응답이 끊기면 기한(측정 시간 + 여유) 뒤 실패로 끝내고 폴링을 멈춘다', async () => {
    vi.useFakeTimers();
    vi.mocked(actions.cameraCheckStart).mockResolvedValue(cam({ elapsed_s: 0 }));
    vi.mocked(actions.cameraCheckGet).mockRejectedValue(new Error('down'));
    await startCameraCheck('192.168.0.10');
    await vi.advanceTimersByTimeAsync(40000);
    expect(useCameraCheck.getState().result?.state).toBe('failed');
    expect(useCameraCheck.getState().error).toContain('응답하지 않습니다');
    expect(cameraCheckLine(useCameraCheck.getState())).toMatchObject({ tone: 'bad', closable: true });
    const calls = vi.mocked(actions.cameraCheckGet).mock.calls.length;
    await vi.advanceTimersByTimeAsync(5000);
    expect(vi.mocked(actions.cameraCheckGet).mock.calls.length).toBe(calls);
    vi.useRealTimers();
  });

  it('로봇을 바꾸면 폴링을 멈추고 결과를 비운다', async () => {
    vi.useFakeTimers();
    vi.mocked(actions.cameraCheckStart).mockResolvedValue(cam({ elapsed_s: 0 }));
    vi.mocked(actions.cameraCheckGet).mockResolvedValue(cam({ elapsed_s: 2 }));
    await startCameraCheck('192.168.0.10');
    useRobot.setState({ ip: '192.168.0.30' });
    expect(useCameraCheck.getState().result).toBeNull();
    const calls = vi.mocked(actions.cameraCheckGet).mock.calls.length;
    await vi.advanceTimersByTimeAsync(5000);
    expect(vi.mocked(actions.cameraCheckGet).mock.calls.length).toBe(calls);
    expect(useCameraCheck.getState().result).toBeNull();
    vi.useRealTimers();
  });

  it('화면을 벗어날 때 도는 중이면 결과를 지우지 않는다 — 끝난 것만 스냅샷이라 지운다', () => {
    useCameraCheck.setState({ result: cam({}) });
    clearCameraCheck();
    expect(useCameraCheck.getState().result?.state).toBe('running');
    useCameraCheck.setState({ result: cam({ state: 'done' }) });
    clearCameraCheck();
    expect(useCameraCheck.getState().result).toBeNull();
  });
});
