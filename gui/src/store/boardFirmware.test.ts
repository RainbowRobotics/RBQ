import { describe, it, expect, beforeEach, vi } from 'vitest';

vi.mock('./robot', () => ({ useRobot: { getState: () => ({ conn: 'disconnected', ip: '' }), subscribe: vi.fn() } }));
vi.mock('./capability', () => ({ useFeatures: { getState: () => ({ features: null }), subscribe: vi.fn() } }));
vi.mock('@/lib/rest', () => ({ rest: { boardFw: vi.fn() } }));

import { useBoardFw, runResultStale } from './boardFirmware';
import { parseFwStatus, motorGroup, type FwStatus } from '@/lib/boardFirmware';

type Run = { id: number; kind: string; state: string; stage: string; board: number; reason: number };
const NO_RUN: Run = { id: 0, kind: 'none', state: 'none', stage: 'none', board: -1, reason: 0 };

const mk = (seq: number, motorsPresent: boolean, run: Run = NO_RUN, busy = 'none'): FwStatus => parseFwStatus({
  supported: true, check_seq: seq, busy, run,
  boards: Array.from({ length: 21 }, (_, slot) => ({
    slot, mode: 'app', result: 'none', reason: 0,
    present: slot >= 16 || motorsPresent,
    verdict: slot >= 16 || motorsPresent ? 'latest' : 'no_response',
    hw: { state: slot >= 16 || motorsPresent ? 'ok' : 'unchecked', ver: slot >= 16 || motorsPresent ? '1.2.0.0' : null },
  })),
})!;

beforeEach(() => useBoardFw.getState().reset());

describe('버전 확인 중 표', () => {
  it('확인 번호가 오를 때까지 지난 표를 둔다 · 진행 표시는 새 값', () => {
    const { apply } = useBoardFw.getState();
    apply(mk(3, true));
    apply(mk(3, false, { ...NO_RUN, state: 'checking' }, 'check'));
    let s = useBoardFw.getState().status!;
    expect(s.run.state).toBe('checking');
    expect(motorGroup(s).verdict).toBe('latest');
    expect(s.boards[0].hw.ver).toBe('1.2.0.0');
    apply(mk(3, false, { ...NO_RUN, state: 'checking' }, 'update'));
    expect(motorGroup(useBoardFw.getState().status!).verdict).toBe('latest');
    apply(mk(4, false));
    s = useBoardFw.getState().status!;
    expect(motorGroup(s).verdict).toBe('legs_off');
    expect(useBoardFw.getState().checkedAt).not.toBeNull();
  });
  it('처음 받은 값이 확인 중이면 그대로 쓴다(둘 지난 표가 없다)', () => {
    useBoardFw.getState().apply(mk(0, false, { ...NO_RUN, state: 'checking' }, 'check'));
    expect(motorGroup(useBoardFw.getState().status!).verdict).toBe('legs_off');
  });
  it('업데이트 진행 중에는 표를 묶지 않는다(굽는 칸의 진행이 보여야 한다)', () => {
    const { apply } = useBoardFw.getState();
    apply(mk(3, true));
    apply(mk(3, false, { id: 2, kind: 'board', state: 'running', stage: 'leg_on', board: -1, reason: 0 }, 'update'));
    expect(motorGroup(useBoardFw.getState().status!).verdict).toBe('legs_off');
  });
});

describe('끝난 업데이트 결과 카드', () => {
  it('끝난 뒤 버전 확인이 또 끝나면 지난 일 — 다음 업데이트가 끝나면 다시 보인다', () => {
    const { apply } = useBoardFw.getState();
    const rejected: Run = { id: 6, kind: 'board', state: 'rejected', stage: 'leg_off', board: -1, reason: 7 };
    apply(mk(5, false, rejected));
    expect(runResultStale(useBoardFw.getState())).toBe(false);
    apply(mk(5, true, rejected));
    expect(runResultStale(useBoardFw.getState())).toBe(false);
    apply(mk(6, true, rejected));
    expect(runResultStale(useBoardFw.getState())).toBe(true);
    apply(mk(7, true, { ...rejected, id: 7, state: 'success', board: -1 }));
    expect(runResultStale(useBoardFw.getState())).toBe(false);
  });
  it('진행 중 · 업데이트 없음은 해당 없음', () => {
    const { apply } = useBoardFw.getState();
    apply(mk(5, true));
    apply(mk(6, true));
    expect(runResultStale(useBoardFw.getState())).toBe(false);
    expect(useBoardFw.getState().resultSeen).toBeNull();
  });
});
