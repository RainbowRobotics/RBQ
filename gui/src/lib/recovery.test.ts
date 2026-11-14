import { describe, it, expect, vi, beforeEach } from 'vitest';
const sent: number[][] = [];
vi.mock('@/lib/userCommand', () => ({ sendUserCommand: (_p: number, _c: number, args: number[] = []) => { sent.push(args); } }));
vi.mock('@/lib/robotState', () => ({ PROGRAM: { WalkReady: 1 }, WALKREADY_CMD: { GO_RECOVERY_READY: 1, FALL_RECOVERY_MOTION: 2, JOINT_LOCK_UNLOCK: 3, JOINT_SPACE_JOG: 4 } }));
import { jogHold } from './recovery';

describe('관절 조그 — 누른 채 패널이 사라져도 멈춘다', () => {
  beforeEach(() => { sent.length = 0; });

  it('누름 → 뗌: 시작·정지 한 번씩', () => {
    const h = jogHold(4);
    h.start(true); h.stop(); h.dispose();
    expect(sent).toEqual([[4, 1, 1], [4, 0, 0]]);
  });

  it('누른 채 사라지면(onPressOut 없음) 정지를 보낸다 — 로봇 조그는 정지가 올 때까지 관절 한계까지 간다', () => {
    const h = jogHold(4);
    h.start(false); h.dispose();
    expect(sent).toEqual([[4, -1, 1], [4, 0, 0]]);
  });

  it('안 누른 채 사라지면 아무것도 안 보낸다', () => {
    jogHold(4).dispose();
    expect(sent).toEqual([]);
  });
});
