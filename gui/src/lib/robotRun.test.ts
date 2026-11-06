import { describe, it, expect } from 'vitest';
import { robotRunView } from './robotRun';

const DONE = 3, FAILED = 4, WALKING = 1;
type Info = { run: number; state: number; mode: number };
const forStage5 = (i: Info | undefined) => (i && i.mode === 0 ? i : undefined);

describe('robotRunView', () => {
  it('⑤ 를 마친 뒤 계단 왕복(mode 1)이 칸을 가져가도 ⑤ 결과가 남는다 — 무응답으로 바뀌지 않는다', () => {
    const start = 7;
    const done5: Info = { run: 8, state: DONE, mode: 0 };
    const a = robotRunView(forStage5(done5), start, undefined, DONE);
    expect(a.shown).toEqual(done5);

    const shuttle: Info = { run: 9, state: WALKING, mode: 1 };
    const b = robotRunView(forStage5(shuttle), start, a.cur, DONE);
    expect(b.cur).toEqual(done5);
    expect(b.shown).toEqual(done5);
  });

  it('중단된 런은 값을 그리지 않고, 실패 사유는 cur 로 읽는다', () => {
    const failed: Info = { run: 8, state: FAILED, mode: 0 };
    const v = robotRunView(failed, 7, undefined, DONE);
    expect(v.cur).toEqual(failed);
    expect(v.shown).toBeUndefined();
  });

  it('진행 중에도 값을 그리지 않는다 — 완료 전 부분 값', () => {
    const v = robotRunView<Info>({ run: 8, state: WALKING, mode: 0 }, 7, undefined, DONE);
    expect(v.cur?.state).toBe(WALKING);
    expect(v.shown).toBeUndefined();
  });

  it('아직 이번 런이 안 보이면 cur 가 없다 — 훅이 5초 무응답 타이머를 건다', () => {
    const v = robotRunView<Info>({ run: 7, state: DONE, mode: 0 }, 7, undefined, DONE);
    expect(v.cur).toBeUndefined();
    expect(v.shown).toBeUndefined();
  });

  it('이번 화면에서 잰 게 없으면 로봇에 남은 지난 완료 결과를 보여 준다', () => {
    const last: Info = { run: 5, state: DONE, mode: 0 };
    expect(robotRunView(last, null, undefined, DONE).shown).toEqual(last);
    expect(robotRunView<Info>({ run: 5, state: FAILED, mode: 0 }, null, undefined, DONE).shown).toBeUndefined();
  });
});
