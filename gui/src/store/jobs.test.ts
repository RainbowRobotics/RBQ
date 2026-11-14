import { describe, it, expect, beforeEach } from 'vitest';
import { screenState, jobLine } from './jobs';

const job = screenState({ pct: null as number | null, err: null as string | null, done: false }, (s) => s.pct != null);
const S = () => job.getState();

describe('jobs — 화면 밖에서 이어지는 오래 걸리는 일', () => {
  beforeEach(() => job.setState({ pct: null, err: null, done: false, open: 0, unseen: false }));

  it('use(k) 는 [값, 바꾸기] — 바꾸기는 키마다 같은 함수', () => {
    const a = job.setter('pct'), b = job.setter('pct');
    expect(a).toBe(b);
    a(40);
    expect(S().pct).toBe(40);
  });

  it('화면이 없을 때 일이 끝나면 unseen — 결과를 한 번 알릴 수 있다', () => {
    job.setState({ pct: 10 });
    job.setState({ pct: null, done: true });
    expect(S().unseen).toBe(true);
  });

  it('화면이 떠 있는 동안 끝나면 unseen 이 아니다(화면에 이미 보인다)', () => {
    job.setState({ open: 1, pct: 10 });
    job.setState({ pct: null, done: true });
    expect(S().unseen).toBe(false);
  });

  it('jobLine — 화면이 떠 있으면 없음, 도는 중이면 run(닫기 없음), 끝난 결과는 unseen 일 때만 닫기와 함께', () => {
    const line = (s: ReturnType<typeof S>) => jobLine(s, {
      run: s.pct != null ? `보내는 중 ${s.pct}%` : null,
      bad: s.err ? `실패 — ${s.err}` : null,
      ok: s.done ? '끝남' : null,
    });
    job.setState({ pct: 30 });
    expect(line(S())).toEqual({ text: '보내는 중 30%', tone: 'run', closable: false });
    job.setState({ open: 1 });
    expect(line(S())).toBeNull();
    job.setState({ open: 0, pct: null, done: true, unseen: true });
    expect(line(S())).toEqual({ text: '끝남', tone: 'ok', closable: true });
    job.setState({ err: 'x' });
    expect(line(S())).toEqual({ text: '실패 — x', tone: 'bad', closable: true });
    job.setState({ unseen: false });
    expect(line(S())).toBeNull();
  });
});
