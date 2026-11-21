import { describe, it, expect, beforeEach } from 'vitest';
import { useLogSave, logSaveLine, logSaveCanShare, logSaveJob } from './logSave';

const S = () => useLogSave.getState();
const ZIP = 'file:///cache/blackbox-RBQ10_20261112_10_00_00.zip';

describe('로그 저장 — 로그 화면을 벗어나도 이어진다', () => {
  beforeEach(() => useLogSave.setState({ open: 0, unseen: false, saving: false, what: '', err: '', done: null, share: null }));

  it('저장 중을 알린다(닫기 없음)', () => {
    useLogSave.setState({ saving: true, what: 'Black Box' });
    expect(logSaveLine(S())).toMatchObject({ tone: 'run', closable: false });
    expect(logSaveLine(S())!.text).toContain('Black Box');
  });

  it('화면이 떠 있으면 공유 시트를 바로, 없으면 미뤄 둔다', () => {
    useLogSave.setState({ open: 1 });
    expect(logSaveCanShare(ZIP)).toBe(true);
    expect(S().share).toBeNull();
    useLogSave.setState({ open: 0 });
    expect(logSaveCanShare(ZIP)).toBe(false);
    expect(S().share).toBe(ZIP);
  });

  it('화면 밖에서 끝나면 결과를 한 번 — 미룬 공유는 눌러서 공유, 실패가 먼저, 화면이 떠 있으면 없음', () => {
    useLogSave.setState({ saving: true, what: 'Black Box' });
    useLogSave.setState({ saving: false, done: 'blackbox.zip', share: ZIP });
    expect(S().unseen).toBe(true);
    expect(logSaveLine(S())).toMatchObject({ tone: 'ok', closable: true });
    expect(logSaveLine(S())!.text).toContain('눌러서 공유');
    useLogSave.setState({ err: '기기 저장공간이 부족합니다' });
    expect(logSaveLine(S())).toMatchObject({ tone: 'bad', closable: true });
    expect(logSaveLine(S())!.text).toContain('저장공간');
    useLogSave.setState({ open: 1 });
    expect(logSaveLine(S())).toBeNull();
  });

  it('미룬 공유가 없으면(웹 — 브라우저가 이미 받았다) 저장했다고만', () => {
    useLogSave.setState({ saving: true, what: 'System Log' });
    useLogSave.setState({ saving: false, done: 'systemlog-20261112.jsonl' });
    expect(logSaveLine(S())!.text).toContain('저장했습니다');
    expect(logSaveLine(S())!.text).not.toContain('공유');
  });

  it('배너 X 는 결과를 본 것 — 다음에 로그 화면을 열어도 지난 결과·실패 창이 다시 뜨지 않는다', () => {
    useLogSave.setState({ unseen: true, done: 'blackbox.zip', err: '기기 저장공간이 부족합니다', share: ZIP });
    logSaveJob.dismiss();
    expect(S()).toMatchObject({ unseen: false, done: null, err: '', share: null });
  });
});
