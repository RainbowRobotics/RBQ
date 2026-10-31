import { describe, it, expect, beforeEach } from 'vitest';
import { useViewport } from './viewport';

const mode = { key: 'cctv', exit: () => {}, leave: () => {} };

describe('모듈 화면 모드', () => {
  beforeEach(() => useViewport.setState({ key: 'front', viewMode: null }));

  it('소스를 다른 것으로 바꾸면 꺼진다', () => {
    useViewport.getState().setKey('cctv');
    useViewport.getState().setViewMode(mode);
    useViewport.getState().setKey('front');
    expect(useViewport.getState().viewMode).toBeNull();
  });

  it('같은 소스로 다시 고르는 것은 모드를 유지한다', () => {
    useViewport.getState().setKey('cctv');
    useViewport.getState().setViewMode(mode);
    useViewport.getState().setKey('cctv');
    expect(useViewport.getState().viewMode).toBe(mode);
  });

  it('소스를 바꿔 꺼질 때 leave 를 한 번 부른다 — 드롭다운으로 나가도 정리가 같다', () => {
    let left = 0;
    useViewport.getState().setKey('cctv');
    useViewport.getState().setViewMode({ ...mode, leave: () => { left++; } });
    useViewport.getState().setKey('cctv');
    useViewport.getState().setKey('front');
    useViewport.getState().setKey('rear');
    expect(left).toBe(1);
  });

  it('소스를 고르는 것만으로는 켜지지 않는다 — PTZ 행·드롭다운은 PTZ 짐벌 쪽', () => {
    useViewport.getState().setKey('cctv');
    expect(useViewport.getState().viewMode).toBeNull();
  });
});
