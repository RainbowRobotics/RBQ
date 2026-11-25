import { describe, it, expect, beforeEach } from 'vitest';
import { useViewport, homeColumn, spectateTop, SPECTATE_PILL } from './viewport';

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

const dropdown = (ddBelow: boolean): [number, number] => (ddBelow ? [60, 104] : [4, 48]);

describe('homeColumn — 상단 가운데 열', () => {
  const winH = 360, bottomH = 68, toolH = 50, toast = 102;
  for (const ddBelow of [false, true]) for (const spectating of [false, true]) for (const statusH of [0, 23]) for (const toasts of [0, 1, 2]) {
    it(`겹침 없음·최신 ERROR 보임 — ddBelow=${ddBelow} spectating=${spectating} statusH=${statusH} toasts=${toasts}`, () => {
      const need = toasts ? toast : 0;
      const col = homeColumn({ ddBelow, spectating, statusH, bannerH: need, winH, bottomH, toolH, need });
      const limit = winH - bottomH - 8;
      const shown = toasts ? Math.max(1, Math.min(toasts, Math.floor((col.toastRoom + 6) / (toast + 6)))) : 0;
      const bannerH = shown ? shown * toast + (shown - 1) * 6 : 0;
      const boxes: [string, number, number][] = [['dropdown', ...dropdown(ddBelow)]];
      if (col.showSpectate) { const t = spectateTop(ddBelow) + SPECTATE_PILL.mt; boxes.push(['spectate', t, t + SPECTATE_PILL.h]); }
      if (statusH && col.showStatus) boxes.push(['status', col.status, col.status + statusH]);
      if (bannerH) boxes.push(['banner', col.banner, col.banner + bannerH]);
      const tool = col.banner + (bannerH ? bannerH + 8 : 0);
      if (tool + toolH <= limit) boxes.push(['tool', tool, tool + toolH]);
      boxes.push(['bottom', winH - bottomH, winH]);
      for (let i = 1; i < boxes.length; i++) expect(boxes[i][1], `${boxes[i - 1][0]} → ${boxes[i][0]}`).toBeGreaterThanOrEqual(boxes[i - 1][2]);
      if (toasts) expect(shown).toBeGreaterThanOrEqual(1);
    });
  }

  it('780×360 관전 + SLAM 알약 + ERROR 토스트 — 상태 알약이 비켜 ERROR 가 [관전] 알약 아래에 보인다', () => {
    const col = homeColumn({ ddBelow: true, spectating: true, statusH: 23, bannerH: toast, winH, bottomH, toolH, need: toast });
    expect(col.showStatus).toBe(false);
    expect(col.showSpectate).toBe(true);
    expect(col.banner).toBe(160);
    expect(col.banner + toast).toBeLessThanOrEqual(winH - bottomH - 8);
    expect(homeColumn({ ddBelow: true, spectating: true, statusH: 23, bannerH: 0, winH, bottomH, toolH, need: 0 }).showStatus).toBe(true);
  });

  it('아직 안 잰 상태 알약(높이 0)은 접지 않는다 — 그려야 잰다', () => {
    expect(homeColumn({ ddBelow: true, spectating: true, statusH: 0, bannerH: toast, winH, bottomH, toolH, need: toast }).showStatus).toBe(true);
  });

  it('그래도 모자라면 [관전] 알약도 비킨다(막·쓰기 잠금은 그대로) — ERROR 가 먼저다', () => {
    const col = homeColumn({ ddBelow: true, spectating: true, statusH: 23, bannerH: toast, winH: 300, bottomH, toolH, need: toast });
    expect(col.showSpectate).toBe(false);
    expect(col.banner + toast).toBeLessThanOrEqual(300 - bottomH - 8);
  });

  it('편집 막대 — 알약+토스트 하나면 PC 에선 들어가고, 폰(360)에선 숨긴다', () => {
    const base = { ddBelow: true, spectating: false, statusH: 23, bottomH, toolH };
    expect(homeColumn({ ...base, winH: 900, bannerH: 76, need: 76 }).toolFits).toBe(true);
    expect(homeColumn({ ...base, winH, bannerH: 160, need: 102 }).toolFits).toBe(false);
    expect(homeColumn({ ...base, bannerH: 0 }).toolFits).toBe(true);
  });

  it('종전 자리 그대로 — 알약이 없으면 104, 관전 중 116, 관전 중에 드롭다운이 바 아래면 160, PC 알약 76', () => {
    expect(homeColumn({ ddBelow: false, spectating: false, statusH: 0, bannerH: 0 }).banner).toBe(104);
    expect(homeColumn({ ddBelow: true, spectating: false, statusH: 0, bannerH: 0 }).banner).toBe(104);
    expect(homeColumn({ ddBelow: false, spectating: true, statusH: 0, bannerH: 0 }).banner).toBe(116);
    expect(homeColumn({ ddBelow: true, spectating: true, statusH: 0, bannerH: 0 }).banner).toBe(160);
    expect(homeColumn({ ddBelow: false, spectating: false, statusH: 23, bannerH: 0 }).status).toBe(76);
  });
});
