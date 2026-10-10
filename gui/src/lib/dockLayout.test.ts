import { describe, it, expect } from 'vitest';
import { DOCK_PANES, paneIndexOf, paneU } from './dockLayout';
import { MARKER_CAM } from './arucoState';

describe('dockLayout', () => {
  it('칸 순서가 서버(Front, Rear, Bottom2)와 같다', () => {
    expect(DOCK_PANES).toEqual([MARKER_CAM.front, MARKER_CAM.rear, MARKER_CAM.bottom]);
  });

  it('markerCam 을 칸 번호로 옮긴다', () => {
    expect(paneIndexOf(MARKER_CAM.front)).toBe(0);
    expect(paneIndexOf(MARKER_CAM.rear)).toBe(1);
    expect(paneIndexOf(MARKER_CAM.bottom)).toBe(2);
    expect(paneIndexOf(9)).toBe(-1);
  });

  it('칸 안 정규화 u 가 칸 경계 안으로 접힌다', () => {
    for (let i = 0; i < 3; i++) {
      expect(paneU(i, 0)).toBeCloseTo(i / 3);
      expect(paneU(i, 1)).toBeCloseTo((i + 1) / 3);
      expect(paneU(i, 0.5)).toBeCloseTo((i + 0.5) / 3);
    }
  });

  it('서버의 정수 칸 경계와 1px 안에서 일치한다', () => {
    const W = 1280;
    for (let i = 0; i < 3; i++) {
      expect(Math.abs(paneU(i, 0) * W - Math.floor((W * i) / 3))).toBeLessThan(1);
    }
  });
});
