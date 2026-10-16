import { describe, it, expect, vi } from 'vitest';

vi.mock('@/lib/i18n', () => ({ t: (s: string) => s }));
import { coverage, timeAt, firstStart, clock, livePos, archiveErrText, DAY_MS, viewWindow, needsRecenter, ticks, spans, durLabel } from './archive';

const D0 = 1_759_762_800_000;
const DAY = { from: D0, to: D0 + DAY_MS };
const seg = (cam: string, fromMin: number, toMin: number) =>
  ({ cam, path: `${cam}.mp4`, start: D0 + fromMin * 60_000, end: D0 + toMin * 60_000 });

describe('상시 녹화 표시 함수', () => {
  it('30 분 roll 로 이어진 세그먼트는 한 구간, 카메라는 따로', () => {
    const segs = [seg('front', 60, 90), seg('front', 90, 120), seg('rear', 60, 90), seg('front', 600, 630)];
    const bands = coverage(segs, 'front', DAY);
    expect(bands).toHaveLength(2);
    expect(bands[0].from).toBeCloseTo(60 / 1440);
    expect(bands[0].to).toBeCloseTo(120 / 1440);
    expect(coverage(segs, 'rear', DAY)).toHaveLength(1);
  });

  it('막대 0.1 % 안쪽 틈은 잇고, 그보다 길면 끊는다', () => {
    expect(coverage([seg('front', 0, 30), { ...seg('front', 30, 60), start: D0 + 30 * 60_000 + 40_000 }], 'front', DAY)).toHaveLength(1);
    expect(coverage([seg('front', 0, 30), seg('front', 32, 60)], 'front', DAY)).toHaveLength(2);
  });

  it('자정을 넘긴 세그먼트는 하루 축 안으로 자른다', () => {
    const b = coverage([seg('front', 1430, 1460)], 'front', DAY);
    expect(b[0].to).toBe(1);
  });

  it('막대 비율 ↔ 시각, 범위 밖은 끝에 붙인다', () => {
    expect(timeAt(0.5, DAY)).toBe(D0 + DAY_MS / 2);
    expect(timeAt(-1, DAY)).toBe(D0);
    expect(timeAt(2, DAY)).toBe(D0 + DAY_MS);
  });

  it('첫 녹화 시작은 고른 카메라 기준', () => {
    const segs = [seg('rear', 10, 40), seg('front', 70, 100), seg('front', 50, 70)];
    expect(firstStart(segs, 'front')).toBe(D0 + 50 * 60_000);
    expect(firstStart(segs, 'side')).toBeNull();
  });

  it('시각 표기는 로봇 자정 기준(기기 시간대 무관)', () => {
    expect(clock(D0 + (13 * 3600 + 5 * 60 + 9) * 1000, D0)).toBe('13:05:09');
    expect(clock(D0 - 5000, D0)).toBe('00:00:00');
  });

  it('위치 보간 — 재생 중일 때만 경과×배속', () => {
    expect(livePos(1000, 0, 500, 4, true)).toBe(3000);
    expect(livePos(1000, 0, 500, 4, false)).toBe(1000);
  });

  it('명령 실패 코드를 안내 문구로', () => {
    expect(archiveErrText(new Error('/api/blackbox_video/playback → command status 404'))).toMatch(/업데이트/);
    expect(archiveErrText(new Error('x → command status 409'))).toMatch(/준비/);
    expect(archiveErrText(new Error('스트리머 command 응답 없음(8s)'))).toBe('스트리머 command 응답 없음(8s)');
  });

  it('맞춤 — 3 분 녹화도 막대에서 누를 수 있는 폭이 된다(최소 10 분 창)', () => {
    const segs = [seg('front', 716, 719)];
    const w = viewWindow('fit', segs, 'front', D0, null);
    expect(w.to - w.from).toBe(600_000);
    const b = coverage(segs, 'front', w);
    expect(b[0].to - b[0].from).toBeCloseTo(0.3);
  });

  it('맞춤 — 긴 녹화는 앞뒤 5 % 여유, 녹화 없으면 하루 전체', () => {
    const w = viewWindow('fit', [seg('front', 600, 720)], 'front', D0, null);
    expect(w.from).toBe(D0 + 594 * 60_000);
    expect(w.to).toBe(D0 + 726 * 60_000);
    expect(viewWindow('fit', [], 'front', D0, null)).toEqual(DAY);
  });

  it('고정 폭은 재생 위치를 가운데 두되 그 날 밖으로 안 나간다', () => {
    const at = D0 + 12 * 3600_000;
    expect(viewWindow('hour', [], 'front', D0, at)).toEqual({ from: at - 1800_000, to: at + 1800_000 });
    expect(viewWindow('hour', [], 'front', D0, D0 + 60_000)).toEqual({ from: D0, to: D0 + 3600_000 });
    expect(viewWindow('min10', [], 'front', D0, D0 + DAY_MS)).toEqual({ from: D0 + DAY_MS - 600_000, to: D0 + DAY_MS });
  });

  it('재생 위치가 막대 끝 5 % 에 닿으면 다시 가운데로', () => {
    const w = { from: 0, to: 1000 };
    expect(needsRecenter(w, 500)).toBe(false);
    expect(needsRecenter(w, 960)).toBe(true);
    expect(needsRecenter(w, -10)).toBe(true);
  });

  it('눈금 간격은 막대 폭에 맞춘다 — 24 h 는 3 시간, 1 h 는 10 분, 10 분은 1 분', () => {
    expect(ticks(DAY, D0).map((k) => k.label)).toEqual(['00:00', '03:00', '06:00', '09:00', '12:00', '15:00', '18:00', '21:00', '24:00']);
    const h = ticks({ from: D0 + 11.5 * 3600_000, to: D0 + 12.5 * 3600_000 }, D0);
    expect(h[0].label).toBe('11:30');
    expect(h).toHaveLength(7);
    expect(ticks({ from: D0 + 60_000 * 715, to: D0 + 60_000 * 725 }, D0)).toHaveLength(11);
  });

  it('녹화 구간 — 5 초 이하 틈은 하나로, 카메라별, 시간순', () => {
    const segs = [seg('front', 60, 90), { ...seg('front', 90, 120), start: D0 + 90 * 60_000 + 3000 }, seg('front', 200, 230), seg('rear', 0, 10)];
    expect(spans(segs, 'front')).toEqual([
      { start: D0 + 60 * 60_000, end: D0 + 120 * 60_000 },
      { start: D0 + 200 * 60_000, end: D0 + 230 * 60_000 },
    ]);
    expect(spans(segs, 'rear')).toHaveLength(1);
  });

  it('길이 표기', () => {
    expect(durLabel(40_000)).toBe('40초');
    expect(durLabel(43 * 60_000)).toBe('43분');
    expect(durLabel(65 * 60_000)).toBe('1시간 5분');
  });
});
