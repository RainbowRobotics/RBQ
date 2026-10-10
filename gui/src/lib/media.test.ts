import { describe, it, expect, vi } from 'vitest';

vi.mock('@/lib/rest', () => ({ rest: {} }));
vi.mock('@/lib/webrtcClient', () => ({ webrtcClient: {} }));
vi.mock('@/store/robot', () => ({ useRobot: { getState: () => ({ ip: '' }) } }));
vi.mock('@/lib/demoFlag', () => ({ isDemo: () => false }));
vi.mock('@/lib/assetBytes', () => ({ assetBytes: async () => new ArrayBuffer(0) }));
import { groupCaptures, groupByDay, dayLabel, clipSeconds, mmss, stampParts, prefixOf, userSoundName } from './media';

const f = (path: string, mtime = 0, size = 1000) => ({ path, mtime, size });

describe('media 표시 함수', () => {
  it('같은 초의 전방·후방을 촬영 하나로 묶고 최신이 먼저 온다', () => {
    const caps = groupCaptures([
      f('20260910/front_20260910_175050.jpg', 1), f('20260910/rear_20260910_175050.jpg', 2),
      f('20260910/front_20260910_180000.jpg', 9),
    ]);
    expect(caps.map((c) => c.key)).toEqual(['20260910_180000', '20260910_175050']);
    expect(caps[1].front?.path).toContain('front_');
    expect(caps[1].rear?.path).toContain('rear_');
    expect(caps[0].rear).toBeUndefined();
  });

  it('카메라 이름이 아닌 파일은 other 로 — 버리지 않는다', () => {
    const [c] = groupCaptures([f('20260910/side_20260910_175050.jpg')]);
    expect(c.other).toHaveLength(1);
  });

  it('날짜 묶음은 입력 순서를 지킨다', () => {
    const g = groupByDay(['20260910', '20260910', '20260909'], (d) => d);
    expect(g.map((x) => [x.day, x.items.length])).toEqual([['20260910', 2], ['20260909', 1]]);
  });

  it('오늘·어제는 말로, 그 전은 날짜로', () => {
    const now = new Date(2026, 8, 10, 12);
    expect(dayLabel('20260910', now)).toBe('오늘');
    expect(dayLabel('20260909', now)).toBe('어제');
    expect(dayLabel('20260901', now)).toMatch(/^9\/1 \(/);
    expect(dayLabel('', now)).toBe('날짜 미상');
  });

  it('녹음 길이는 48 kHz mono 16-bit 기준 — 헤더만 있는 파일은 0초', () => {
    expect(clipSeconds(44)).toBe(0);
    expect(clipSeconds(44 + 96000 * 3)).toBe(3);
    expect(mmss(125.9)).toBe('2:05');
  });

  it('파일명에서 시각·앞머리를 뽑는다', () => {
    expect(stampParts('20260910/mic_20260910_175101.wav')).toEqual({ day: '20260910', time: '17:51:01', key: '20260910_175101' });
    expect(stampParts('weird.wav')).toBeNull();
    expect(prefixOf('20260910/uplink_20260910_175101.wav')).toBe('uplink');
  });
});

describe('userSoundName', () => {
  it('내장 소리와 겹치지 않게 my- 를 붙이고 경로·특수문자를 뺀다', () => {
    expect(userSoundName('경고 방송 1.MP3')).toBe('my-경고_방송_1.mp3');
    expect(userSoundName('../../etc/passwd')).toBe('my-etc_passwd.bin');
    expect(userSoundName('.wav')).toBe('my-sound.wav');
  });
});

describe('staleSoundPaths', () => {
  it('같은 id 의 옛 개정만 고른다', async () => {
    const { staleSoundPaths } = await import('./media');
    const files = ['library/dog.wav', 'library/dog-r1.wav', 'library/dog-r2.wav', 'library/dog-house.wav', 'library/my-dog.wav', '20261006/mic_1.wav'];
    expect(staleSoundPaths('dog', 'library/dog-r2.wav', files)).toEqual(['library/dog.wav', 'library/dog-r1.wav']);
  });
});
