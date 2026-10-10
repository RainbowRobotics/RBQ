import { describe, it, expect, vi } from 'vitest';
import { zipSync, strToU8 } from 'fflate';

vi.mock('@/lib/rest', () => ({ rest: {} }));
import { buildSession, buildVideo, parseBlackboxZip } from './blackbox';

const META = JSON.stringify({
  data_frame_count: 3000, data_start_epoch_ms: 1788507060092, data_tick_ms: 10,
  trigger_epoch_ms: 1788507090092,
});
const VIDEO = JSON.stringify({
  clip_end_epoch_ms: 1788507090089,
  front: { duration_ms: 29902, ok: true, start_epoch_ms: 1788507060182 },
  rear:  { duration_ms: 29902, ok: true, start_epoch_ms: 1788507060183 },
});
const DATA = ['joint.pos[0]\timu.rpy.r', '0\t0', '0\t0'].join('\n');
const mp4 = (n: number) => new Uint8Array(n).fill(7);

const mk = (videoTxt: string, clips?: any) =>
  buildSession({ metaTxt: META, dataTxt: DATA, logTxt: '', videoTxt, clips, session: '16_31_30', date: '20260904' });

describe('블랙박스 이벤트 클립(front/rear.mp4)', () => {
  it('clips 를 넘기면 카메라별 buf 로 붙는다', () => {
    const s = mk(VIDEO, { front: mp4(11).buffer, rear: mp4(22).buffer });
    expect(s.video.front?.buf?.byteLength).toBe(11);
    expect(s.video.rear?.buf?.byteLength).toBe(22);
  });

  it('skew = 데이터 프레임0 − 영상 시작 (실측 세션: front -90ms / rear -91ms)', () => {
    const s = mk(VIDEO);
    expect(s.video.front?.skewMs).toBe(-90);
    expect(s.video.rear?.skewMs).toBe(-91);
    expect(s.video.front?.durationMs).toBe(29902);
    expect(s.sync.warn).toBe(false);
  });

  it('인코딩 실패(ok:false)면 클립을 넘겨도 항목이 생기지 않는다', () => {
    const bad = JSON.stringify({
      clip_end_epoch_ms: 1788507090089,
      front: { duration_ms: 0, ok: false, start_epoch_ms: 0 },
      rear:  { duration_ms: 0, ok: false, start_epoch_ms: 0 },
    });
    const s = mk(bad, { front: mp4(11).buffer, rear: mp4(22).buffer });
    expect(s.video.front).toBeUndefined();
    expect(s.video.rear).toBeUndefined();
  });

  it('video.json 이 없으면(구 세션) 영상 없이도 세션은 열린다', () => {
    const s = mk('', { front: mp4(11).buffer });
    expect(s.video.front).toBeUndefined();
    expect(s.frameCount).toBe(2);
  });

  it('zip 임포트 — 세션 폴더 하위의 mp4 까지 꺼낸다', () => {
    const zip = zipSync({
      '20260904/16_31_30/data.log': strToU8(DATA),
      '20260904/16_31_30/meta.json': strToU8(META),
      '20260904/16_31_30/video.json': strToU8(VIDEO),
      '20260904/16_31_30/front.mp4': mp4(33),
      '20260904/16_31_30/rear.mp4': mp4(44),
    });
    const s = parseBlackboxZip(zip);
    expect(s.video.front?.buf?.byteLength).toBe(33);
    expect(s.video.rear?.buf?.byteLength).toBe(44);
    expect(s.video.front?.skewMs).toBe(-90);
  });
});

describe('늦게 도착하는 클립 (save 직후 경합)', () => {
  it('video.json 은 왔는데 mp4 를 못 받으면 pending', () => {
    const s = buildSession({ metaTxt: META, dataTxt: DATA, logTxt: '', videoTxt: VIDEO,
                             clips: {}, clipsAttempted: true, session: 's', date: 'd' });
    expect(s.video.front?.pending).toBe(true);
    expect(s.video.front?.buf).toBeUndefined();
  });

  it('바이트를 받아봤고 받았으면 pending 아님', () => {
    const s = buildSession({ metaTxt: META, dataTxt: DATA, logTxt: '', videoTxt: VIDEO,
                             clips: { front: mp4(9).buffer }, clipsAttempted: true, session: 's', date: 'd' });
    expect(s.video.front?.pending).toBeFalsy();
    expect(s.video.rear?.pending).toBe(true);
  });

  it('애초에 안 받아본 경우(네이티브 URL 스트리밍)는 pending 아님', () => {
    const s = buildSession({ metaTxt: META, dataTxt: DATA, logTxt: '', videoTxt: VIDEO,
                             clipsAttempted: false, session: 's', date: 'd' });
    expect(s.video.front?.pending).toBeFalsy();
  });

  it('video.json 이 아직 없으면 클립 항목 자체가 없다 — 플레이어가 이 상태를 재시도한다', () => {
    const s = buildSession({ metaTxt: META, dataTxt: DATA, logTxt: '', videoTxt: '',
                             clips: {}, clipsAttempted: true, session: 's', date: 'd' });
    expect(s.video.front).toBeUndefined();
    expect(s.video.rear).toBeUndefined();
  });

  it('buildVideo 는 meta 없이는(startEpochMs null) 아무것도 만들지 않는다', () => {
    expect(buildVideo(VIDEO, null, {}, true)).toEqual({});
  });

  it('buildVideo 가 깨진 json 에 안 죽는다', () => {
    expect(buildVideo('{not json', 1788507060092, {}, true)).toEqual({});
  });
});
