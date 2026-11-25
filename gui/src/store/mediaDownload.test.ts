import { describe, it, expect, beforeEach, vi } from 'vitest';
import { useMediaDl, mediaDownload, mediaLine, mediaJob, type MediaItem, type MediaSaver } from './mediaDownload';
import type { MediaKind } from '@/lib/rest';

const S = () => useMediaDl.getState();
const item = (path: string, kind: MediaKind = 'clip'): MediaItem => ({ ip: '192.168.0.10', kind, file: { path, size: 1 << 20, mtime: 0 } });
const flush = () => new Promise((r) => setTimeout(r, 0));
const saver = (fail?: string): MediaSaver => vi.fn(async (_ip, _kind, path, _size, canShare) => {
  if (path === fail) throw new Error('로봇 응답 HTTP 404 — 파일을 받지 못했습니다');
  const uri = `file:///cache/media-${path}`;
  canShare(uri);
  return uri;
});

describe('미디어 받기 — 미디어 화면을 벗어나도 묶음 끝까지', () => {
  beforeEach(() => useMediaDl.setState({ open: 0, unseen: false, queue: [], busy: null, sec: 'photo', err: null, got: 0, share: [] }));

  it('녹화 묶음은 화면 밖에서도 다음 파일까지 받고, 공유 시트는 미뤄 둔다', async () => {
    const save = saver();
    mediaDownload([item('front_1.mp4'), item('rear_1.mp4')], save);
    expect(S().busy).toBe('front_1.mp4');
    expect(mediaLine(S())).toMatchObject({ tone: 'run', closable: false });
    expect(mediaLine(S())!.text).toContain('front_1.mp4');
    await flush();
    expect(save).toHaveBeenCalledTimes(2);
    expect(S().busy).toBeNull();
    expect(S().share).toEqual(['file:///cache/media-front_1.mp4', 'file:///cache/media-rear_1.mp4']);
    expect(S().unseen).toBe(true);
    expect(mediaLine(S())).toMatchObject({ tone: 'ok', closable: true });
    expect(mediaLine(S())!.text).toContain('눌러서 공유');
  });

  it('화면이 떠 있으면 공유 시트를 바로 띄운다(미루지 않는다), 배너도 없다', async () => {
    useMediaDl.setState({ open: 1 });
    mediaDownload([item('a.jpg', 'photo')], saver());
    expect(mediaLine(S())).toBeNull();
    await flush();
    expect(S().share).toEqual([]);
    expect(S().unseen).toBe(false);
  });

  it('하나가 실패해도 나머지는 받는다 — 실패는 그 종류 화면에 남고 배너가 먼저 알린다', async () => {
    const save = saver('front_1.mp4');
    mediaDownload([item('front_1.mp4'), item('rear_1.mp4')], save);
    await flush();
    expect(save).toHaveBeenCalledTimes(2);
    expect(S().got).toBe(1);
    expect(S().err).toMatchObject({ kind: 'clip' });
    expect(mediaLine(S())).toMatchObject({ tone: 'bad' });
    expect(mediaLine(S())!.text).toContain('404');
  });

  it('받는 중에 같은 파일을 또 누르면 줄에 두 번 넣지 않는다 — 다른 파일은 뒤에 붙는다', async () => {
    const save = saver();
    mediaDownload([item('a.wav', 'audio')], save);
    mediaDownload([item('a.wav', 'audio'), item('b.wav', 'audio')], save);
    mediaDownload([item('a.wav', 'clip')], save);
    expect(S().queue.map((q) => `${q.kind}:${q.file.path}`)).toEqual(['audio:b.wav', 'clip:a.wav']);
    await flush();
    expect(save).toHaveBeenCalledTimes(3);
  });

  it('배너 X 는 결과를 본 것 — 실패 문구·미룬 공유를 지운다', () => {
    useMediaDl.setState({ unseen: true, got: 2, err: { kind: 'clip', text: 'HTTP 404' }, share: ['file:///cache/media-a.mp4'] });
    mediaJob.dismiss();
    expect(S()).toMatchObject({ unseen: false, got: 0, err: null, share: [] });
  });

  it('배너를 누르면 받던 종류의 화면으로', () => {
    mediaDownload([item('a.wav', 'audio')], saver());
    expect(mediaJob.route).toBe('/media?sec=audio');
  });
});
