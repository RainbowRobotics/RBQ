import { describe, it, expect } from 'vitest';
import { mergeToasts, type Toast } from '@/lib/logToast';
import type { LogLine } from '@/types/robot';

const line = (msg: string, level: LogLine['level'] = 'ERROR'): LogLine =>
  ({ ts: '', process: 'Motion', level, msg });

describe('mergeToasts', () => {
  it('새 메시지는 뒤에 쌓인다(최신이 아래)', () => {
    const { next, bumped } = mergeToasts([], [line('a'), line('b')], 0, 3);
    expect(next.map((t) => t.msg)).toEqual(['a', 'b']);
    expect(bumped).toEqual([1, 2]);
  });

  it('반복 메시지는 새로 쌓지 않고 count 만 올린다', () => {
    const first = mergeToasts([], [line('boom')], 0, 3);
    const second = mergeToasts(first.next, [line('boom'), line('boom')], first.lastId, 3);
    expect(second.next).toHaveLength(1);
    expect(second.next[0].count).toBe(3);
    expect(second.bumped).toEqual([1, 1]);
  });

  it('기존 객체를 변형하지 않는다(불변)', () => {
    const prev: Toast[] = [{ id: 1, ts: '', msg: 'x', level: 'ERROR', count: 1 }];
    const { next } = mergeToasts(prev, [line('x')], 1, 3);
    expect(prev[0].count).toBe(1);
    expect(next[0].count).toBe(2);
    expect(next[0]).not.toBe(prev[0]);
  });

  it('최대 개수를 넘으면 오래된 것부터 버리고 dropped 로 알린다', () => {
    const { next, dropped } = mergeToasts([], [line('a'), line('b'), line('c'), line('d')], 0, 3);
    expect(next.map((t) => t.msg)).toEqual(['b', 'c', 'd']);
    expect(dropped).toEqual([1]);
  });

  it('id 는 seed 부터 이어진다(재사용 없음)', () => {
    const first = mergeToasts([], [line('a')], 0, 3);
    const second = mergeToasts(first.next, [line('b')], first.lastId, 3);
    expect(second.next.map((t) => t.id)).toEqual([1, 2]);
    expect(second.lastId).toBe(2);
  });

  it('ts 를 보존한다 — 탭 시 /log 포커스(focusTs)에 쓰인다', () => {
    const { next } = mergeToasts([], [{ ts: '2026-08-26T01:00:00Z', process: 'M', level: 'ERROR', msg: 'x' }], 0, 3);
    expect(next[0].ts).toBe('2026-08-26T01:00:00Z');
  });

  it('FATAL 도 레벨 그대로 보존한다(색 구분용)', () => {
    const { next } = mergeToasts([], [line('dead', 'FATAL')], 0, 3);
    expect(next[0].level).toBe('FATAL');
  });
});
