import { describe, it, expect } from 'vitest';
import { createStickGate } from './stickGate';

function rig() {
  const out: [number, number][] = [];
  const g = createStickGate((x, y) => out.push([x, y]));
  g.mount();
  return { g, out, last: () => out[out.length - 1] };
}

describe('가상 조이스틱 관문 — 재밍 방지', () => {
  it('정상 조종: 누르고 움직이면 그대로 나가고, 떼면 0', () => {
    const { g, out, last } = rig();
    g.begin(); g.move(0.5, 0.2); g.move(0.6, 0.3);
    expect(out).toEqual([[0.5, 0.2], [0.6, 0.3]]);
    g.release();
    expect(last()).toEqual([0, 0]);
  });

  it('뗀 뒤 늦게 도착한 이동값은 버린다 — 0 을 덮어쓰지 않는다', () => {
    const { g, last } = rig();
    g.begin(); g.move(0.8, 0);
    g.release();
    g.move(0.8, 0);
    expect(last()).toEqual([0, 0]);
  });

  it('잡힌 채 부품이 사라지면 0 을 보내고, 그 뒤 도착하는 값은 전부 버린다', () => {
    const { g, out, last } = rig();
    g.begin(); g.move(0.7, 0.7);
    g.unmount();
    expect(last()).toEqual([0, 0]);
    const n = out.length;
    g.move(0.7, 0.7);
    g.begin(); g.move(0.3, 0);
    expect(out.length).toBe(n);
  });

  it('잡혀 있지 않았으면 사라질 때 아무것도 보내지 않는다(다른 스틱 값을 지우지 않게)', () => {
    const { g, out } = rig();
    g.unmount();
    expect(out).toEqual([]);
  });

  it('다시 누르면 곧바로 움직인다 — 떼었다 다시 잡아도 막히지 않는다', () => {
    const { g, last } = rig();
    g.begin(); g.move(0.4, 0); g.release();
    g.begin(); g.move(-0.4, 0.1);
    expect(last()).toEqual([-0.4, 0.1]);
  });

  it('begin 이 여러 번 와도(onBegin·onStart) 같다', () => {
    const { g, out } = rig();
    g.begin(); g.begin(); g.move(0.2, 0.2);
    expect(out).toEqual([[0.2, 0.2]]);
  });

  it('개발 모드처럼 붙였다 뗐다 다시 붙여도 조이스틱이 살아 있다', () => {
    const { g, last } = rig();
    g.unmount(); g.mount();
    g.begin(); g.move(0.9, 0);
    expect(last()).toEqual([0.9, 0]);
  });
});
