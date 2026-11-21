export type StickGate = {
  begin(): void;
  move(x: number, y: number): void;
  release(): void;
  mount(): void;
  unmount(): void;
};

export function createStickGate(out: (x: number, y: number) => void): StickGate {
  let touching = false;
  let alive = true;
  return {
    begin() { if (alive) touching = true; },
    move(x, y) { if (alive && touching) out(x, y); },
    release() {
      touching = false;
      if (alive) out(0, 0);
    },
    mount() { alive = true; },
    unmount() {
      const was = touching;
      touching = false;
      alive = false;
      if (was) out(0, 0);
    },
  };
}
