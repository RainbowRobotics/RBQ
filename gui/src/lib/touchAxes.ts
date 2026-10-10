export type Pad = { x: number; y: number };

export function resolveTouchAxes(
  oneStick: boolean,
  left: Pad,
  right: Pad
): { L: Pad; R: Pad } {
  if (!oneStick) return { L: left, R: right };
  return {
    L: { x: right.x, y: left.y },
    R: { x: left.x, y: right.y },
  };
}
