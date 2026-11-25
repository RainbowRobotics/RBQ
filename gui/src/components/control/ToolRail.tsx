
let railCardW = 92;
export function setRailCardW(w: number) { railCardW = w; }
export function railAnchor(winH: number, side: 'left' | 'right' = 'left', inset = 0) {
  const top = 56 + 12 + 48 + 8;
  const maxHeight = Math.max(160, winH - top - 12);
  const x = 12 + railCardW + 8 + inset;
  return side === 'right' ? { right: x, top, maxHeight } : { left: x, top, maxHeight };
}


let dock = { left: 12, bottom: 70 };
export function setDockAnchor(a: { left: number; bottom: number }) { dock = a; }
export function dockAnchor(winH: number) {
  return { left: dock.left, bottom: dock.bottom, maxHeight: Math.max(160, winH - dock.bottom - 56 - 12) };
}
