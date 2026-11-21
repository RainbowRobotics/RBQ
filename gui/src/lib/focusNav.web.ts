export type NavDir = 'up' | 'down' | 'left' | 'right';
type Box = { x: number; y: number; w: number; h: number };

export function pickNext(cur: Box, cands: Box[], dir: NavDir): number {
  const cx = cur.x + cur.w / 2, cy = cur.y + cur.h / 2;
  let best = -1, bestScore = Infinity;
  cands.forEach((b, i) => {
    const dx = b.x + b.w / 2 - cx, dy = b.y + b.h / 2 - cy;
    const main = dir === 'right' ? dx : dir === 'left' ? -dx : dir === 'down' ? dy : -dy;
    const side = dir === 'left' || dir === 'right' ? Math.abs(dy) : Math.abs(dx);
    if (main <= 1) return;
    const score = main + side * 2;
    if (score < bestScore) { bestScore = score; best = i; }
  });
  return best;
}

const FOCUSABLE = '[tabindex]:not([tabindex="-1"]), button, input, textarea, select, a[href]';

function candidates(): HTMLElement[] {
  const modals = document.querySelectorAll('[aria-modal="true"]');
  const root: ParentNode = modals.length ? modals[modals.length - 1] : document;
  return [...root.querySelectorAll<HTMLElement>(FOCUSABLE)].filter((el) => {
    if (el.getAttribute('aria-disabled') === 'true' || (el as HTMLButtonElement).disabled) return false;
    const r = el.getBoundingClientRect();
    const backdrop = r.width >= innerWidth * 0.9 && r.height >= innerHeight * 0.9;
    return r.width > 0 && r.height > 0 && !backdrop && getComputedStyle(el).visibility !== 'hidden';
  });
}

const box = (el: Element): Box => { const r = el.getBoundingClientRect(); return { x: r.left, y: r.top, w: r.width, h: r.height }; };

function focus(el: HTMLElement) {
  el.focus({ focusVisible: true } as FocusOptions);
  el.scrollIntoView({ block: 'nearest', inline: 'nearest' });
}

export function padNav(cmd: NavDir | 'press') {
  const list = candidates();
  const cur = document.activeElement as HTMLElement | null;
  const on = cur && list.includes(cur) ? cur : null;
  if (cmd === 'press') { if (on?.matches(':focus-visible')) on.click(); return; }
  if (!on) {
    const first = list.map((el) => ({ el, b: box(el) })).sort((a, b) => a.b.y - b.b.y || a.b.x - b.b.x)[0];
    if (first) focus(first.el);
    return;
  }
  const rest = list.filter((el) => el !== on);
  const i = pickNext(box(on), rest.map(box), cmd);
  if (i >= 0) focus(rest[i]);
}
