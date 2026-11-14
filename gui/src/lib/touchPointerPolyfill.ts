import { isDesktop } from '@/lib/desktopBridge';
import { getPlatformInfo } from '@/lib/platformInfo';

export function installMouseDragScrollShim() {
  if (typeof document === 'undefined') return;
  if (!isDesktop() || !/Linux|X11/.test(navigator.userAgent)) return;
  getPlatformInfo().then((pf) => {
    if (!pf.steamos) return;

  const findScrollable = (el: Element | null): Element | null => {
    for (let n = el; n && n !== document.body; n = n.parentElement) {
      const st = getComputedStyle(n);
      const oy = st.overflowY;
      if ((oy === 'auto' || oy === 'scroll') && n.scrollHeight > n.clientHeight + 1) return n;
      const ox = st.overflowX;
      if ((ox === 'auto' || ox === 'scroll') && n.scrollWidth > n.clientWidth + 1) return n;
    }
    return null;
  };
  const isNoScrollZone = (el: Element | null): boolean => {
    for (let n = el; n && n !== document.body; n = n.parentElement) {
      if (getComputedStyle(n).touchAction === 'none') return true;
    }
    return false;
  };

  let start: { x: number; y: number; target: Element | null } | null = null;
  let scrollable: Element | null = null;
  let dragging = false;
  let scrolled = false;
  let lastX = 0; let lastY = 0;

  document.addEventListener('pointerdown', (e) => {
    if (e.pointerType !== 'mouse' || e.button !== 0) return;
    const t = e.target as Element | null;
    if (isNoScrollZone(t)) { start = null; return; }
    start = { x: e.clientX, y: e.clientY, target: t };
    scrollable = findScrollable(t);
    dragging = false; scrolled = false; lastX = e.clientX; lastY = e.clientY;
  }, true);

  document.addEventListener('pointermove', (e) => {
    if (!start || e.pointerType !== 'mouse' || !(e.buttons & 1)) return;
    const dx = e.clientX - lastX; const dy = e.clientY - lastY;
    if (!dragging && Math.hypot(e.clientX - start.x, e.clientY - start.y) > 12) {
      dragging = true;
      const ae = document.activeElement as HTMLElement | null;
      if (ae && (ae.tagName === 'INPUT' || ae.tagName === 'TEXTAREA')) ae.blur();
    }
    if (dragging && scrollable) {
      const pt = scrollable.scrollTop; const pl = scrollable.scrollLeft;
      scrollable.scrollTop -= dy;
      scrollable.scrollLeft -= dx;
      if (scrollable.scrollTop !== pt || scrollable.scrollLeft !== pl) scrolled = true;
      e.preventDefault(); e.stopPropagation();
    }
    lastX = e.clientX; lastY = e.clientY;
  }, true);

  const end = () => {
    if (scrolled) {
      const absorb = (ce: Event) => { ce.stopPropagation(); ce.preventDefault(); };
      document.addEventListener('click', absorb, { capture: true, once: true });
      setTimeout(() => document.removeEventListener('click', absorb, { capture: true } as any), 150);
    }
    start = null; scrollable = null; dragging = false; scrolled = false;
  };
  document.addEventListener('pointerup', end, true);
  document.addEventListener('pointercancel', end, true);
  });
}


export function installDeckCursorHide() {
  if (typeof document === 'undefined') return;
  if (!isDesktop() || !/Linux|X11/.test(navigator.userAgent)) return;
  getPlatformInfo().then((p) => {
    if (!p.steamos) return;
    const s = document.createElement('style');
    s.textContent = '*{cursor:none !important} body{-webkit-user-select:none;user-select:none} input,textarea{-webkit-user-select:text;user-select:text}';
    document.head.appendChild(s);
  });
}
