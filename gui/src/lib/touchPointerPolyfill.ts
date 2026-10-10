import { isDesktop } from '@/lib/desktopBridge';
import { getPlatformInfo } from '@/lib/platformInfo';

export function installTouchPointerPolyfill() {
  if (typeof document === 'undefined' || typeof PointerEvent === 'undefined') return;
  if (!isDesktop() || !/Linux|X11/.test(navigator.userAgent)) return;

  for (const m of ['setPointerCapture', 'releasePointerCapture'] as const) {
    const orig = (Element.prototype as any)[m];
    (Element.prototype as any)[m] = function (this: Element, id: number) {
      try { return orig.call(this, id); } catch { }
    };
  }

  type Entry = { target: EventTarget; prevent: boolean; at: number };
  const active = new Map<number, Entry>();

  const wantsNoTouchAction = (el: Element | null): boolean => {
    for (let n = el; n; n = n.parentElement) {
      try { if (getComputedStyle(n).touchAction === 'none') return true; } catch { return false; }
    }
    return false;
  };

  const fire = (type: string, t: Touch, target: EventTarget) => {
    target.dispatchEvent(new PointerEvent(type, {
      bubbles: true, cancelable: true, composed: true,
      pointerId: (t.identifier || 0) + 2,
      pointerType: 'touch', isPrimary: active.size <= 1,
      clientX: t.clientX, clientY: t.clientY, screenX: t.screenX, screenY: t.screenY,
      buttons: type === 'pointerup' || type === 'pointercancel' ? 0 : 1,
      pressure: type === 'pointerup' || type === 'pointercancel' ? 0 : 0.5,
    }));
  };

  document.addEventListener('touchstart', (e) => {
    const now = Date.now();
    for (const [id, a] of active) if (now - a.at > 15000) active.delete(id);
    for (const t of Array.from(e.changedTouches)) {
      const prevent = wantsNoTouchAction(t.target as Element);
      active.set(t.identifier, { target: t.target, prevent, at: now });
      const tgtEl = t.target as Element | null;
      const ed = (tgtEl?.closest?.('input, textarea, select, [contenteditable]')
        ?? tgtEl?.querySelector?.('input, textarea, select, [contenteditable]')) as HTMLElement | null;
      if (!ed) {
        const ae = document.activeElement as HTMLElement | null;
        if (ae && ae !== document.body && ae.matches?.('input, textarea, select, [contenteditable]')) {
          try { ae.blur(); } catch { }
        }
      }
      if ((prevent || ed || active.size >= 2) && e.cancelable) e.preventDefault();
      fire('pointerdown', t, t.target);
      if (ed) {
        try { ed.focus({ preventScroll: true }); } catch { }
      }
    }
  }, { capture: true, passive: false });

  document.addEventListener('touchmove', (e) => {
    const multi = e.touches.length >= 2;
    for (const t of Array.from(e.changedTouches)) {
      const a = active.get(t.identifier);
      if (!a) continue;
      if ((multi || a.prevent) && e.cancelable) e.preventDefault();
      fire('pointermove', t, a.target);
    }
  }, { capture: true, passive: false });

  const end = (type: 'pointerup' | 'pointercancel') => (e: TouchEvent) => {
    for (const t of Array.from(e.changedTouches)) {
      const a = active.get(t.identifier);
      if (!a) continue;
      fire(type, t, a.target);
      active.delete(t.identifier);
    }
  };
  document.addEventListener('touchend', end('pointerup'), { capture: true });
  document.addEventListener('touchcancel', end('pointercancel'), { capture: true });

}

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
