import { getPlatformInfo } from '@/lib/platformInfo';
import { apply, type Field, type Ime } from '@/lib/osk/engine';
import { rows, charOf, labelOf, type KeyDef, type Mode } from '@/lib/osk/layout';
import { setOskButtonHandler } from '@/lib/osk/capture';
import { clearGamepadOutputs } from '@/lib/gamepad/manager';
import type { ButtonRole } from '@/lib/gamepad/profiles';

type Editable = HTMLInputElement | HTMLTextAreaElement;

const C = {
  bg: '#15171C', key: '#1F232A', keyHi: '#2A303A', line: '#2C3038',
  text: '#E6E8EB', dim: '#98A0AC', accent: '#4D9CF5',
};
const GAP = 6;
const keyH = () => (typeof window !== 'undefined' && window.innerHeight < 700 ? 40 : 46);
const PAD = 8;

let host: HTMLDivElement | null = null;
let target: Editable | null = null;
let mode: Mode = 'ko';
let letters: 'ko' | 'en' = 'ko';
let shift = false;
let ime: Ime = null;
let sel = 0;
let lifted:
  | { kind: 'transform'; el: HTMLElement; prev: string }
  | { kind: 'scroll'; el: HTMLElement; prevPad: string; prevTop: number }
  | null = null;
let pressing = 0;
let selfBlur = false;
let markPrev = '';
let caret = 0;
let watch: ReturnType<typeof setInterval> | null = null;
let flat: { def: KeyDef; el: HTMLDivElement }[] = [];

const isEditable = (el: EventTarget | null): el is Editable => {
  const e = el as HTMLElement | null;
  return !!e && (e.tagName === 'INPUT' || e.tagName === 'TEXTAREA');
};


function writeValue(el: Editable, next: Field) {
  const proto = el instanceof HTMLTextAreaElement ? HTMLTextAreaElement.prototype : HTMLInputElement.prototype;
  const setter = Object.getOwnPropertyDescriptor(proto, 'value')?.set;
  setter ? setter.call(el, next.value) : (el.value = next.value);
  el.dispatchEvent(new Event('input', { bubbles: true }));
  caret = next.caret;
  try { el.setSelectionRange(next.caret, next.caret); } catch { }
}

function readField(el: Editable): Field {
  return { value: el.value, caret: Math.max(0, Math.min(caret, el.value.length)) };
}

function syncCaretFromDom(el: Editable) {
  const sel = el.selectionStart;
  caret = sel == null ? el.value.length : Math.max(sel, el.selectionEnd ?? sel);
}

function press(def: KeyDef) {
  const el = target;
  if (!el) return;
  switch (def.k) {
    case 'char': {
      const r = apply(readField(el), ime, { type: 'char', char: charOf(def, shift) });
      ime = r.ime; writeValue(el, r.field);
      if (shift && mode !== 'sym') { shift = false; render(); }
      break;
    }
    case 'back':  { const r = apply(readField(el), ime, { type: 'backspace' }); ime = r.ime; writeValue(el, r.field); break; }
    case 'left':  { const r = apply(readField(el), ime, { type: 'left' });  ime = r.ime; writeValue(el, r.field); break; }
    case 'right': { const r = apply(readField(el), ime, { type: 'right' }); ime = r.ime; writeValue(el, r.field); break; }
    case 'space': { const r = apply(readField(el), ime, { type: 'char', char: ' ' }); ime = r.ime; writeValue(el, r.field); break; }
    case 'shift': shift = !shift; render(); break;
    case 'lang':  letters = letters === 'ko' ? 'en' : 'ko'; mode = letters; shift = false; ime = null; render(); break;
    case 'sym':   mode = mode === 'sym' ? letters : 'sym'; shift = false; ime = null; render(); break;
    case 'done': {
      const btn = confirmButton(el);
      close();
      if (btn) { clickEl(btn); break; }
      for (const t of ['keydown', 'keypress', 'keyup']) {
        el.dispatchEvent(new KeyboardEvent(t, { key: 'Enter', code: 'Enter', keyCode: 13, bubbles: true } as KeyboardEventInit));
      }
      break;
    }
  }
}


function keyEl(def: KeyDef): HTMLDivElement {
  const el = document.createElement('div');
  const fn = def.k !== 'char';
  Object.assign(el.style, {
    flex: `${def.w ?? 1} 1 0`, height: `${keyH()}px`,
    display: 'flex', alignItems: 'center', justifyContent: 'center',
    background: def.k === 'shift' && shift ? C.accent : fn ? C.keyHi : C.key,
    color: def.k === 'shift' && shift ? '#fff' : fn ? C.dim : C.text,
    border: `1px solid ${C.line}`, borderRadius: '9px',
    font: `600 ${def.k === 'char' ? 19 : 15}px/1 system-ui, sans-serif`,
    userSelect: 'none', cursor: 'pointer',
  } as CSSStyleDeclaration);
  el.textContent = labelOf(def, mode, shift, letters);
  el.addEventListener('pointerdown', (e) => { e.preventDefault(); e.stopPropagation(); guard(); press(def); flash(el); });
  el.addEventListener('mousedown', (e) => e.preventDefault());
  el.addEventListener('touchstart', (e) => { e.preventDefault(); }, { passive: false });
  return el;
}

function guard() {
  pressing += 1;
  setTimeout(() => { pressing = Math.max(0, pressing - 1); }, 80);
}

function flash(el: HTMLDivElement) {
  const before = el.style.background;
  el.style.background = C.accent;
  setTimeout(() => { el.style.background = before; }, 90);
}

function render() {
  if (!host) return;
  host.innerHTML = '';
  flat = [];
  for (const row of rows(mode, shift)) {
    const r = document.createElement('div');
    Object.assign(r.style, { display: 'flex', gap: `${GAP}px`, marginBottom: `${GAP}px` } as CSSStyleDeclaration);
    for (const def of row) {
      const el = keyEl(def);
      r.appendChild(el);
      flat.push({ def, el });
    }
    host.appendChild(r);
  }
  paintSelection();
  lift();
}

function paintSelection() {
  flat.forEach(({ el }, i) => {
    const on = i === sel;
    el.style.outline = on ? `2px solid ${C.accent}` : 'none';
    el.style.outlineOffset = '1px';
  });
}


function cardOf(input: Editable): HTMLElement {
  let card: HTMLElement = input, node: HTMLElement = input;
  while (node.parentElement && node.parentElement !== document.body) {
    node = node.parentElement;
    const r = node.getBoundingClientRect();
    if (r.height > 0 && r.height <= window.innerHeight * 0.85) card = node;
  }
  return card;
}

function liftAmount(input: Editable, kbTop: number): number {
  const margin = 10;
  const card = cardOf(input);
  const need = (r: DOMRect) => r.bottom - (kbTop - margin);
  const forInput = Math.max(0, need(input.getBoundingClientRect()));
  const c = card.getBoundingClientRect();
  const forCard = need(c);
  return forCard > forInput && c.top - forCard >= 8 ? forCard : forInput;
}

function lift() {
  unlift();
  const el = target, kb = host;
  if (!el || !kb) return;
  let top: HTMLElement = el;
  while (top.parentElement && top.parentElement !== document.body) top = top.parentElement;
  if (top === kb) return;
  const kbTop = kb.getBoundingClientRect().top;
  const need = liftAmount(el, kbTop);
  if (need <= 0) return;

  if (top.id === 'root') {
    const sc = scrollParent(el);
    if (!sc) return;
    lifted = { kind: 'scroll', el: sc, prevPad: sc.style.paddingBottom, prevTop: sc.scrollTop };
    sc.style.paddingBottom = `${Math.ceil(window.innerHeight - kbTop) + 16}px`;
    sc.scrollTop += Math.ceil(need);
    return;
  }
  lifted = { kind: 'transform', el: top, prev: top.style.transform };
  top.style.transition = 'transform 140ms ease';
  top.style.transform = `translateY(${-Math.ceil(need)}px)`;
}

function scrollParent(el: HTMLElement): HTMLElement | null {
  for (let n = el.parentElement; n && n !== document.body; n = n.parentElement) {
    const oy = getComputedStyle(n).overflowY;
    if (oy === 'auto' || oy === 'scroll') return n;
  }
  return null;
}

function unlift() {
  if (!lifted) return;
  if (lifted.kind === 'transform') {
    lifted.el.style.transform = lifted.prev;
  } else {
    lifted.el.style.paddingBottom = lifted.prevPad;
    lifted.el.scrollTop = lifted.prevTop;
  }
  lifted = null;
}


const CONFIRM_WORDS = ['확인', '저장', '연결', '추가', '적용', '등록', '완료', '전송',
                       'ok', 'save', 'connect', 'add', 'apply', 'done'];

function confirmButton(input: Editable): HTMLElement | null {
  for (const el of cardOf(input).querySelectorAll<HTMLElement>('[tabindex], button')) {
    const t = (el.textContent ?? '').trim().toLowerCase();
    if (t && t.length <= 12 && CONFIRM_WORDS.includes(t)) return el;
  }
  return null;
}

function clickEl(el: HTMLElement) {
  for (const type of ['pointerdown', 'mousedown', 'pointerup', 'mouseup', 'click']) {
    const E = type.startsWith('pointer') ? PointerEvent : MouseEvent;
    el.dispatchEvent(new E(type, { bubbles: true, cancelable: true }));
  }
}


function open(el: Editable) {
  const prevTarget = target;
  target = el;
  ime = null;
  sel = -1;
  syncCaretFromDom(el);
  if (!host) {
    host = document.createElement('div');
    Object.assign(host.style, {
      position: 'fixed', left: '0', right: '0', bottom: '0', zIndex: '10000',
      padding: `${PAD}px`, paddingBottom: `${PAD + 2}px`,
      background: C.bg, borderTop: `1px solid ${C.line}`,
      boxShadow: '0 -8px 24px rgba(0,0,0,0.45)',
      touchAction: 'none',
    } as CSSStyleDeclaration);
    host.addEventListener('pointerdown', (e) => { e.preventDefault(); guard(); });
    host.addEventListener('mousedown', (e) => e.preventDefault());
    host.addEventListener('touchstart', (e) => { e.preventDefault(); guard(); }, { passive: false });
    document.body.appendChild(host);
  }
  host.style.display = 'block';
  setOskButtonHandler(onPad);
  clearGamepadOutputs();
  if (watch) clearInterval(watch);
  watch = setInterval(() => {
    if (!target) return;
    if (!target.isConnected || target.offsetParent === null) close();
  }, 400);
  if (prevTarget) prevTarget.style.outline = markPrev;
  markPrev = el.style.outline;
  el.style.outline = `2px solid ${C.accent}`;
  selfBlur = true;
  el.blur();
  setTimeout(() => { selfBlur = false; }, 0);
  render();
}

function close() {
  const el = target;
  if (watch) { clearInterval(watch); watch = null; }
  if (el) el.style.outline = markPrev;
  ime = null;
  target = null;
  pressing = 0;
  setOskButtonHandler(null);
  unlift();
  if (host) host.style.display = 'none';
  el?.blur();
}


function grid(): number[][] {
  const out: number[][] = [];
  let i = 0;
  for (const row of rows(mode, shift)) {
    out.push(row.map(() => i++));
  }
  return out;
}

function move(dx: number, dy: number) {
  const g = grid();
  if (sel < 0) { sel = g[0][0]; paintSelection(); return; }
  let r = g.findIndex((row) => row.includes(sel));
  let c = g[r].indexOf(sel);
  if (dy) {
    const ratio = c / Math.max(1, g[r].length - 1);
    r = (r + dy + g.length) % g.length;
    c = Math.round(ratio * (g[r].length - 1));
  }
  if (dx) c = (c + dx + g[r].length) % g[r].length;
  sel = g[r][c];
  paintSelection();
}

function onPad(role: ButtonRole) {
  switch (role) {
    case 'DPAD_L': move(-1, 0); break;
    case 'DPAD_R': move(1, 0); break;
    case 'DPAD_U': move(0, -1); break;
    case 'DPAD_D': move(0, 1); break;
    case 'A': {
      const hit = flat[sel];
      if (hit) { press(hit.def); flash(hit.el); }
      break;
    }
    case 'B': { const el = target; close(); el?.blur(); break; }
    case 'X': press({ k: 'back' }); break;
    case 'Y': press({ k: 'space' }); break;
    case 'L1': press({ k: 'shift' }); break;
    case 'R1': press({ k: 'lang' }); break;
    default: break;
  }
}


export function installOsk() {
  if (typeof document === 'undefined') return;
  const forced = typeof location !== 'undefined' && /[?&]osk=1/.test(location.search);
  const gate = forced ? Promise.resolve(true) : getPlatformInfo().then((p) => p.steamos);
  gate.then((on) => {
    if (!on) return;
    document.addEventListener('focusin', (e) => { if (isEditable(e.target)) open(e.target); });
    document.addEventListener('focusout', (e) => {
      if (selfBlur) return;
      if (!isEditable(e.target) || e.target !== target) return;
      setTimeout(() => {
        const a = document.activeElement;
        if (isEditable(a)) return;
        if ((pressing > 0 || (host && a && host.contains(a))) && target) {
          const el = target;
          el.focus();
          try { el.setSelectionRange(caret, caret); } catch { }
          return;
        }
        close();
      }, 0);
    });
    document.addEventListener('pointerdown', (e) => {
      if (!isEditable(e.target)) return;
      ime = null;
      const el = e.target;
      setTimeout(() => { if (el === target) syncCaretFromDom(el); }, 0);
    });
    document.addEventListener('pointerdown', (e) => {
      const n = e.target as Node | null;
      if (!target || (host && n && host.contains(n)) || isEditable(n)) return;
      close();
    }, true);
    window.addEventListener('resize', () => { if (target) lift(); });
  });
}
