import type { WebGLRenderer } from 'three';

const MAX_W = 960;
let scratch: HTMLCanvasElement | null = null;

export function grabGl(gl: WebGLRenderer): Promise<string | null> {
  const src = gl.domElement;
  if (!src?.width || !src.height) return Promise.resolve(null);
  const scale = Math.min(1, MAX_W / src.width);
  scratch ??= document.createElement('canvas');
  scratch.width = Math.max(2, Math.round(src.width * scale) & ~1);
  scratch.height = Math.max(2, Math.round(src.height * scale) & ~1);
  const ctx = scratch.getContext('2d');
  if (!ctx) return Promise.resolve(null);
  ctx.drawImage(src, 0, 0, scratch.width, scratch.height);
  const url = scratch.toDataURL('image/jpeg', 0.7);
  const i = url.indexOf(',');
  return Promise.resolve(i > 0 ? url.slice(i + 1) : null);
}
