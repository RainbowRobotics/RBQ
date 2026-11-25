import type { WebGLRenderer } from 'three';
import { GLView, type ExpoWebGLRenderingContext } from 'expo-gl';
import { File } from 'expo-file-system';

export async function grabGl(gl: WebGLRenderer): Promise<string | null> {
  const ctx = gl.getContext() as unknown as ExpoWebGLRenderingContext;
  const snap = await GLView.takeSnapshotAsync(ctx, { format: 'jpeg', compress: 0.6 });
  if (typeof snap.uri !== 'string') return null;
  const f = new File(snap.uri);
  try {
    return await f.base64();
  } finally {
    try { f.delete(); } catch { }
  }
}
