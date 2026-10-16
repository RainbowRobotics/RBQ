export type OrbitPreset = { label: string; yaw: number; pitch: number; local: boolean };

const HALF_PI = Math.PI / 2;
export const ORBIT_PRESETS: OrbitPreset[] = [
  { label: '⟳', yaw: -0.7, pitch: 0, local: false },
  { label: '앞', yaw: -HALF_PI, pitch: 0, local: true },
  { label: '뒤', yaw: HALF_PI, pitch: 0, local: true },
  { label: '좌', yaw: Math.PI, pitch: 0, local: true },
  { label: '우', yaw: 0, pitch: 0, local: true },
  { label: '위', yaw: 0, pitch: HALF_PI, local: true },
  { label: '아래', yaw: 0, pitch: -HALF_PI, local: true },
];

type Listener = (p: OrbitPreset) => void;
const listeners = new Set<Listener>();

export function applyOrbitPreset(p: OrbitPreset) {
  listeners.forEach((l) => l(p));
}

export function onOrbitPreset(cb: Listener): () => void {
  listeners.add(cb);
  return () => { listeners.delete(cb); };
}
