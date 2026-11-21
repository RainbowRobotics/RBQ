import type { Category } from './types';

export const CATEGORIES: Category[] = [
  { key: 'primitive', label: '프리미티브', icon: 'palette', load: () => import('./cases/primitives').then((m) => m.primitives) },
  { key: 'form', label: '폼·입력', icon: 'sliders', load: () => import('./cases/forms').then((m) => m.forms) },
  { key: 'overlay', label: '오버레이', icon: 'box', load: () => import('./cases/overlays').then((m) => m.overlays) },
  { key: 'recipe', label: '조립 레시피', icon: 'route', load: () => import('./cases/recipes').then((m) => m.recipes) },
  { key: 'status', label: '상태 표시', icon: 'pulse', load: () => import('./cases/status').then((m) => m.status) },
  { key: 'robot', label: '로봇 조작', icon: 'gamepad', load: () => import('./cases/robot').then((m) => m.robot) },
  { key: 'three', label: '3D', icon: 'box', load: () => import('./cases/three').then((m) => m.three) },
  { key: 'playback', label: '재생기', icon: 'play2', load: () => import('./cases/playback').then((m) => m.playback) },
  { key: 'live', label: '라이브 소스', icon: 'wifi', load: () => import('./cases/live').then((m) => m.live) },
];
