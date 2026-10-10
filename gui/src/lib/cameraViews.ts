import { webrtcClient } from '@/lib/webrtcClient';

export const NO_VIEW = 0;
let holds = 0;
let releaseTimer: ReturnType<typeof setTimeout> | null = null;

export function holdCameraView() {
  holds += 1;
  if (releaseTimer) { clearTimeout(releaseTimer); releaseTimer = null; }
}

export function releaseCameraView() {
  holds = Math.max(0, holds - 1);
  if (holds > 0 || releaseTimer) return;
  releaseTimer = setTimeout(() => { releaseTimer = null; if (holds === 0) webrtcClient.setSource(NO_VIEW); }, 400);
}
