import { rest } from '@/lib/rest';
import { CAMERA_SOURCES } from '@/lib/visionSources';

const FRAME_MS = 250;
const GRAB_TIMEOUT_MS = 1000;

export function recordViewOf(sourceKey: string): number | undefined {
  if (sourceKey === 'pose3d') return 0;
  return CAMERA_SOURCES.find((s) => s.key === sourceKey)?.streamId;
}

type Grab = () => Promise<string | null>;
let grabber: Grab | null = null;

export function registerFrameGrabber(g: Grab): () => void {
  grabber = g;
  return () => { if (grabber === g) grabber = null; };
}

let timer: ReturnType<typeof setInterval> | null = null;

export function start3dFrames(ip: string) {
  stop3dFrames();
  let busy = false;
  const id = setInterval(() => {
    if (busy || !grabber) return;
    busy = true;
    const g = grabber;
    void (async () => {
      try {
        const b64 = await Promise.race([g(), new Promise<null>((r) => setTimeout(() => r(null), GRAB_TIMEOUT_MS))]);
        if (b64 && timer === id) await rest.videoRecordFrame(ip, b64);
      } catch (e) {
        if (/not recording|unknown command route|40[49]/.test((e as Error)?.message ?? '')) stop3dFrames();
      } finally {
        busy = false;
      }
    })();
  }, FRAME_MS);
  timer = id;
}

export function stop3dFrames() {
  if (timer) clearInterval(timer);
  timer = null;
}
