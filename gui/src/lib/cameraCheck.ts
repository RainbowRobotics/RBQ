import { create } from 'zustand';
import { useCameraCheck } from '@/store/robotCheck';
import { t } from '@/lib/i18n';
import { actions, restErrorText, type CameraLive, type CameraLiveView, type DepthCheck, type DepthCheckAction, type UserExtrinsic, type UserExtrinsicAction } from '@/lib/rest';

const POLL_MS = 1000;
const GRACE_MS = 20000;

export { useCameraCheck };

let pollTimer: ReturnType<typeof setTimeout> | null = null;
let starting = false;
let deadline = 0;
const polling = () => useCameraCheck.getState().result?.state === 'running';

function schedulePoll(ip: string) {
  if (pollTimer) clearTimeout(pollTimer);
  pollTimer = setTimeout(async () => {
    pollTimer = null;
    if (!polling()) return;
    if (Date.now() > deadline) {
      const r = useCameraCheck.getState().result!;
      useCameraCheck.setState({ result: { ...r, state: 'failed' }, error: t('로봇이 응답하지 않습니다 — 연결을 확인하세요') });
      return;
    }
    try {
      const r = await actions.cameraCheckGet(ip);
      if (!polling()) return;
      useCameraCheck.setState({ result: r });
      if (r.state === 'running') schedulePoll(ip);
    } catch {
      if (polling()) schedulePoll(ip);
    }
  }, POLL_MS);
}

export async function startCameraCheck(ip: string) {
  if (starting) return;
  starting = true;
  useCameraCheck.setState({ error: '' });
  try {
    const r = await actions.cameraCheckStart(ip);
    deadline = Date.now() + r.duration_s * 1000 + GRACE_MS;
    useCameraCheck.setState({ result: r });
    schedulePoll(ip);
  } catch (e) {
    useCameraCheck.setState({ error: restErrorText(e) });
  } finally {
    starting = false;
  }
}

export function clearCameraCheck() {
  if (useCameraCheck.getState().result?.state === 'running') return;
  if (pollTimer) { clearTimeout(pollTimer); pollTimer = null; }
  useCameraCheck.setState({ result: null, error: '' });
}

export async function stopCameraCheck(ip: string) {
  if (pollTimer) { clearTimeout(pollTimer); pollTimer = null; }
  try { useCameraCheck.setState({ result: await actions.cameraCheckStop(ip) }); } catch { }
}

type CameraLiveUi = { live: CameraLive | null; error: string };
export const useCameraLive = create<CameraLiveUi>(() => ({ live: null, error: '' }));

let liveTimer: ReturnType<typeof setTimeout> | null = null;
let liveOn = false;

function scheduleLive(ip: string) {
  if (liveTimer) clearTimeout(liveTimer);
  liveTimer = setTimeout(async () => {
    liveTimer = null;
    if (!liveOn) return;
    try {
      const live = await actions.cameraLiveGet(ip);
      if (liveOn) useCameraLive.setState({ live, error: '' });
    } catch (e) {
      if (liveOn) useCameraLive.setState({ error: restErrorText(e) });
    }
    if (liveOn) scheduleLive(ip);
  }, POLL_MS);
}

export function startCameraLive(ip: string) {
  liveOn = true;
  useCameraLive.setState({ live: null, error: '' });
  if (liveTimer) clearTimeout(liveTimer);
  actions.cameraLiveGet(ip)
    .then((live) => { if (liveOn) useCameraLive.setState({ live }); })
    .catch((e) => { if (liveOn) useCameraLive.setState({ error: restErrorText(e) }); })
    .finally(() => { if (liveOn) scheduleLive(ip); });
}

export function setCameraLiveViews(ip: string, views: CameraLiveView[], focus: number) {
  actions.cameraLivePut(ip, views, focus)
    .then((live) => { if (liveOn) useCameraLive.setState({ live }); })
    .catch((e) => { if (liveOn) useCameraLive.setState({ error: restErrorText(e) }); });
}

export function stopCameraLive(ip: string) {
  liveOn = false;
  if (liveTimer) { clearTimeout(liveTimer); liveTimer = null; }
  useCameraLive.setState({ live: null, error: '' });
  actions.cameraLiveStop(ip).catch(() => {});
}

type DepthCheckUi = { depth: DepthCheck | null; error: string };
export const useDepthCheck = create<DepthCheckUi>(() => ({ depth: null, error: '' }));

let depthTimer: ReturnType<typeof setTimeout> | null = null;

function scheduleDepth(ip: string) {
  if (depthTimer) clearTimeout(depthTimer);
  depthTimer = setTimeout(async () => {
    depthTimer = null;
    try {
      const depth = await actions.cameraDepthGet(ip);
      useDepthCheck.setState({ depth });
      if (depth.state !== 'idle') scheduleDepth(ip);
    } catch {
      scheduleDepth(ip);
    }
  }, POLL_MS);
}

export async function loadDepthCheck(ip: string) {
  try {
    const depth = await actions.cameraDepthGet(ip);
    useDepthCheck.setState({ depth });
    if (depth.state !== 'idle') scheduleDepth(ip);
  } catch { }
}

export async function startDepthCheck(ip: string, action: DepthCheckAction, sensors?: number[]) {
  useDepthCheck.setState({ error: '' });
  try {
    useDepthCheck.setState({ depth: await actions.cameraDepthPost(ip, action, sensors) });
    scheduleDepth(ip);
  } catch (e) {
    useDepthCheck.setState({ error: restErrorText(e) });
  }
}

type UserExtrinsicUi = { ux: UserExtrinsic | null; error: string };
export const useUserExtrinsic = create<UserExtrinsicUi>(() => ({ ux: null, error: '' }));

let uxTimer: ReturnType<typeof setTimeout> | null = null;

function scheduleUserExtrinsic(ip: string) {
  if (uxTimer) clearTimeout(uxTimer);
  uxTimer = setTimeout(async () => {
    uxTimer = null;
    try {
      const ux = await actions.cameraUserExtrinsicGet(ip);
      useUserExtrinsic.setState({ ux });
      if (ux.state === 'running') scheduleUserExtrinsic(ip);
    } catch {
      scheduleUserExtrinsic(ip);
    }
  }, POLL_MS);
}

export async function loadUserExtrinsic(ip: string) {
  try {
    const ux = await actions.cameraUserExtrinsicGet(ip);
    useUserExtrinsic.setState({ ux });
    if (ux.state === 'running') scheduleUserExtrinsic(ip);
  } catch { }
}

export async function startUserExtrinsic(ip: string, action: UserExtrinsicAction = 'measure') {
  useUserExtrinsic.setState({ error: '' });
  try {
    useUserExtrinsic.setState({ ux: await actions.cameraUserExtrinsicPost(ip, action) });
    scheduleUserExtrinsic(ip);
  } catch (e) {
    useUserExtrinsic.setState({ error: restErrorText(e) });
  }
}
