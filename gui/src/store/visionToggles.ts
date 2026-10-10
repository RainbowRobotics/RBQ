import { create } from 'zustand';
import { actions } from '@/lib/rest';
import { setNightMode } from '@/lib/vision';
import { setStealthMode } from '@/lib/stealth';
import { bindRobotCache } from '@/store/robotSettings';
import { useRobot } from '@/store/robot';

type VisionToggles = {
  night: boolean;
  stealth: boolean;
  guide: boolean;
  faceDetect: boolean;
  hmStairs: boolean;
  hmEdge: boolean;
  resIdx: number;
  dockScan: boolean;
  dockOverride: boolean;
  irProjector: boolean;
  irPendingUntil: number;
  err: string;
  toNight: (ip: string, v: boolean) => void;
  toProjector: (ip: string, v: boolean) => void;
  toStealth: (ip: string, v: boolean) => void;
  toGuide: (ip: string, v: boolean) => void;
  toFace: (ip: string, v: boolean) => void;
  putHeightmap: (ip: string, stairs: boolean, edge: boolean) => void;
  toRes: (ip: string, i: number, w: number, h: number) => void;
  toDockScan: (ip: string, v: boolean) => void;
  setDockOverride: (v: boolean) => void;
};

export const PROJECTOR_PENDING_MS = 4000;

export const useVisionToggles = create<VisionToggles>((set, get) => ({
  night: false, stealth: false, guide: false, faceDetect: false,
  hmStairs: false, hmEdge: false, resIdx: 0, dockScan: true, dockOverride: false, err: '',
  irProjector: false, irPendingUntil: 0,
  toProjector: (ip, v) => {
    set({ err: '', irProjector: v, irPendingUntil: Date.now() + PROJECTOR_PENDING_MS });
    actions.irProjector(ip, v).catch((e: any) => set({ err: String(e?.message ?? e), irProjector: !v, irPendingUntil: 0 }));
  },

  toNight: (ip, v) => {
    set({ err: '', night: v });
    setNightMode(ip, v).catch((e: any) => set({ err: String(e?.message ?? e), night: !v }));
  },
  toStealth: (ip, v) => {
    set({ err: '', stealth: v });
    setStealthMode(ip, v).catch((e: any) => set({ err: String(e?.message ?? e), stealth: !v }));
  },
  toGuide: (ip, v) => {
    set({ err: '', guide: v });
    actions.visionGuide(ip, v).catch((e: any) => set({ err: String(e?.message ?? e), guide: !v }));
  },
  toFace: (ip, v) => {
    set({ err: '', faceDetect: v });
    actions.visionFaceDetect(ip, v).catch((e: any) => set({ err: String(e?.message ?? e), faceDetect: !v }));
  },
  putHeightmap: (ip, stairs, edge) => {
    const prev = { s: get().hmStairs, e: get().hmEdge };
    const eff = stairs ? edge : false;
    set({ err: '', hmStairs: stairs, hmEdge: eff });
    actions.visionHeightmapSettings(ip, stairs, eff)
      .catch((e: any) => set({ err: String(e?.message ?? e), hmStairs: prev.s, hmEdge: prev.e }));
  },
  toRes: (ip, i, w, h) => {
    const prev = get().resIdx;
    set({ err: '', resIdx: i });
    actions.visionWebrtcResolution(ip, w, h).catch((e: any) => set({ err: String(e?.message ?? e), resIdx: prev }));
  },
  toDockScan: (ip, v) => {
    set(v ? { err: '', dockScan: true, dockOverride: false } : { err: '', dockScan: false });
    actions.visionDockScan(ip, v).catch((e: any) => set({ err: String(e?.message ?? e), dockScan: !v }));
  },
  setDockOverride: (v) => { if (get().dockOverride !== v) set({ dockOverride: v }); },
}));

bindRobotCache(useVisionToggles,
  ['night', 'stealth', 'guide', 'faceDetect', 'hmStairs', 'hmEdge', 'resIdx', 'dockScan', 'irProjector'], 'vision');

let wasDocking = false;
useRobot.subscribe((s) => {
  const st = s.robot?.docking_status;
  const now = st != null && st >= 1 && st <= 7;
  if (now && !wasDocking) useVisionToggles.getState().setDockOverride(false);
  wasDocking = now;
});
