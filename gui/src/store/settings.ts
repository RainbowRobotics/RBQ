import { useEffect, useState } from 'react';
import { create } from 'zustand';
import { persist, createJSONStorage } from 'zustand/middleware';
import { nextKeymap } from '@/lib/accessKeymap';
import AsyncStorage from '@react-native-async-storage/async-storage';
import { connection } from '@/lib/connection';
import { getDemoBaseLevel } from '@/lib/demoFlag';
import { persistedSettingKeys } from '@/modules/registry';
import { sendUserCommand } from '@/lib/userCommand';
import { PROGRAM, QUADWALK_CMD } from '@/lib/robotState';

type SettingsStore = {
  walk: { body_height: number; max_speed: number; foot_height: number };
  setWalk: (k: 'body_height' | 'max_speed' | 'foot_height', v: number) => void;
  commitWalk: () => void;

  bodyTilt: number;
  setBodyTilt: (deg: number) => void;
  commitBodyTilt: () => void;

  sensitivity: number;
  deadzone: number;
  wakeLock: boolean;
  setSensitivity: (v: number) => void;
  setDeadzone: (v: number) => void;
  setWakeLock: (v: boolean) => void;

  accessLevel: 1 | 2 | 3;
  accountLevel: 1 | 2 | 3 | null;
  nickname: string;
  setNickname: (v: string) => void;
  setAccessLevel: (v: 1 | 2 | 3) => void;
  setAccountLevel: (v: 1 | 2 | 3 | null) => void;


  gyroWidgetEnabled: boolean;
  cardsOpen: { left: boolean; right: boolean };
  setCardOpen: (side: 'left' | 'right', open: boolean) => void;
  setGyroWidgetEnabled: (v: boolean) => void;

  robotVersion: string;
  setRobotVersion: (v: string) => void;

  aiMotions: boolean;
  setAiMotions: (v: boolean) => void;
  videoAppEnabled: boolean;
  setVideoAppEnabled: (v: boolean) => void;
  gamepadType: number;
  setGamepadType: (v: number) => void;
  slamVersion: number;
  setSlamVersion: (v: number) => void;
  batteryLowBeep: boolean;
  setBatteryLowBeep: (v: boolean) => void;
  pushAlerts: boolean;
  setPushAlerts: (v: boolean) => void;
  obstacleBeep: boolean;
  setObstacleBeep: (v: boolean) => void;
  walkieEnabled: boolean;
  setWalkieEnabled: (v: boolean) => void;

  visionWalkEnabled: boolean;
  setVisionWalkEnabled: (v: boolean) => void;

  obsAvoidEnabled: boolean;
  setObsAvoidEnabled: (v: boolean) => void;
  obsAvoidMargin: number;
  setObsAvoidMargin: (v: number) => void;
  commitObsAvoidMargin: () => void;
  showObsMap: boolean;
  setShowObsMap: (v: boolean) => void;
  showObsMapCam: boolean;
  setShowObsMapCam: (v: boolean) => void;
  obsAvoidMode: 'avoid' | 'stop';
  setObsAvoidMode: (v: 'avoid' | 'stop') => void;

  animationEnabled: boolean;
  setAnimationEnabled: (v: boolean) => void;

  virtualJoystick: boolean;
  setVirtualJoystick: (v: boolean) => void;

  gpSensitivity: number;
  gpDeadzone: number;
  gpAllButtons: boolean;
  gpOneStick: boolean;
  gpCruise: boolean;
  gpPublishHz: number;
  speedHud: boolean;
  uiZoom: number | null;
  gpUiMode: 'auto' | 'virtual' | 'gamepad';
  setGpSensitivity: (v: number) => void;
  setGpDeadzone: (v: number) => void;
  setGpAllButtons: (v: boolean) => void;
  setGpOneStick: (v: boolean) => void;
  setSpeedHud: (v: boolean) => void;
  setUiZoom: (v: number | null) => void;
  setGpUiMode: (v: 'auto' | 'virtual' | 'gamepad') => void;
  setGpCruise: (v: boolean) => void;
  setGpPublishHz: (v: number) => void;

  audio: { robotSpk: number; robotMic: number; appVol: number };
  setAudio: (k: 'robotSpk' | 'robotMic' | 'appVol', v: number) => void;

  lowConnWarnEnabled: boolean;
  lowConnThreshold: number;
  setLowConnWarnEnabled: (v: boolean) => void;
  setLowConnThreshold: (v: number) => void;

  notifyBeep: boolean;
  setNotifyBeep: (v: boolean) => void;

  gcsMicGain: number;
  setGcsMicGain: (v: number) => void;

  webrtcToken: string;
  setWebrtcToken: (v: string) => void;

  connProfile: 'lo' | 'lan' | 'wan';
  lanIp: string;
  wanIp: string;
  setConnProfile: (v: 'lo' | 'lan' | 'wan') => void;
  setLanIp: (v: string) => void;
  setWanIp: (v: string) => void;

  rendezvousUrl: string;
  robotId: string;
  setRendezvousUrl: (v: string) => void;
  setRobotId: (v: string) => void;

  gcsSpeakerId: string;
  gcsMicId: string;
  setGcsSpeakerId: (v: string) => void;
  setGcsMicId: (v: string) => void;

  panelCollapsed: Record<string, boolean>;
  togglePanelCollapsed: (k: string) => void;
};

export const NEUTRAL_BODY_H = (0.25 / 0.35) * 100;

export const useSettings = create<SettingsStore>()(
  persist(
    (set, get) => ({
  walk: { body_height: NEUTRAL_BODY_H, max_speed: 25, foot_height: 50 },
  setWalk: (k, v) => set((s) => ({ walk: { ...s.walk, [k]: Math.round(v) } })),
  commitWalk: () => connection.putBodyTilt(get().bodyTilt, get().walk),

  bodyTilt: 0,
  setBodyTilt: (deg) => set({ bodyTilt: Math.max(-25, Math.min(25, deg)) }),
  commitBodyTilt: () => connection.putBodyTilt(get().bodyTilt, get().walk),

  sensitivity: 62,
  deadzone: 22,
  wakeLock: true,
  setSensitivity: (v) => set({ sensitivity: Math.round(v) }),
  setDeadzone: (v) => set({ deadzone: Math.round(v) }),
  setWakeLock: (v) => set({ wakeLock: v }),

  accessLevel: 1,
  setAccessLevel: (v) =>
    set((s) => ({ accessLevel: v, gpAllButtons: nextKeymap(s.accessLevel, v, s.gpAllButtons) })),
  accountLevel: null,
  setAccountLevel: (v) => set({ accountLevel: v }),
  nickname: '',
  setNickname: (v) => set({ nickname: v.trim() }),


  cardsOpen: { left: true, right: true },
  setCardOpen: (side, open) => set((s) => ({ cardsOpen: { ...s.cardsOpen, [side]: open } })),
  gyroWidgetEnabled: true,
  setGyroWidgetEnabled: (v) => set({ gyroWidgetEnabled: v }),

  robotVersion: 'none',
  setRobotVersion: (v) => set({ robotVersion: v }),

  aiMotions: false,
  setAiMotions: (v) => set({ aiMotions: v }),
  videoAppEnabled: true,
  setVideoAppEnabled: (v) => set({ videoAppEnabled: v }),
  gamepadType: 1,
  setGamepadType: (v) => set({ gamepadType: v }),
  slamVersion: 0,
  setSlamVersion: (v) => set({ slamVersion: v }),
  batteryLowBeep: false,
  setBatteryLowBeep: (v) => set({ batteryLowBeep: v }),
  pushAlerts: true,
  setPushAlerts: (v) => set({ pushAlerts: v }),
  obstacleBeep: false,
  setObstacleBeep: (v) => set({ obstacleBeep: v }),
  walkieEnabled: true,
  visionWalkEnabled: true,
  setVisionWalkEnabled: (v) => set({ visionWalkEnabled: v }),

  obsAvoidEnabled: false,
  setObsAvoidEnabled: (v) => set({ obsAvoidEnabled: v }),
  obsAvoidMargin: 0.2,
  setObsAvoidMargin: (v) => set({ obsAvoidMargin: Math.max(0.2, Math.min(0.5, v)) }),
  commitObsAvoidMargin: () => sendUserCommand(
    PROGRAM.QuadWalk, QUADWALK_CMD.OBS_AVOID, [], [get().obsAvoidEnabled ? 1 : 0], [get().obsAvoidMargin],
  ),
  showObsMap: false,
  setShowObsMap: (v) => set({ showObsMap: v }),
  showObsMapCam: true,
  setShowObsMapCam: (v) => set({ showObsMapCam: v }),
  obsAvoidMode: 'avoid',
  setObsAvoidMode: (v) => set({ obsAvoidMode: v }),
  animationEnabled: false,
  setAnimationEnabled: (v) => set({ animationEnabled: v }),
  setWalkieEnabled: (v) => set({ walkieEnabled: v }),
  virtualJoystick: true,
  setVirtualJoystick: (v) => set({ virtualJoystick: v }),

  gpSensitivity: 62,
  gpDeadzone: 15,
  gpAllButtons: false,
  gpOneStick: false,
  setGpSensitivity: (v) => set({ gpSensitivity: Math.round(v) }),
  setGpDeadzone: (v) => set({ gpDeadzone: Math.round(v) }),
  setGpAllButtons: (v) => set({ gpAllButtons: v }),
  setGpOneStick: (v) => set({ gpOneStick: v }),
  speedHud: true,
  setSpeedHud: (v) => set({ speedHud: v }),
  uiZoom: null,
  setUiZoom: (v) => set({ uiZoom: v }),
  gpUiMode: 'auto',
  setGpUiMode: (v) => set({ gpUiMode: v }),
  gpCruise: false,
  setGpCruise: (v) => set({ gpCruise: v }),
  gpPublishHz: 40,
  setGpPublishHz: (v) => set({ gpPublishHz: Math.round(v) }),

  audio: { robotSpk: 80, robotMic: 70, appVol: 65 },
  setAudio: (k, v) => set((s) => ({ audio: { ...s.audio, [k]: Math.round(v) } })),

  lowConnWarnEnabled: true,
  lowConnThreshold: 20,
  setLowConnWarnEnabled: (v) => set({ lowConnWarnEnabled: v }),
  setLowConnThreshold: (v) => set({ lowConnThreshold: Math.max(0, Math.min(100, Math.round(v))) }),

  notifyBeep: false,
  setNotifyBeep: (v) => set({ notifyBeep: v }),

  gcsMicGain: 100,
  setGcsMicGain: (v) => set({ gcsMicGain: Math.max(0, Math.min(100, Math.round(v))) }),

  webrtcToken: '',
  setWebrtcToken: (v) => set({ webrtcToken: v }),

  connProfile: 'lan',
  lanIp: '',
  wanIp: '',
  setConnProfile: (v) => set({ connProfile: v }),
  setLanIp: (v) => set({ lanIp: v }),
  setWanIp: (v) => set({ wanIp: v }),

  rendezvousUrl: '',
  robotId: '',
  setRendezvousUrl: (v) => set({ rendezvousUrl: v }),
  setRobotId: (v) => set({ robotId: v }),

  gcsSpeakerId: '',
  gcsMicId: '',
  setGcsSpeakerId: (v) => set({ gcsSpeakerId: v }),
  setGcsMicId: (v) => set({ gcsMicId: v }),

  panelCollapsed: {},
  togglePanelCollapsed: (k) => set((s) => ({ panelCollapsed: { ...s.panelCollapsed, [k]: !s.panelCollapsed[k] } })),
    }),
    {
      name: 'rbq-settings',
      storage: createJSONStorage(() => AsyncStorage),
      version: 1,
      migrate: (persisted: any, ver: number) => {
        if (ver < 1 && persisted?.walk?.body_height === 50) {
          persisted.walk.body_height = NEUTRAL_BODY_H;
        }
        return persisted;
      },
      partialize: (s) => ({
        walk: { ...s.walk, max_speed: 25 }, bodyTilt: s.bodyTilt,
        sensitivity: s.sensitivity, deadzone: s.deadzone, wakeLock: s.wakeLock, cardsOpen: s.cardsOpen,
        accessLevel: getDemoBaseLevel() ?? s.accessLevel,
        accountLevel: s.accountLevel, nickname: s.nickname,
        gyroWidgetEnabled: s.gyroWidgetEnabled,
        aiMotions: s.aiMotions, videoAppEnabled: s.videoAppEnabled, gamepadType: s.gamepadType,
        slamVersion: s.slamVersion, batteryLowBeep: s.batteryLowBeep, obstacleBeep: s.obstacleBeep, pushAlerts: s.pushAlerts,
        walkieEnabled: s.walkieEnabled, visionWalkEnabled: s.visionWalkEnabled,
        obsAvoidEnabled: s.obsAvoidEnabled, obsAvoidMargin: s.obsAvoidMargin, showObsMap: s.showObsMap,
        showObsMapCam: s.showObsMapCam, obsAvoidMode: s.obsAvoidMode,
        animationEnabled: s.animationEnabled,
        virtualJoystick: s.virtualJoystick,
        gpSensitivity: s.gpSensitivity, gpDeadzone: s.gpDeadzone,
        gpAllButtons: s.gpAllButtons, gpOneStick: s.gpOneStick, speedHud: s.speedHud, gpUiMode: s.gpUiMode, gpCruise: s.gpCruise, gpPublishHz: s.gpPublishHz,
        uiZoom: s.uiZoom,
        audio: s.audio,
        lowConnWarnEnabled: s.lowConnWarnEnabled, lowConnThreshold: s.lowConnThreshold,
        notifyBeep: s.notifyBeep, gcsMicGain: s.gcsMicGain,
        webrtcToken: s.webrtcToken,
        connProfile: s.connProfile, lanIp: s.lanIp, wanIp: s.wanIp,
        rendezvousUrl: s.rendezvousUrl, robotId: s.robotId,
        gcsSpeakerId: s.gcsSpeakerId, gcsMicId: s.gcsMicId,
        panelCollapsed: s.panelCollapsed,
        ...Object.fromEntries(persistedSettingKeys.map((k) => [k, (s as unknown as Record<string, unknown>)[k]])),
      }),
    },
  ),
);

export function useModuleSetting<T>(key: string, def: T): T {
  return useSettings((s) => ((s as unknown as Record<string, unknown>)[key] as T | undefined) ?? def);
}
export function setModuleSetting(key: string, v: unknown) {
  useSettings.setState({ [key]: v } as Partial<SettingsStore>);
}

export const useDevMode = () => useSettings((s) => s.accessLevel >= 2);
export const useAccessLevel = () => useSettings((s) => s.accessLevel);

const HYDRATE_TIMEOUT_MS = 3000;

export function useSettingsHydrated() {
  const [hydrated, setHydrated] = useState(useSettings.persist.hasHydrated());
  useEffect(() => {
    if (useSettings.persist.hasHydrated()) { setHydrated(true); return; }
    const off = useSettings.persist.onFinishHydration(() => setHydrated(true));
    const timer = setTimeout(() => setHydrated(true), HYDRATE_TIMEOUT_MS);
    return () => { off(); clearTimeout(timer); };
  }, []);
  return hydrated;
}
