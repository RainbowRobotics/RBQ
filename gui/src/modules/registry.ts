import type { ComponentType } from 'react';
import type { IconName } from '@/components/Icon';
import type { RobotFeatures } from '@/lib/rest';
import type { ButtonRole } from '@/lib/gamepad/profiles';
import type { SlamState } from '@/lib/slamState';
import type { useTelemetry } from '@/store/telemetry';
import { EN } from '@/lib/i18n.en';

type TelemetryState = ReturnType<typeof useTelemetry.getState>;

export function addTranslations(map: Record<string, string>) {
  Object.assign(EN, map);
}

export const persistedSettingKeys: string[] = [];

export type MaintenanceTab = {
  key: string; label: string; icon: IconName;
  visible: (s: TelemetryState) => boolean;
  Panel: ComponentType;
};
export const maintenanceTabs: MaintenanceTab[] = [];

export type DashboardRow = { key: string; label: string; useVisible: () => boolean; useOk: () => boolean };
export const dashboardRows: DashboardRow[] = [];

export type DashboardCardData = { title: string; live: boolean; rows: [string, string][]; empty: string };
export type DashboardCard = { key: string; icon: IconName; use: () => DashboardCardData | null };
export const dashboardCards: DashboardCard[] = [];

export const gamepadCombos: ((side: 'L' | 'R') => void)[] = [];

export type MotionFrameHandler = { size: number; onFrame: (buf: ArrayBuffer, offset: number) => void; clear?: () => void };
export const motionFrames = new Map<number, MotionFrameHandler>();

export type RobotVersionDef = {
  key: string; label: string;
  feature?: string;
  hideDock?: boolean;
  slamWithoutLidar?: boolean;
};
export const robotVersions: RobotVersionDef[] = [];
export const robotVersionDef = (key: string) => robotVersions.find((v) => v.key === key);
export const autoRobotVersion = (f: RobotFeatures) => robotVersions.find((v) => v.feature && f[v.feature]);

export type PayloadSlotFilter = { feature: string; keep: (slotId: number) => boolean };
export const payloadSlotFilters: PayloadSlotFilter[] = [];

export type SlamRow = { label: string; onPress?: () => void; active?: boolean; disabled?: boolean; note?: string };
export type SlamGroup = { title: string; rows: SlamRow[]; minLevel?: 1 | 2 | 3 };
export type SlamVariant = {
  key: string; label: string; slamVersion: number;
  estop?: boolean;
  groups: (ctx: { st?: SlamState; hlc: (on: boolean) => void; view: SlamGroup }) => SlamGroup[];
};
export const slam = { defaultLabel: 'Default', variants: [] as SlamVariant[] };

export type RobotKind = {
  key: string;
  channel: string;
  onMessage: (raw: string) => void;
  reset: () => void;
  isActive: () => boolean;
  useActive: () => boolean;
  route: string;
  HubModel?: ComponentType<{ fill?: boolean }>;
  frameIndex: Partial<Record<ButtonRole, number>>;
  estop: { label: string; title: string; body: string[]; send: () => boolean };
};
export const robotKinds: RobotKind[] = [];
export const activeRobotKind = () => robotKinds.find((k) => k.isActive());
export function useRobotKind(): RobotKind | undefined {
  let found: RobotKind | undefined;
  for (const k of robotKinds) if (k.useActive() && !found) found = k;
  return found;
}
