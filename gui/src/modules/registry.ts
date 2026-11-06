import type { ComponentType, ReactNode } from 'react';
import type { IconName } from '@/components/Icon';
import type { RobotFeatures } from '@/lib/rest';
import type { ButtonRole } from '@/lib/gamepad/profiles';
import type { CloudSet, MapCloud, SlamnavState, SlamUiState } from '@/lib/slamnavWire';
import type { useTelemetry } from '@/store/telemetry';
import type { RobotProfile } from '@/store/robots';
import type { Target } from '@/lib/connectTarget';
import type { IceServer } from '@/lib/remoteIce';
import type { IniSource } from '@/lib/iniSetting';
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
  lv?: 2 | 3;
  ownScroll?: boolean;
};
export const maintenanceTabs: MaintenanceTab[] = [];

export type DashboardRow = { key: string; label: string; useVisible: () => boolean; useOk: () => boolean };
export const dashboardRows: DashboardRow[] = [];

export type DashboardCardData = { title: string; live: boolean; rows: [string, string][]; empty: string };
export type DashboardCard = { key: string; icon: IconName; useData: () => DashboardCardData | null };
export const dashboardCards: DashboardCard[] = [];

export type ViewRowDef = { key: string; icon: IconName; label: string; useVisible: () => boolean; useActive: () => boolean; onPress: () => void };
export const viewRows: ViewRowDef[] = [];

export const accessCodes: { level: 2 | 3 | 4; hash: string }[] = [];

export const staffTools = { useOn: (): boolean => false };

export const iniSources = { useExtra: (_ip: string): IniSource[] => [] };

export const topBarButtons: ComponentType[] = [];

export const appSettingsRows: ComponentType[] = [];
export const appNotifyRows: ComponentType[] = [];

export type LogActionProps = { tab: 'rt' | 'sys' | 'bb'; ip: string; sysDate: string; bbDate: string; bbSession: string; fromLibrary: boolean; srcBytes: number };
export const logActions: ComponentType<LogActionProps>[] = [];

export const accountSections: ComponentType[] = [];

export type MotionTileDef = { icon: IconName; label: string; motion: string };
export type MotionGroupDef = { key: string; title: string; tiles: MotionTileDef[]; useVisible: () => boolean };
export const motionGroups: MotionGroupDef[] = [];

export type ControlToolDef = { key: string; icon: IconName; label: string; useVisible: () => boolean; Panel: ComponentType<{ onClose: () => void }> };
export const controlTools: ControlToolDef[] = [];
export function useVisibleControlTools(): ControlToolDef[] {
  const out: ControlToolDef[] = [];
  for (const m of controlTools) if (m.useVisible()) out.push(m);
  return out;
}

export const positionTools: ControlToolDef[] = [];
export function useVisiblePositionTools(): ControlToolDef[] {
  const out: ControlToolDef[] = [];
  for (const m of positionTools) if (m.useVisible()) out.push(m);
  return out;
}

export const viewportOverlays: ComponentType[] = [];

export const hubButtons: ComponentType[] = [];
export const standalonePaths: string[] = [];

export const appBanners: ComponentType[] = [];
export const appRoots: ComponentType[] = [];

export const robotCheckSections: ComponentType[] = [];

export type RobotSheetActionDef = {
  key: string; icon: IconName; label: string; sub: string; useVisible: () => boolean;
  Dialog: ComponentType<{ onClose: () => void; onConnected: () => void }>;
};
export const robotSheetActions: RobotSheetActionDef[] = [];
export function useVisibleRobotSheetActions(): RobotSheetActionDef[] {
  const out: RobotSheetActionDef[] = [];
  for (const a of robotSheetActions) if (a.useVisible()) out.push(a);
  return out;
}

export type RobotRowActionDef = {
  key: string; label: string;
  useShow: () => (serial: string, current: boolean) => boolean;
  Dialog: ComponentType<{ serial: string; onClose: () => void }>;
};
export const robotRowActions: RobotRowActionDef[] = [];

export type AddRobotCardProps = { card: (icon: IconName, title: string, sub: string, body: ReactNode) => ReactNode };
export const addRobotCards: ComponentType<AddRobotCardProps>[] = [];

export type AccountMenuProps = { onClose: () => void; beforeNavigate?: (next: () => void) => void };
export const accountGuestItems: ComponentType<AccountMenuProps>[] = [];

export const connRejectReasons: (() => string | undefined)[] = [];
export const connRejectReason = (): string | undefined => {
  for (const f of connRejectReasons) { const r = f(); if (r) return r; }
  return undefined;
};

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
export type SlamGroup = { title: string; rows: SlamRow[]; minLevel?: 1 | 2 | 3 | 4 };
export type SlamVariant = {
  key: string; label: string; slamVersion: number;
  estop?: boolean;
  groups: (ctx: { st?: SlamUiState; hlc: (on: boolean) => void; view: SlamGroup }) => SlamGroup[];
};
export const slam = { defaultLabel: 'Default', variants: [] as SlamVariant[], coreGroups: [] as ((st?: SlamUiState) => SlamGroup)[] };

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

export type DemoDriver = {
  start: (conn: { onMessage: (data: string) => void }) => void;
  rest: (path: string, method?: string, body?: Record<string, any>) => any;
  ptzQc: (method: string, path: string, body: Record<string, any>) => any;
  slam: {
    start: (sink: { state: (s: SlamnavState) => void; cloud: (c: CloudSet) => void; log: (text: string) => void }) => void;
    stop: () => void;
    mapCloud: (stride: number) => MapCloud;
    topoJson: () => any;
    pathJson: () => any;
    command: (path: string, body: Record<string, any>) => { status: number; body: Record<string, unknown> };
  };
};
export const demoMode: { driver?: DemoDriver } = {};

export type PeerService = 'motion' | 'vision' | 'slamnav';
export type RemoteNet = {
  active(): boolean;
  key(): string;
  pass(clientId: string): Promise<string>;
  noPassText(): string;
  waitStatus: string;
  iceServers(): Promise<IceServer[]>;
  exchange(s: PeerService, a: { clientId: string; sdp: string; token: string; pass: string }): Promise<{ sdp: string }>;
};
export type RemoteRobots = {
  lastVia: string;
  has(p: RobotProfile): boolean;
  idOf(p: RobotProfile): string | undefined;
  target(p: RobotProfile): Target | null;
  merge(local: RobotProfile[]): RobotProfile[];
  useMerge(local: RobotProfile[]): RobotProfile[];
  canConnect(): boolean;
  apply(t: Target): void;
  useConnectedId(): string;
};
export type RemoteAccountUi = {
  useInitial(): string | null;
  useSignedIn(): boolean;
  Popover: ComponentType<AccountMenuProps>;
  codeNeeded: { status: string; button: string };
  addRobotSub: string;
  login(): void;
  useReady(): boolean;
  entry(i: { hasRobots: boolean }): string | undefined;
};
export const remote: { net?: RemoteNet; robots?: RemoteRobots; account?: RemoteAccountUi } = {};
export const remoteActive = (): boolean => !!remote.net?.active();
