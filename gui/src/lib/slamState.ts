export const ID_SLAM_STATE = 21;
export const SLAM_STATE_SIZE = 72;

const STATUS_BASE = 8;
const FLAGS = [
  'lidarConnected', 'synced', 'commandingOnOff', 'recStarted', 'recLoaded',
  'mapBuilding', 'mapSaved', 'mapLoaded', 'mapReloaded',
  'locaInit', 'locaStarted',
  'driveStarted', 'patrolStarted',
  'autoTravel', 'clearTravel', 'clearTopo',
  'quickAnnotation', 'annotationSaved',
  'plotKfrm', 'plotLive', 'plotClip', 'streamEnabled',
  'annotationMode', 'driveMode',
] as const;

const BEEP_OFFSET = 55;

export type SlamFlags = { [K in (typeof FLAGS)[number]]: boolean };
export type SlamState = SlamFlags & {
  beep: boolean;
  driveGoal: number;
  patrolGoal1: number;
  patrolGoal2: number;
  mapColor: number;
  pointSize: number;
  plotView: number;
};

export function parseSlamState(buf: ArrayBuffer, byteOffset = 0): SlamState {
  const dv = new DataView(buf, byteOffset, SLAM_STATE_SIZE);
  const out = {} as SlamState;
  FLAGS.forEach((name, i) => { (out as Record<string, unknown>)[name] = dv.getUint8(STATUS_BASE + i) !== 0; });
  out.beep = dv.getUint8(BEEP_OFFSET) !== 0;
  out.driveGoal = dv.getInt32(56, true);
  out.patrolGoal1 = dv.getInt32(60, true);
  out.patrolGoal2 = dv.getInt32(64, true);
  out.mapColor = dv.getUint8(68);
  out.pointSize = dv.getUint8(69);
  out.plotView = dv.getUint8(70);
  return out;
}
