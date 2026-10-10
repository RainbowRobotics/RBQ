import { useTelemetry } from '@/store/telemetry';
import { useWallMap } from '@/lib/wallMap';
import type { SensorState } from '@/lib/sensorState';

export type VisionSource = {
  key: string;
  kind: 'pose3d' | 'camera' | 'sim' | 'obsmap';
  label: string;
  streamId?: number;
  requires: string[];
};

export const POSE3D_SOURCE: VisionSource = { key: 'pose3d', kind: 'pose3d', label: '3D', requires: [] };

export const SIM_SOURCE: VisionSource = { key: 'sim', kind: 'sim', label: '물리 시뮬', requires: [] };

export const DOCK_STREAM_ID = 15;
export const NO_STREAM_ID = 0;

export const CAMERA_SOURCES: VisionSource[] = [
  { key: 'front', kind: 'camera', label: '전방', streamId: 1, requires: ['FT0'] },
  { key: 'rear', kind: 'camera', label: '후방', streamId: 2, requires: ['RR0'] },
  { key: 'left', kind: 'camera', label: '좌측', streamId: 3, requires: ['LT0'] },
  { key: 'right', kind: 'camera', label: '우측', streamId: 4, requires: ['RT0'] },
  { key: 'stairs', kind: 'camera', label: '계단', streamId: 13, requires: ['FT0', 'RR0', 'BT0', 'BT1', 'BT2', 'BT3'] },
  { key: 'stacked', kind: 'camera', label: '4분할', streamId: 11, requires: ['FT0', 'RR0', 'LT0', 'RT0'] },
  { key: 'panorama', kind: 'camera', label: '파노라마', streamId: 12, requires: ['FT0', 'RR0', 'LT0', 'RT0'] },
  { key: 'cctv', kind: 'camera', label: 'CCTV', streamId: 31, requires: ['CTV'] },
  { key: 'thermal', kind: 'camera', label: '열화상', streamId: 32, requires: ['TML'] },
  { key: 'ptzmix', kind: 'camera', label: 'PTZ 합성', streamId: 33, requires: ['CTV', 'TML'] },
  { key: 'handcam', kind: 'camera', label: '핸드캠', streamId: 22, requires: ['HC0'] },
];

export const HANDEYE_STREAM_ID = 21;

export const SLAM_STREAM_ID = 41;
export const SLAM_SOURCE: VisionSource = { key: 'slam', kind: 'camera', label: 'SLAM', streamId: SLAM_STREAM_ID, requires: [] };

export const OBSMAP_SOURCE: VisionSource = { key: 'obsmap', kind: 'obsmap', label: '장애물 지도', requires: [] };

export function sensorActive(s: SensorState): boolean {
  return (s.connected || s.running) && !s.failed;
}

export function deriveAvailableSources(
  sensors?: SensorState[], hasLidar = false, hasWallMap = false,
): VisionSource[] {
  const active = new Set((sensors ?? []).filter(sensorActive).map((s) => s.name));
  const out: VisionSource[] = [POSE3D_SOURCE];
  for (const src of CAMERA_SOURCES) {
    if (src.requires.every((name) => active.has(name))) out.push(src);
  }
  if (hasLidar) out.push(SLAM_SOURCE);
  if (hasWallMap) out.push(OBSMAP_SOURCE);
  out.push(SIM_SOURCE);
  return out;
}

export function useAvailableSources(): VisionSource[] {
  const sensors = useTelemetry((s) => s.sensors);
  const hasLidar = useTelemetry((s) => !!s.devices?.[14]?.connected);
  const hasWallMap = useWallMap((s) => !!s.frame);
  return deriveAvailableSources(sensors, hasLidar, hasWallMap);
}
