import { describe, it, expect } from 'vitest';
import { deriveAvailableSources, sensorActive } from './visionSources';
import type { SensorState } from './sensorState';

const base: Omit<SensorState, 'name' | 'connected' | 'running'> = {
  attached: false, powered: true, detected: false, idle: false, sleep: false, failed: false,
  rgb: false, rgbOn: false, ir: false, irOn: false, depth: false, depthOn: false,
  projector: false, projectorOn: false, sensorEnabled: false, commUsb: false, commLan: false,
  camCalibRunning: false, camCalibSuccess: false, day: false, night: false, zoom: 0, errorId: 0,
  fps: [0, 0, 0],
};
const S = (name: string, connected: boolean, running: boolean): SensorState =>
  ({ ...base, name, connected, running });

const SNAPSHOT: SensorState[] = [
  S('BT0', true, true), S('BT1', true, true), S('BT2', true, true), S('BT3', true, true),
  S('FT0', true, true), S('RR0', true, true),
  S('LT0', false, false), S('RT0', false, false),
  S('CTV', false, true),
  S('TML', false, false),
  S('SLM', false, false), S('HC0', false, false),
];

describe('sensorActive — PTZ 계열은 running 으로 보고한다', () => {
  it('CTV: connected=false 여도 running 이면 활성', () => {
    expect(sensorActive(S('CTV', false, true))).toBe(true);
  });
  it('TML: 둘 다 false 면 비활성', () => {
    expect(sensorActive(S('TML', false, false))).toBe(false);
  });
  it('failed 면 running 이어도 비활성', () => {
    expect(sensorActive({ ...S('CTV', true, true), failed: true })).toBe(false);
  });
});

describe('deriveAvailableSources — 실기 스냅샷', () => {
  const keys = deriveAvailableSources(SNAPSHOT).map((s) => s.key);

  it('PTZ(CCTV) 뷰가 목록에 뜬다 — 이 PR 이 고친 것', () => {
    expect(keys).toContain('cctv');
  });
  it('전방·후방·계단은 뜬다', () => {
    expect(keys).toEqual(expect.arrayContaining(['front', 'rear', 'stairs']));
  });
  it('없는 장치는 여전히 안 뜬다 — 좌/우측(LT0·RT0), 열화상(TML), 핸드캠(HC0)', () => {
    expect(keys).not.toContain('left');
    expect(keys).not.toContain('right');
    expect(keys).not.toContain('thermal');
    expect(keys).not.toContain('handcam');
  });
  it('PTZ 합성은 열화상이 없어 안 뜬다(CTV+TML 둘 다 필요)', () => {
    expect(keys).not.toContain('ptzmix');
  });
  it('3D 는 항상 있다', () => {
    expect(keys).toContain('pose3d');
  });
});
