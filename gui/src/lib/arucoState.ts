export const ID_ARUCO_STATE = 28;
export const ARUCO_STATE_SIZE = 68;

export const MARKER_CAM = { bottom: 2, front: 4, rear: 5 } as const;

export const DISCARD = {
  none: 0,
  markerCnt: 1,
  notDockingMarker: 2,
  solvePnpFail: 3,
  ambiguous: 4,
  noFacing: 5,
  lowQuality: 6,
} as const;

export function discardHint(reason: number): { title: string; action: string } | null {
  if (reason === DISCARD.notDockingMarker)
    return { title: '도킹 스테이션 마커가 아닙니다', action: '카메라 보정판 같은 다른 마커가 보이면 치워 주세요' };
  if (reason === DISCARD.solvePnpFail)
    return { title: '마커 자세를 계산하지 못했습니다', action: '마커가 가려지거나 더럽지 않은지 확인하세요' };
  if (reason === DISCARD.ambiguous || reason === DISCARD.noFacing || reason === DISCARD.lowQuality)
    return { title: '마커 검출 품질이 낮습니다', action: '충전기에 더 가까이 옮기세요' };
  return null;
}

export type MarkerState = {
  detected: boolean;
  cam: number;
  ver: number;
  uMin: number;
  vMin: number;
  uMax: number;
  vMax: number;
  axes: [number, number][];
  ids: number[];
  discard: number;
};

export function parseArucoState(buf: ArrayBuffer, byteOffset = 0): MarkerState {
  const dv = new DataView(buf, byteOffset, ARUCO_STATE_SIZE);
  const axes: [number, number][] = [];
  for (let k = 0; k < 4; k++) {
    axes.push([dv.getFloat32(28 + k * 8, true), dv.getFloat32(32 + k * 8, true)]);
  }
  const ids: number[] = [];
  for (let k = 0; k < 4; k++) {
    const id = dv.getUint8(60 + k);
    if (id !== 0xff) ids.push(id);
  }
  return {
    detected: dv.getUint8(8) !== 0,
    cam: dv.getUint8(9),
    ver: dv.getUint8(10),
    uMin: dv.getFloat32(12, true),
    vMin: dv.getFloat32(16, true),
    uMax: dv.getFloat32(20, true),
    vMax: dv.getFloat32(24, true),
    axes,
    ids,
    discard: dv.getUint8(64),
  };
}
