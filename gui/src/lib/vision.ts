import { actions } from './rest';
import { PROGRAM, PDU_PORT, buildRequest } from './robotState';
import { sendUserCommand } from './userCommand';
import { visionRequestViaDc } from './commandBus';
import { VISION_REQ } from './sensorState';
import { simEngine } from '@/lib/simEngine';

const REQ_ID = {
  PTZ_PAN_TILT_ZOOM_PERCENTAGE: 12001,
  PTZ_PAN_TILT_ZOOM_VELOCITY_DEG: 12006,
  PTZ_ZOOM_X: 12009,
  STREAMER_ZOOM_CCTV: 1020,
  STREAMER_PTZ_FACE_TEMP: 1030,
} as const;

const DAEMON_SERIAL_MSG = 601;

function buildGeneralRequest(
  requestID: number,
  c?: { ints?: number[]; floats?: number[]; bools?: boolean[] },
): Uint8Array {
  const buf = new ArrayBuffer(144);
  const dv = new DataView(buf);
  dv.setInt32(32, requestID, true);
  dv.setUint8(37, 10);
  (c?.bools ?? []).forEach((v, i) => dv.setUint8(38 + i, v ? 1 : 0));
  (c?.ints ?? []).forEach((v, i) => dv.setInt32(60 + i * 4, v, true));
  (c?.floats ?? []).forEach((v, i) => dv.setFloat32(100 + i * 4, v, true));
  dv.setUint8(140, 2);
  dv.setUint8(141, 128);
  dv.setUint8(142, 127);
  return new Uint8Array(buf);
}

export function buildVisionRequest(
  requestID: number,
  c?: { ints?: number[]; floats?: number[]; bools?: boolean[] },
): Uint8Array {
  const buf = new ArrayBuffer(120);
  const dv = new DataView(buf);
  dv.setUint8(0, 255);
  dv.setUint8(1, 254);
  dv.setUint8(2, 2);
  dv.setInt32(4, requestID, true);
  dv.setUint32(8, 1, true);
  (c?.bools ?? []).forEach((v, i) => dv.setUint8(14 + i, v ? 1 : 0));
  (c?.ints ?? []).forEach((v, i) => dv.setInt32(36 + i * 4, v, true));
  (c?.floats ?? []).forEach((v, i) => dv.setFloat32(76 + i * 4, v, true));
  dv.setUint8(116, 128);
  dv.setUint8(117, 127);
  return new Uint8Array(buf);
}


export function sendVisionRequest(frame: Uint8Array, label = 'PTZ·CCTV'): boolean {
  if (simEngine.active) {
    simEngine.hwNote(label);
    return false;
  }
  return visionRequestViaDc(frame);
}

let reqSeq = 0;
export const visionRequest = {
  calcDoorHandlePose: (x: number, y: number, r: number, z = 0.05) =>
    sendVisionRequest(buildRequest(VISION_REQ.DOOR_HANDLE_POSE, ++reqSeq, [x, y, r, z]), '문 좌표'),
  calcDoorPose: (x: number, y: number, r: number, z = 0.05) =>
    sendVisionRequest(buildRequest(VISION_REQ.DOOR_POSE, ++reqSeq, [x, y, r, z]), '문 좌표'),
  startOakCalib: () => sendVisionRequest(buildRequest(VISION_REQ.HAL_START_OAK_CALIB, ++reqSeq), '카메라 캘리브'),
  startJigCalib: () => sendVisionRequest(buildRequest(VISION_REQ.HAL_START_JIG_CALIB, ++reqSeq), '카메라 캘리브'),
  stopCameraCalib: () => sendVisionRequest(buildRequest(VISION_REQ.HAL_STOP_CAMERA_CALIB, ++reqSeq), '카메라 캘리브'),
};

export const ptz = {
  velocity: (pan: number, tilt: number, zoom: number) =>
    sendVisionRequest(buildGeneralRequest(REQ_ID.PTZ_PAN_TILT_ZOOM_VELOCITY_DEG, { floats: [pan, tilt, zoom] })),
  zoomX: (zoom: 1 | 3 | 8 | 16 | 32) =>
    sendVisionRequest(buildGeneralRequest(REQ_ID.PTZ_ZOOM_X, { floats: [zoom] })),
  returnCenter: () =>
    sendVisionRequest(buildGeneralRequest(REQ_ID.PTZ_PAN_TILT_ZOOM_PERCENTAGE, { ints: [50, 50, 0] })),
  faceTemp: (on: boolean) =>
    sendVisionRequest(buildVisionRequest(REQ_ID.STREAMER_PTZ_FACE_TEMP, { bools: [on] })),
};

export const cctv = {
  zoom: (level: 1 | 2 | 4 | 6, x = 0, y = 0) =>
    sendVisionRequest(buildVisionRequest(REQ_ID.STREAMER_ZOOM_CCTV, { ints: [level], floats: [x, y], bools: [true] })),
  dayNight: (_ip: string, day: boolean, both = false) =>
    Promise.resolve(sendUserCommand(PROGRAM.Motion, DAEMON_SERIAL_MSG, [], [0x21, both ? 0 : day ? 1 : 2])),
};

export async function setNightMode(ip: string, night: boolean) {
  await actions.visionDayNight(ip, !night);
  cctv.dayNight(ip, !night).catch(() => {});
  if (!night) actions.pduPower(ip, PDU_PORT.IRLED, false).catch(() => {});
}
