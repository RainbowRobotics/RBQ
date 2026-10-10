import { useTelemetry } from '@/store/telemetry';
import { parseSensorStates, isValidSensorFrame, VISION_SIZEOF, VISION_REQ } from './sensorState';
import { isRequestEcho, parseRequestEcho, SIZEOF } from './robotState';
import { parseArucoState, ID_ARUCO_STATE, ARUCO_STATE_SIZE } from './arucoState';
import { parseSlamState, ID_SLAM_STATE, SLAM_STATE_SIZE } from './slamState';
import { parseVisionProgramStates, ID_VISION_PROGRAM_STATES, VISION_PROGRAM_STATES_SIZE } from './visionProgramStates';
import { parseP2gState, ID_P2G_STATE, P2G_STATE_SIZE } from './p2gState';
import { handleCameraCalibFrame, ID_CAMERA_CALIB, CAMERA_CALIB_SIZE } from './cameraCalib';

const VS_MAGIC = 0x5356;
const ID_SENSOR_STATES = 20;
const ID_REQUEST_ANSWER = 25;

export function handleVisionStateFrame(data: ArrayBuffer) {
  if (data.byteLength < 3) return;
  const dv = new DataView(data);
  if (dv.getUint16(0, true) !== VS_MAGIC) return;
  const id = dv.getUint8(2);
  const off = 3;
  const len = data.byteLength - off;
  const tel = useTelemetry.getState();
  if (id === ID_SENSOR_STATES) {
    if (len >= VISION_SIZEOF.SENSOR_STATES && isValidSensorFrame(data, off)) {
      try { tel.applySensorStates(parseSensorStates(data, off, tel.sensorLayout)); } catch {}
    }
  } else if (id === ID_ARUCO_STATE) {
    if (len === ARUCO_STATE_SIZE) {
      try { tel.applyMarkerState(parseArucoState(data, off)); } catch {}
    }
  } else if (id === ID_P2G_STATE) {
    if (len === P2G_STATE_SIZE) {
      try { tel.applyP2gState(parseP2gState(data, off)); } catch {}
    }
  } else if (id === ID_SLAM_STATE) {
    if (len === SLAM_STATE_SIZE) {
      try { tel.applySlamState(parseSlamState(data, off)); } catch {}
    }
  } else if (id === ID_VISION_PROGRAM_STATES) {
    tel.noteVisionProgramWire(len);
    if (len === VISION_PROGRAM_STATES_SIZE) {
      try { tel.applyVisionPrograms(parseVisionProgramStates(data, off)); } catch {}
    }
  } else if (id === ID_CAMERA_CALIB) {
    if (len === CAMERA_CALIB_SIZE) {
      try { handleCameraCalibFrame(data, off); } catch {}
    }
  } else if (id === ID_REQUEST_ANSWER) {
    if (len >= SIZEOF.REQUEST && isRequestEcho(data, off)) {
      const { requestID, ok, floats } = parseRequestEcho(data, off);
      if (!ok) return;
      const pose = floats as [number, number, number, number, number, number];
      if (requestID === VISION_REQ.DOOR_HANDLE_POSE) tel.applyDoorHandlePose(pose);
      else if (requestID === VISION_REQ.DOOR_POSE) tel.applyDoorPose(pose);
    }
  }
}

(globalThis as any).__rbqVisionFrame = handleVisionStateFrame;
