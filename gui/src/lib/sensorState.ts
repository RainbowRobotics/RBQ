
export const VISION_REQ = {
  SENSOR_STATES: 10001, DOOR_HANDLE_POSE: 200, DOOR_POSE: 201,
  HAL_START_OAK_CALIB: 2002, HAL_STOP_CAMERA_CALIB: 2003, HAL_START_JIG_CALIB: 2004,
} as const;
export const VISION_SIZEOF = { SENSOR_STATES: 360, SENSOR_STATES_440: 440 } as const;

const SENSOR_BASE = 32;
const STRIDE_360 = 16;
const STRIDE_440 = 20;
const MAX_SENSORS = 20;
const isAlnum = (c: number) => (c >= 48 && c <= 57) || (c >= 65 && c <= 90) || (c >= 97 && c <= 122);

export type SensorLayout = 'legacy118' | 'current';

export type SensorState = {
  name: string;
  attached: boolean;
  powered: boolean;
  detected: boolean;
  connected: boolean;
  idle: boolean;
  sleep: boolean;
  running: boolean;
  failed: boolean;
  rgb: boolean;
  rgbOn: boolean;
  ir: boolean;
  irOn: boolean;
  depth: boolean;
  depthOn: boolean;
  projector: boolean;
  projectorOn: boolean;
  sensorEnabled: boolean;
  commUsb: boolean;
  commLan: boolean;
  camCalibRunning: boolean;
  camCalibSuccess: boolean;
  day: boolean;
  night: boolean;
  zoom: number;
  errorId: number;
  fps: [number, number, number];
};

function bit(b: number, n: number): boolean {
  return (b & (1 << n)) !== 0;
}

export function isValidSensorFrame(buf: ArrayBuffer, byteOffset = 0, total: number = VISION_SIZEOF.SENSOR_STATES, stride: number = STRIDE_360): boolean {
  if (buf.byteLength - byteOffset < total) return false;
  const dv = new DataView(buf, byteOffset, total);
  const n = dv.getUint8(31);
  if (n < 1 || n > MAX_SENSORS) return false;
  const nameOk = (o: number) => {
    if (!isAlnum(dv.getUint8(o))) return false;
    for (let k = 1; k <= 2; k++) {
      const c = dv.getUint8(o + k);
      if (c !== 0 && c !== 0x20 && !isAlnum(c)) return false;
    }
    return true;
  };
  if (!nameOk(SENSOR_BASE + 1)) return false;
  if (n >= 2 && !nameOk(SENSOR_BASE + stride + 1)) return false;
  return true;
}

export function parseSensorStates(
  buf: ArrayBuffer, byteOffset = 0, layout: SensorLayout = 'current',
  total: number = VISION_SIZEOF.SENSOR_STATES, stride: number = STRIDE_360,
): SensorState[] {
  const dv = new DataView(buf, byteOffset, total);
  const n = Math.min(dv.getUint8(31), MAX_SENSORS);
  const is440 = stride === STRIDE_440;
  const legacyBits = layout === 'legacy118' || is440;
  const out: SensorState[] = [];
  for (let i = 0; i < n; i++) {
    const o = SENSOR_BASE + i * stride;
    let name = '';
    for (let k = 1; k <= 3; k++) {
      const c = dv.getUint8(o + k);
      if (c !== 0 && c !== 0x20) name += String.fromCharCode(c);
    }
    const b4 = dv.getUint8(o + 4);
    const b5 = dv.getUint8(o + 5);
    const b6 = dv.getUint8(o + 6);
    const b7 = dv.getUint8(o + 7);
    const attached = bit(b4, 0);
    out.push({
      name,
      attached, powered: bit(b4, 1), detected: bit(b4, 2), connected: bit(b4, 3),
      idle: bit(b4, 4), sleep: bit(b4, 5), running: bit(b4, 6), failed: bit(b4, 7),
      rgb: bit(b5, 0), rgbOn: bit(b5, 1), ir: bit(b5, 2), irOn: bit(b5, 3),
      depth: bit(b5, 4), depthOn: bit(b5, 5), projector: bit(b5, 6), projectorOn: bit(b5, 7),
      ...(legacyBits
        ? { sensorEnabled: bit(b6, 0), commUsb: bit(b6, 1), commLan: bit(b6, 2), day: bit(b7, 1), night: bit(b7, 2),
            camCalibRunning: false, camCalibSuccess: false }
        : { sensorEnabled: attached, commUsb: bit(b6, 0), commLan: bit(b6, 1), day: bit(b7, 0), night: bit(b7, 1),
            camCalibRunning: bit(b6, 2), camCalibSuccess: bit(b6, 3) }),
      zoom: !legacyBits ? dv.getUint8(o + 8) : 0,
      errorId: !legacyBits ? dv.getUint8(o + 9) : 0,
      fps: is440
        ? [dv.getUint8(o + 16), dv.getUint8(o + 17), dv.getUint8(o + 18)]
        : layout === 'current'
          ? [dv.getUint8(o + 12), dv.getUint8(o + 13), dv.getUint8(o + 14)]
          : [0, 0, 0],
    });
  }
  return out;
}

export { STRIDE_360, STRIDE_440 };
