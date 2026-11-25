export const ID_VISION_PROGRAM_STATES = 23;
export const VISION_PROGRAM_STATES_SIZE = 2760;

const MAX_SLOTS = 25;
const ERROR_MSG_SIZE = 96;
const OFF = { running: 108, threadRunning: 133, gateMode: 160, errorCode: 260, errorMsg: 360 };

export type VisionProgramState = {
  running: boolean;
  threadRunning: boolean;
  gateMode: number;
  errorCode: number;
  errorMsg: string;
};

export function parseVisionProgramStates(buf: ArrayBuffer, byteOffset = 0): VisionProgramState[] {
  const dv = new DataView(buf, byteOffset, VISION_PROGRAM_STATES_SIZE);
  const out: VisionProgramState[] = new Array(MAX_SLOTS);
  for (let i = 0; i < MAX_SLOTS; i++) {
    let msg = '';
    for (let k = 0; k < ERROR_MSG_SIZE; k++) {
      const c = dv.getUint8(OFF.errorMsg + i * ERROR_MSG_SIZE + k);
      if (c === 0) break;
      msg += String.fromCharCode(c);
    }
    out[i] = {
      running: dv.getUint8(OFF.running + i) !== 0,
      threadRunning: dv.getUint8(OFF.threadRunning + i) !== 0,
      gateMode: dv.getInt32(OFF.gateMode + i * 4, true),
      errorCode: dv.getInt32(OFF.errorCode + i * 4, true),
      errorMsg: msg,
    };
  }
  return out;
}

export type ProgramHealth = 'run' | 'idle' | 'stop' | 'error';

export function programHealth(st?: VisionProgramState): ProgramHealth | null {
  if (!st) return null;
  if (st.errorCode !== 0) return 'error';
  if (!st.running) return 'stop';
  return st.threadRunning ? 'run' : 'idle';
}
