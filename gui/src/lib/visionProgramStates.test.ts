import { describe, it, expect } from 'vitest';
import { parseVisionProgramStates, VISION_PROGRAM_STATES_SIZE } from '@/lib/visionProgramStates';

function frame(): ArrayBuffer {
  const buf = new ArrayBuffer(VISION_PROGRAM_STATES_SIZE);
  const dv = new DataView(buf);
  dv.setUint32(0, 7, true);
  dv.setUint8(108 + 0, 1);
  dv.setUint8(108 + 2, 1);
  dv.setUint8(133 + 2, 1);
  dv.setInt32(160 + 1 * 4, 2, true);
  dv.setInt32(260 + 1 * 4, 42, true);
  const msg = 'camera open failed';
  for (let i = 0; i < msg.length; i++) dv.setUint8(360 + 1 * 96 + i, msg.charCodeAt(i));
  return buf;
}

describe('parseVisionProgramStates', () => {
  it('슬롯별 running/threadRunning/gateMode/errorCode/errorMsg 를 읽는다', () => {
    const p = parseVisionProgramStates(frame());
    expect(p).toHaveLength(25);
    expect(p[0].running).toBe(true);
    expect(p[0].threadRunning).toBe(false);
    expect(p[2]).toMatchObject({ running: true, threadRunning: true });
    expect(p[1]).toMatchObject({ running: false, gateMode: 2, errorCode: 42, errorMsg: 'camera open failed' });
  });

  it('빈 프레임은 전부 정지로 읽힌다(경계값)', () => {
    const p = parseVisionProgramStates(new ArrayBuffer(VISION_PROGRAM_STATES_SIZE));
    expect(p.every((x) => !x.running && !x.threadRunning && x.errorCode === 0 && x.errorMsg === '')).toBe(true);
  });
});
