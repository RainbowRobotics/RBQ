import { describe, it, expect } from 'vitest';
import { parseSensorStates } from './sensorState';
import { worstVerdict, reasonText, usbLabel, usbNote, statusCell, emitCell, fpsCell, defaultView, cameraName } from './cameraCheckView';
import type { CameraCheckSensor, CameraStatus } from './rest';

const status = (over: Partial<CameraStatus> = {}): CameraStatus => ({
  enabled: true, powered: true, detected: true, connected: true, idle: false, sleep: false,
  running: true, failed: false, error_id: 0, projector: false, projector_on: false, ...over,
});
const sensor = (over: Partial<CameraCheckSensor>): CameraCheckSensor => ({
  id: 0, name: 'BT0', type: 'realsense', serial: '', usb: 3,
  status: status(), streams: {}, verdict: 'pass', reason: 'ok', reason_streams: [], ...over,
});

describe('SensorStates USB_LINK', () => {
  it('current 레이아웃은 센서마다 +10 바이트를 usbLink 로 읽는다', () => {
    const buf = new ArrayBuffer(360);
    const dv = new DataView(buf);
    dv.setUint8(31, 2);
    dv.setUint8(32 + 10, 2);
    dv.setUint8(32 + 16 + 10, 3);
    const s = parseSensorStates(buf);
    expect(s.map((x) => x.usbLink)).toEqual([2, 3]);
  });
  it('legacy 레이아웃은 0', () => {
    const buf = new ArrayBuffer(360);
    const dv = new DataView(buf);
    dv.setUint8(31, 1);
    dv.setUint8(32 + 10, 2);
    expect(parseSensorStates(buf, 0, 'legacy118')[0].usbLink).toBe(0);
  });
});

describe('판정 표시', () => {
  it('가장 나쁜 판정 — 꺼진 센서만 있으면 정상', () => {
    expect(worstVerdict([sensor({ verdict: 'skip' })])).toBe('pass');
    expect(worstVerdict([sensor({}), sensor({ verdict: 'skip' }), sensor({ verdict: 'fail' })])).toBe('fail');
  });
  it('frame_drop 은 USB2 면 케이블·포트, 아니면 카메라 점검', () => {
    const drop = { verdict: 'fail' as const, reason: 'frame_drop' as const, reason_streams: ['ir', 'depth'] };
    expect(reasonText(sensor({ ...drop, usb: 2 }))).toContain('케이블·포트');
    expect(reasonText(sensor({ ...drop, usb: 3 }))).not.toContain('케이블·포트');
    expect(reasonText(sensor({ ...drop, usb: 3 }))).toContain('IR·Depth');
  });
  it('USB2 안내는 판정과 별개 — 정상이어도 나오고, frame_drop 이면 사유 문구가 대신한다', () => {
    expect(usbNote(sensor({ usb: 2 }))).toContain('USB2');
    expect(usbNote(sensor({ usb: 3 }))).toBe('');
    expect(usbNote(sensor({ usb: 2, verdict: 'fail', reason: 'frame_drop' }))).toBe('');
  });
  it('USB 표시', () => {
    expect([usbLabel(3), usbLabel(2), usbLabel(0), usbLabel(null)]).toEqual(['3.0', '2.0', '', '']);
  });
});

describe('표 칸 — 대시보드 비전 센서 표와 같은 규칙', () => {
  it('이름은 센서 번호로 풀어 쓴다', () => {
    expect([0, 3, 4, 5].map(cameraName)).toEqual(['Bottom0', 'Bottom3', 'Front', 'Rear']);
  });
  it('STATUS — RUN 이 아니면 막힌 곳', () => {
    expect(statusCell(status()).text).toBe('RUN');
    expect(statusCell(status({ running: false, detected: false })).tone).toBe('warn');
    expect(statusCell(status({ running: false, powered: false })).tone).toBe('fail');
    expect(statusCell(status({ enabled: false })).tone).toBe('off');
  });
  it('FPS·EMIT 칸 — 없는 스트림은 N, 0 은 빨강', () => {
    expect(fpsCell(undefined)).toEqual({ text: 'N', tone: 'off' });
    expect(fpsCell(0).tone).toBe('fail');
    expect(fpsCell(15)).toEqual({ text: '15', tone: 'ok' });
    expect([emitCell(status()), emitCell(status({ projector: true })), emitCell(status({ projector: true, projector_on: true }))])
      .toEqual(['N', 'OFF', 'ON']);
  });
  it('팝업 칸 기본 스트림 — RGB → IR → Depth', () => {
    expect(defaultView(['depth', 'ir'])).toBe('ir');
    expect(defaultView(['depth', 'rgb', 'ir'])).toBe('rgb');
    expect(defaultView([])).toBe('auto');
  });
});
